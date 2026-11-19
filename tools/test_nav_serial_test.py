"""Offline tests using a pseudo-terminal as the electrical controller."""

import importlib.util
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock


SCRIPT = Path(__file__).with_name("nav_serial_test.py")
spec = importlib.util.spec_from_file_location("nav_serial_test", SCRIPT)
nav = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = nav
spec.loader.exec_module(nav)
FEEDBACK = bytes.fromhex("78 02 00 00 00 00 00 00 EC 42 2C 01 04 00 00 88")


def run_virtual(extra=(), chunks=(FEEDBACK,)):
    master, slave = os.openpty()
    process = None
    try:
        process = subprocess.Popen(
            [sys.executable, str(SCRIPT), "--port", os.ttyname(slave),
             "--duration", "0.35", "--print-hz", "0", *extra],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        ready, _, _ = select.select([process.stdout], [], [], 3)
        if not ready:
            raise AssertionError("serial script did not start")
        first = process.stdout.readline()
        if not first.startswith("OPEN "):
            raise AssertionError(first + process.communicate(timeout=3)[0])
        for chunk in chunks:
            os.write(master, chunk)
            time.sleep(0.015)
        output, _ = process.communicate(timeout=4)
        captured = bytearray()
        while select.select([master], [], [], 0)[0]:
            part = os.read(master, 4096)
            if not part:
                break
            captured.extend(part)
        return process.returncode, first + output, bytes(captured)
    finally:
        if process is not None:
            if process.poll() is None:
                process.kill()
            process.communicate()
        os.close(master)
        os.close(slave)


class ProtocolTest(unittest.TestCase):
    def test_reference_frames(self):
        self.assertEqual(nav.pack_command(0xED, 2, 1, -.5, .25), bytes.fromhex(
            "FF ED 02 00 00 80 3F 00 00 00 BF 00 00 80 3E 0D"))
        self.assertEqual(nav.pack_command(0xEC, 0, 5, 118, 3), bytes.fromhex(
            "FF EC 00 00 00 A0 40 00 00 EC 42 00 00 40 40 0D"))
        feedback = nav.FeedbackParser().feed(FEEDBACK)[0]
        self.assertEqual((feedback.pitch_deg, feedback.yaw_deg, feedback.current_hp), (0, 118, 300))
        self.assertEqual((feedback.mode, feedback.color_code, feedback.game_progress), (1, 2, 4))

    def test_all_fragment_boundaries(self):
        for split in range(1, 16):
            parser = nav.FeedbackParser()
            self.assertEqual(parser.feed(FEEDBACK[:split]), [])
            self.assertEqual([frame.raw for frame in parser.feed(FEEDBACK[split:])], [FEEDBACK])

    def test_noise_bad_tail_concatenation_and_data_markers(self):
        parser = nav.FeedbackParser()
        bad = FEEDBACK[:-1] + b"\x00"
        with_markers = b"\x78\x01\x78\x88\x0D\xFF" + FEEDBACK[6:]
        frames = parser.feed(b"\xA5\x08\x00" + bad + FEEDBACK + with_markers)
        self.assertEqual([frame.raw for frame in frames], [FEEDBACK, with_markers])
        self.assertEqual(parser.bad_tail_candidates, 1)
        self.assertEqual(parser.discarded_bytes, 19)
        self.assertEqual(len(parser.buffer), 0)

    def test_non_finite_is_reported(self):
        frame = bytearray(FEEDBACK)
        frame[2:6] = struct.pack("<f", float("nan"))
        self.assertFalse(nav.FeedbackParser().feed(frame)[0].finite)

    def test_marker_counts_cover_all_bytes_even_when_frames_are_rejected(self):
        parser = nav.FeedbackParser()
        parser.feed(b"\xA5\xFF\x0D" + FEEDBACK[:4])
        parser.feed(FEEDBACK[4:] + b"\x88")
        self.assertEqual(parser.marker_counts, {0x78: 1, 0x88: 2, 0xA5: 1, 0xFF: 1, 0x0D: 1})


class CliTest(unittest.TestCase):
    def test_dry_run_does_not_open_nonexistent_port(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--port", "/no/such/port", "--send", "chassis", "--dry-run"],
            capture_output=True, text=True, timeout=3,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("FF ED 02", result.stdout)

    def test_missing_angles_and_nonfinite_or_ignored_parameters_rejected(self):
        for args in (["--send", "gimbal"], ["--send", "chassis", "--vx", "nan"],
                     ["--send", "chassis", "--vx", "1e300"], ["--vx", "1"],
                     ["--rate", "0"], ["--duration", "-1"],
                     ["--foxglove-port", "70000"]):
            result = subprocess.run([sys.executable, str(SCRIPT), *args],
                                    capture_output=True, text=True, timeout=3)
            self.assertEqual(result.returncode, 2, (args, result.stdout, result.stderr))

    def test_virtual_serial_receives_and_sends_nothing_by_default(self):
        code, output, sent = run_virtual(chunks=(b"noise" + FEEDBACK[:4], FEEDBACK[4:] + FEEDBACK))
        self.assertEqual(code, 0, output)
        self.assertIn("rx_frames=2", output)
        self.assertIn("hp=300", output)
        self.assertIn("SUMMARY ", output)
        self.assertIn("finite_angle_frames=2", output)
        self.assertEqual(sent, b"")

    def test_foxglove_session_starts_bridge_opens_studio_and_stops_bridge(self):
        bridge_process = mock.Mock()
        bridge_process.poll.return_value = None
        bridge_process.wait.return_value = 0
        with mock.patch.object(nav.shutil, "which", side_effect=lambda name: "/usr/bin/" + name), \
                mock.patch.object(nav.subprocess, "Popen", side_effect=[bridge_process, mock.Mock()]) as popen, \
                mock.patch.object(nav.socket, "create_connection") as connect, \
                mock.patch.dict(os.environ, {"DISPLAY": ":1"}):
            connect.return_value.__enter__.return_value = mock.Mock()
            with nav.FoxgloveBridge(8765):
                pass

        self.assertEqual(popen.call_count, 2)
        bridge_command = popen.call_args_list[0].args[0]
        self.assertEqual(bridge_command[-1], "port:=8765")
        open_command = popen.call_args_list[1].args[0]
        self.assertEqual(open_command[0], "/usr/bin/xdg-open")
        self.assertIn("ds=foxglove-websocket", open_command[1])
        self.assertIn("ds.url=ws%3A%2F%2Flocalhost%3A8765", open_command[1])
        bridge_process.terminate.assert_called_once()
        bridge_process.wait.assert_called_once_with(timeout=3)

    def test_virtual_serial_chassis_and_final_zero_frames(self):
        code, output, sent = run_virtual(["--send", "chassis", "--count", "1", "--vx", "0.1"])
        self.assertEqual(code, 0, output)
        self.assertEqual(sent, nav.pack_command(0xED, 2, .1, 0, 0)
                         + nav.pack_command(0xED, 2, 0, 0, 0) * 3)
        self.assertIn("TX_UNCONFIRMED", output)

    def test_virtual_serial_gimbal_and_final_fire_zero(self):
        code, output, sent = run_virtual([
            "--send", "gimbal", "--count", "1", "--pitch", "5", "--yaw", "118", "--fire", "1"])
        self.assertEqual(code, 0, output)
        self.assertEqual(sent, nav.pack_command(0xEC, 1, 5, 118, 3)
                         + nav.pack_command(0xEC, 0, 5, 118, 3) * 3)

    def test_no_feedback_and_old_seasky_do_not_claim_success(self):
        for chunks in ((), (bytes.fromhex("A5 08 00 F4 00 00 EC 42 E1 7A 14 3F C1 3D"),)):
            code, output, _ = run_virtual(chunks=chunks)
            self.assertEqual(code, 2, output)
            self.assertIn("RX_NO_FRAME", output)

    def test_capture_preserves_all_bytes_and_replay_decodes_without_device(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "rx.bin"
            chunks = (b"\xA5\xFF\x0D" + FEEDBACK[:5], FEEDBACK[5:] + FEEDBACK)
            code, output, _ = run_virtual(["--capture", str(path)], chunks=chunks)
            self.assertEqual(code, 0, output)
            self.assertEqual(path.read_bytes(), b"".join(chunks))
            result = subprocess.run(
                [sys.executable, str(SCRIPT), "--port", "/no/such/port", "--replay", str(path)],
                capture_output=True, text=True, timeout=3)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            self.assertIn("rx_frames=2", result.stdout)
            self.assertIn("78=2 88=2 A5=1 FF=1 0D=1", result.stdout)
            result = subprocess.run(
                [sys.executable, str(SCRIPT), "--capture", str(path)],
                capture_output=True, text=True, timeout=3)
            self.assertEqual(result.returncode, 1, result.stdout)
            self.assertEqual(path.read_bytes(), b"".join(chunks))

    def test_replay_rejects_nonmatching_frames_and_send_options(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "old.bin"
            path.write_bytes(bytes.fromhex("A5 08 00 F4 00 00 EC 42 E1 7A 14 3F C1 3D"))
            result = subprocess.run([sys.executable, str(SCRIPT), "--replay", str(path)],
                                    capture_output=True, text=True, timeout=3)
            self.assertEqual(result.returncode, 2, result.stdout)
            self.assertIn("RX_NO_FRAME", result.stdout)
            result = subprocess.run([sys.executable, str(SCRIPT), "--replay", str(path),
                                     "--send", "chassis"],
                                    capture_output=True, text=True, timeout=3)
            self.assertEqual(result.returncode, 2, result.stdout)

    def test_ctrl_c_sends_final_zero_frames(self):
        master, slave = os.openpty()
        process = subprocess.Popen(
            [sys.executable, str(SCRIPT), "--port", os.ttyname(slave), "--send", "chassis",
             "--duration", "0", "--count", "1"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        try:
            self.assertTrue(select.select([process.stdout], [], [], 3)[0])
            self.assertTrue(process.stdout.readline().startswith("OPEN "))
            self.assertTrue(select.select([master], [], [], 3)[0])
            first = os.read(master, 4096)
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 130, output)
            self.assertIn("SUMMARY ", output)
            self.assertIn("finish_tx=3", output)
            sent = bytearray(first)
            while select.select([master], [], [], 0)[0]:
                sent.extend(os.read(master, 4096))
            self.assertEqual(bytes(sent), nav.pack_command(0xED, 2, 0, 0, 0) * 4)
        finally:
            if process.poll() is None:
                process.kill()
            process.communicate()
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    unittest.main()
