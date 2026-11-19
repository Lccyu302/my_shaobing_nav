#!/usr/bin/env python3
"""Test the navigation project's 16-byte serial protocol on Linux.

Uses only Python's standard library unless --ros or --foxglove is selected. No
ROS serial driver should be running on the same port. See tools/README.md.
"""

import argparse
import errno
import glob
import json
import math
import os
import select
import shutil
import socket
import struct
import subprocess
import sys
import time
from contextlib import ExitStack
from dataclasses import dataclass
from urllib.parse import urlencode


FRAME_SIZE = 16


def hex_bytes(data):
    return data.hex(" ").upper()


def log(message):
    print(message, flush=True)


def describe_feedback(feedback, count):
    return ("RX_FRAME #{} {} | pitch_deg={:.7g} yaw_deg={:.7g} hp={} "
            "game={} hurt={} color={} mode={} reserved={}{}".format(
                count, hex_bytes(feedback.raw), feedback.pitch_deg,
                feedback.yaw_deg, feedback.current_hp, feedback.game_progress,
                feedback.hurt_type, feedback.color_code, feedback.mode,
                feedback.reserved, " NON_FINITE_ANGLE" if not feedback.finite else ""))


def pack_command(command, parameter, a, b, c):
    """ED: type/vx/vy/wz; EC: fire_advice/pitch_deg/yaw_deg/distance_m."""
    return struct.pack("<BBBfffB", 0xFF, command, parameter, a, b, c, 0x0D)


@dataclass
class Feedback:
    raw: bytes
    color_code: int
    pitch_deg: float
    yaw_deg: float
    current_hp: int
    game_progress: int
    hurt_type: int
    reserved: int

    @property
    def mode(self):
        return 1 if self.color_code == 2 else 0

    @property
    def finite(self):
        return math.isfinite(self.pitch_deg) and math.isfinite(self.yaw_deg)


class FeedbackParser:
    """Recover 78...88 frames from fragmented or concatenated serial reads."""

    def __init__(self):
        self.buffer = bytearray()
        self.discarded_bytes = 0
        self.bad_tail_candidates = 0
        self.marker_counts = dict.fromkeys((0x78, 0x88, 0xA5, 0xFF, 0x0D), 0)

    def feed(self, data):
        for marker in self.marker_counts:
            self.marker_counts[marker] += data.count(marker)
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            start = self.buffer.find(b"\x78")
            if start < 0:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < FRAME_SIZE:
                break
            if self.buffer[15] != 0x88:
                self.bad_tail_candidates += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            raw = bytes(self.buffer[:FRAME_SIZE])
            del self.buffer[:FRAME_SIZE]
            frames.append(Feedback(raw, *struct.unpack_from("<BffHBBB", raw, 1)))
        return frames


def report_markers(parser):
    log("RX_MARKERS " + " ".join("{:02X}={}".format(marker, count)
                                   for marker, count in parser.marker_counts.items()))


def replay_file(path):
    parser = FeedbackParser()
    rx_bytes = rx_count = finite_count = 0
    preview = bytearray()
    log("REPLAY {} (no serial port opened)".format(path))
    with open(path, "rb") as source:
        while True:
            chunk = source.read(4096)
            if not chunk:
                break
            preview.extend(chunk[:max(0, 128 - len(preview))])
            rx_bytes += len(chunk)
            for feedback in parser.feed(chunk):
                rx_count += 1
                finite_count += int(feedback.finite)
                if rx_count <= 10:
                    log(describe_feedback(feedback, rx_count))
    for offset in range(0, len(preview), 16):
        log("RX_PREVIEW {:08X}: {}".format(offset, hex_bytes(preview[offset:offset + 16])))
    log("REPLAY_SUMMARY rx_bytes={} rx_frames={} finite_angle_frames={} discarded_bytes={} "
        "bad_tail_candidates={} pending_bytes={}".format(
            rx_bytes, rx_count, finite_count, parser.discarded_bytes,
            parser.bad_tail_candidates, len(parser.buffer)))
    report_markers(parser)
    if not rx_count:
        log("RX_NO_FRAME: 文件内未匹配到 78...88 的 16 字节上行帧。标识字节计数不能单独证明协议类型。")
    return 0 if finite_count else 2


class LinuxSerial:
    def __init__(self, path, baud):
        self.path, self.baud = path, baud
        self.fd = None
        self.original_settings = None

    def __enter__(self):
        import fcntl
        import termios

        speed = getattr(termios, "B" + str(self.baud), None)
        if speed is None:
            raise ValueError("当前系统不支持波特率 {}".format(self.baud))
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            # Prevent subsequent opens; an already running driver must be stopped first.
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            self.original_settings = termios.tcgetattr(self.fd)
            settings = termios.tcgetattr(self.fd)
            settings[0] = 0  # raw input, including 0D/11/13
            settings[1] = 0
            settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            settings[3] = 0
            settings[4] = speed
            settings[5] = speed
            settings[6][termios.VMIN] = 0
            settings[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        except termios.error as error:
            self.__exit__(None, None, None)
            raise OSError("串口配置失败：{}".format(error)) from error
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def write(self, frame):
        deadline = time.monotonic() + 1.0
        offset = 0
        while offset < len(frame):
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                raise TimeoutError("串口写入超时")
            try:
                count = os.write(self.fd, frame[offset:])
            except BlockingIOError:
                continue
            if count <= 0:
                raise OSError("串口写入失败")
            offset += count

    def __exit__(self, *_):
        import fcntl
        import termios

        if self.fd is not None:
            try:
                if self.original_settings is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.original_settings)
            except (OSError, termios.error):
                pass
            try:
                fcntl.ioctl(self.fd, termios.TIOCNXCL)
            except OSError:
                pass
            os.close(self.fd)
            self.fd = None


class RosOutput:
    """Publish standard ROS 2 messages for Foxglove; no workspace build needed."""

    def __init__(self):
        try:
            import rclpy
            from rclpy.signals import SignalHandlerOptions
            from std_msgs.msg import Float32, String, UInt8, UInt16, UInt64
        except ImportError as error:
            raise RuntimeError(
                "--ros 需要 ROS 2 Python 环境，请先执行 source /opt/ros/humble/setup.bash"
            ) from error
        self.rclpy = rclpy
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("nav_serial_test")
        types = {
            "pitch_deg": Float32, "yaw_deg": Float32,
            "current_hp": UInt16, "color_code": UInt8, "mode": UInt8,
            "game_progress": UInt8, "hurt_type": UInt8, "reserved": UInt8,
            "rx_count": UInt64, "tx_count": UInt64, "discarded_bytes": UInt64,
            "rx_frame_hex": String, "rx_bytes_hex": String,
            "tx_frame_hex": String, "status": String,
        }
        self.publishers = {
            name: (self.node.create_publisher(kind, "/serial_test/" + name, 10), kind)
            for name, kind in types.items()
        }

    def publish(self, name, value):
        publisher, kind = self.publishers[name]
        message = kind()
        message.data = value
        publisher.publish(message)

    def feedback(self, frame, count):
        for field in ("pitch_deg", "yaw_deg", "current_hp", "color_code", "mode",
                      "game_progress", "hurt_type", "reserved"):
            self.publish(field, getattr(frame, field))
        self.publish("rx_frame_hex", hex_bytes(frame.raw))
        self.publish("rx_count", count)

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=0)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


class FoxgloveBridge:
    """Own the ROS bridge and open the local Foxglove desktop connection."""

    def __init__(self, port):
        self.port = port
        self.process = None
        self.url = "ws://localhost:{}".format(port)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.close()

    def start(self):
        ros2 = shutil.which("ros2")
        if ros2 is None:
            raise RuntimeError("找不到 ros2；请通过 tools/run_nav_serial_test.sh 启动或先加载 ROS 2 环境")
        self.process = subprocess.Popen(
            [ros2, "launch", "foxglove_bridge", "foxglove_bridge_launch.xml",
             "port:={}".format(self.port)],
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        )
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.process = None
                raise RuntimeError("foxglove_bridge 启动失败；检查是否已安装或端口 {} 被占用".format(
                    self.port))
            try:
                with socket.create_connection(("127.0.0.1", self.port), timeout=0.2):
                    break
            except OSError:
                time.sleep(0.1)
        else:
            self.close()
            raise RuntimeError("等待 foxglove_bridge 超时；检查端口 {} 和 ROS 环境".format(
                self.port))

        log("FOXGLOVE_BRIDGE_READY {}".format(self.url))
        opener = shutil.which("xdg-open")
        if opener and (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            deep_link = "foxglove://open?{}".format(urlencode({
                "ds": "foxglove-websocket", "ds.url": self.url,
            }))
            try:
                subprocess.Popen([opener, deep_link], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                log("FOXGLOVE_OPENING: 已请求 Foxglove Studio 连接 {}".format(self.url))
            except OSError as error:
                log("FOXGLOVE_URL: {} (自动打开失败: {})".format(self.url, error))
        else:
            log("FOXGLOVE_URL: {} (在 Foxglove 中连接此地址)".format(self.url))

    def close(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.process = None


def finite_number(text):
    try:
        value = float(text)
        if not math.isfinite(value):
            raise ValueError()
        return value
    except ValueError as error:
        raise argparse.ArgumentTypeError("请输入有限数值") from error


def float32_number(text):
    value = finite_number(text)
    try:
        struct.pack("<f", value)
    except (OverflowError, struct.error) as error:
        raise argparse.ArgumentTypeError("数值超出 float32 范围") from error
    return value


def arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="导航 16 字节串口双向测试：默认只接收；--foxglove 一键启动实时面板。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--list-ports", action="store_true", help="列出串口，不打开设备")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="串口路径")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，8N1、无流控")
    parser.add_argument("--duration", type=finite_number, default=10.0,
                        help="运行秒数；0 表示一直运行到 Ctrl+C")
    parser.add_argument("--send", choices=("chassis", "gimbal"), help="发送测试帧；默认只收")
    parser.add_argument("--rate", type=finite_number, default=10.0, help="发送频率，Hz")
    parser.add_argument("--count", type=int, default=0,
                        help="最多发送几帧；0 表示运行期间持续发送；结束前补发帧另计")
    parser.add_argument("--chassis-type", type=int, choices=(1, 2, 3, 4), default=2)
    parser.add_argument("--vx", type=float32_number, default=0.0, help="底盘 vx，m/s")
    parser.add_argument("--vy", type=float32_number, default=0.0, help="底盘 vy，m/s")
    parser.add_argument("--wz", type=float32_number, default=0.0, help="底盘 wz，rad/s")
    parser.add_argument("--pitch", type=float32_number, help="云台目标仰角，度；水平为 0")
    parser.add_argument("--yaw", type=float32_number, help="云台目标 yaw，度")
    parser.add_argument("--distance", type=float32_number, default=3.0, help="目标距离，m")
    parser.add_argument("--fire", type=int, choices=(0, 1), default=0, help="EC 开火建议")
    parser.add_argument("--dry-run", action="store_true", help="仅显示待发帧，不打开串口")
    parser.add_argument("--raw", action="store_true", help="打印每次读取的全部原始字节")
    parser.add_argument("--capture", metavar="FILE", help="将全部收到的字节保存为二进制文件，不覆盖已有文件")
    parser.add_argument("--replay", metavar="FILE", help="离线解析 --capture 保存的文件，不打开串口")
    parser.add_argument("--print-hz", type=finite_number, default=5.0,
                        help="终端收发显示频率上限；0 表示全部；不限制 ROS 发布")
    parser.add_argument("--ros", action="store_true", help="向 /serial_test/* 发布 ROS 2 话题")
    parser.add_argument("--foxglove", action="store_true",
                        help="启动 foxglove_bridge、发布 ROS 话题并自动打开 Foxglove Studio")
    parser.add_argument("--foxglove-port", type=int, default=8765,
                        help="foxglove_bridge WebSocket 端口")
    args = parser.parse_args(argv)
    if (args.duration < 0 or args.rate <= 0 or args.count < 0 or args.print_hz < 0
            or not 1 <= args.foxglove_port <= 65535):
        parser.error("duration/count/print-hz 必须非负，rate 必须大于 0，foxglove-port 范围为 1..65535")
    if args.dry_run and args.send is None:
        parser.error("--dry-run 需要指定 --send chassis 或 --send gimbal")
    if args.send == "gimbal" and (args.pitch is None or args.yaw is None):
        parser.error("云台测试需要显式指定 --pitch 和 --yaw，单位都是度")
    if args.send != "gimbal" and (args.pitch is not None or args.yaw is not None or args.fire):
        parser.error("--pitch/--yaw/非零 --fire 仅用于 --send gimbal")
    if args.send != "chassis" and (args.vx or args.vy or args.wz or args.chassis_type != 2):
        parser.error("底盘参数需要 --send chassis")
    if args.replay and (args.send or args.ros or args.foxglove or args.capture
                        or args.list_ports or args.dry_run):
        parser.error("--replay 为离线解析，不能同时使用发送、ROS、采集或列设备选项")
    return args


def command_for(args, finish=False):
    if args.send == "chassis":
        if finish:
            return pack_command(0xED, 2, 0, 0, 0)
        return pack_command(0xED, args.chassis_type, args.vx, args.vy, args.wz)
    if args.send == "gimbal":
        return pack_command(0xEC, 0 if finish else args.fire,
                            args.pitch, args.yaw, args.distance)
    return None


def run(args, ros):
    parser = FeedbackParser()
    tx_count = rx_count = rx_bytes = finite_count = 0
    cleanup_count = 0
    interval = 0 if args.print_hz == 0 else 1.0 / args.print_hz
    last_tx_print = last_rx_print = last_raw_print = -math.inf
    frame = command_for(args)
    interrupted = False
    failure = None
    started = time.monotonic()

    with ExitStack() as resources:
        capture = resources.enter_context(open(args.capture, "xb")) if args.capture else None
        port = resources.enter_context(LinuxSerial(args.port, args.baud))
        log("OPEN {} baud={} 8N1 little-endian RX=78...88 (16 bytes, no CRC)".format(
            args.port, args.baud))
        log("MODE={}{}".format(args.send or "receive-only", " + ROS /serial_test/*" if ros else ""))
        if capture:
            log("CAPTURE {} (保存全部接收字节，不受终端显示频率限制)".format(args.capture))
        started = time.monotonic()
        deadline = started + args.duration if args.duration else math.inf
        next_tx = started
        next_status = started
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if ros:
                    ros.spin()
                    if now >= next_status:
                        ros.publish("status", json.dumps({
                            "mode": args.send or "receive-only", "rx_count": rx_count,
                            "tx_count": tx_count, "rx_bytes": rx_bytes,
                            "discarded_bytes": parser.discarded_bytes,
                            "tx_delivery": "unconfirmed" if frame else "not_requested",
                            "rx_check": "16-byte markers only; no CRC",
                        }, ensure_ascii=False))
                        ros.publish("discarded_bytes", parser.discarded_bytes)
                        ros.publish("tx_count", tx_count)
                        ros.publish("rx_count", rx_count)
                        next_status = now + 1.0
                sending = frame is not None and (args.count == 0 or tx_count < args.count)
                if sending and now >= next_tx:
                    port.write(frame)
                    tx_count += 1
                    if ros:
                        ros.publish("tx_frame_hex", hex_bytes(frame))
                        ros.publish("tx_count", tx_count)
                    if now - last_tx_print >= interval:
                        log("TX_WRITTEN #{} {}".format(tx_count, hex_bytes(frame)))
                        last_tx_print = now
                    next_tx = now + 1.0 / args.rate
                timeout = min(0.1, max(0.0, deadline - time.monotonic()))
                if sending:
                    timeout = min(timeout, max(0.0, next_tx - time.monotonic()))
                if not select.select([port.fd], [], [], timeout)[0]:
                    continue
                try:
                    chunk = os.read(port.fd, 4096)
                except BlockingIOError:
                    continue
                if not chunk:
                    raise OSError("串口已断开或设备没有可读数据")
                if capture:
                    capture.write(chunk)
                rx_bytes += len(chunk)
                now = time.monotonic()
                if ros:
                    ros.publish("rx_bytes_hex", hex_bytes(chunk))
                decoded = parser.feed(chunk)
                if args.raw or (not decoded and now - last_raw_print >= interval):
                    preview = chunk if args.raw else chunk[:64]
                    log("RX_RAW len={} {}{}".format(
                        len(chunk), hex_bytes(preview), " ..." if len(preview) < len(chunk) else ""))
                    last_raw_print = now
                for feedback in decoded:
                    rx_count += 1
                    finite_count += int(feedback.finite)
                    if ros:
                        ros.feedback(feedback, rx_count)
                    if now - last_rx_print >= interval or not feedback.finite:
                        log(describe_feedback(feedback, rx_count))
                        last_rx_print = now
        except KeyboardInterrupt:
            interrupted = True
            log("INTERRUPTED: 停止测试")
        except (OSError, RuntimeError) as error:
            failure = str(error)
            log("ERROR: " + failure)
        finally:
            if frame is not None and tx_count:
                finish_frame = command_for(args, finish=True)
                try:
                    for _ in range(3):
                        port.write(finish_frame)
                        cleanup_count += 1
                        if ros:
                            ros.publish("tx_frame_hex", hex_bytes(finish_frame))
                            ros.publish("tx_count", tx_count + cleanup_count)
                    log("TX_FINISH_WRITTEN #{} {} ({})".format(
                        cleanup_count, hex_bytes(finish_frame),
                        "type=2, vx=vy=wz=0" if args.send == "chassis" else "fire=0, 保持所填目标角"))
                except (OSError, RuntimeError) as error:
                    failure = str(error)
                    log("TX_FINISH_FAILED: " + failure)

    log("SUMMARY elapsed={:.2f}s tx={} finish_tx={} rx_bytes={} rx_frames={} "
        "finite_angle_frames={} discarded_bytes={} bad_tail_candidates={} pending_bytes={}".format(
            time.monotonic() - started, tx_count, cleanup_count, rx_bytes, rx_count,
            finite_count, parser.discarded_bytes, parser.bad_tail_candidates, len(parser.buffer)))
    report_markers(parser)
    if frame:
        log("TX_UNCONFIRMED: TX_WRITTEN 只证明本机写入成功；请电控核对 ED/EC 接收变量。"
            "收到姿态帧也不能证明电控正确解析了下行。")
    if not rx_count:
        log("RX_NO_FRAME: 未收到有效 78...88 帧。检查 RX_RAW、波特率、接线、固件发送逻辑及串口占用。")
    else:
        log("RX_RESULT: 已匹配 16 字节上行帧；协议无 CRC，请与电控实测角度、血量逐项核对。")
    return 130 if interrupted else (1 if failure else (0 if finite_count else 2))


def main(argv=None):
    args = arguments(argv)
    if args.list_ports:
        paths = sorted(set(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
                           + glob.glob("/dev/serial/by-id/*")))
        for path in paths:
            log(path + (" -> " + os.path.realpath(path) if os.path.islink(path) else ""))
        if not paths:
            log("未发现 ttyUSB/ttyACM 串口")
        return 0
    if args.dry_run:
        log("DRY_RUN (16 bytes): " + hex_bytes(command_for(args)))
        return 0
    if not sys.platform.startswith("linux"):
        log("ERROR: 直接串口模式需要 Linux；--dry-run 可在其他平台使用")
        return 1
    ros = None
    bridge = None
    try:
        if args.replay:
            return replay_file(args.replay)
        if args.foxglove:
            bridge = FoxgloveBridge(args.foxglove_port)
            bridge.start()
        if args.ros or args.foxglove:
            ros = RosOutput()
        return run(args, ros)
    except (OSError, ValueError, RuntimeError) as error:
        log("ERROR: " + str(error))
        if isinstance(error, OSError) and error.errno in (errno.EACCES, errno.EBUSY):
            log("请检查串口权限，并停止占用同一串口的 ROS 驱动或其他测试工具。")
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if ros:
            ros.close()
        if bridge:
            bridge.close()


if __name__ == "__main__":
    sys.exit(main())
