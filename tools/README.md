# 导航串口联调与 Foxglove 实时面板

这套工具用于验证导航串口的**双向通信**：小电脑持续接收并解析电控上行反馈，同时可以按需向电控发送下行命令。日常联调默认只接收，不会主动发送底盘或云台命令。收到完整反馈后，脚本会持续发布 ROS 2 话题供 Foxglove 实时显示；退出时保留串口收发统计总结。

串口协议为 16 字节、115200、8N1、小端序：下行 `FF ... 0D`，上行 `78 ... 88`。协议没有 CRC，帧首尾匹配只代表长度和标记符合约定，不代表所有字段都经过校验。

## 一键启动

先确认电控串口设备名。设备名已知时不需要单独执行这一步：

```bash
python3 tools/nav_serial_test.py --list-ports
```

从仓库根目录运行下面这一条命令：

```bash
./tools/run_nav_serial_test.sh --port /dev/ttyACM0
```

启动脚本会自动加载 `/opt/ros/humble/setup.bash`、启动 `foxglove_bridge`、打开串口接收电控反馈、发布 `/serial_test/*` 话题，并请求 Foxglove Studio 连接 `ws://localhost:8765`。测试持续运行到按 `Ctrl+C`，然后打印收发总结。默认不发任何串口命令。

如果 ROS 安装在其他位置，可在同一条启动命令中指定环境文件：

```bash
ROS_SETUP_BASH=/opt/ros/jazzy/setup.bash ./tools/run_nav_serial_test.sh --port /dev/ttyACM0
```

`foxglove_bridge` 未安装时，可按当前 ROS 发行版安装，例如 Humble：

```bash
sudo apt install ros-humble-foxglove-bridge
```

串口由本脚本独占打开。启动前需停止同一端口上的 `rm_serial_driver_node`、导航 bringup 或其他串口程序。若本机没有图形桌面，脚本仍会启动桥接器并打印 WebSocket 地址；在另一台 Foxglove 上连接 `ws://小电脑IP:8765`，小电脑 IP 可用 `hostname -I` 查看。

## 实时面板

如果 Foxglove Studio 中已经保存过串口面板布局，一键启动后会直接打开并连接，布局会由 Foxglove 保存在本机。首次使用时，在 Foxglove 添加并保存下面这些面板即可；之后每次联调都只运行上面的一条启动命令。

| 面板 | 订阅内容 | 查看用途 |
| --- | --- | --- |
| Plot | `/serial_test/pitch_deg.data`、`/serial_test/yaw_deg.data`、`/serial_test/current_hp.data` | 实时比较电控上报的角度和血量变化；角度单位为度 |
| Raw Messages | `/serial_test/status` | 查看模式、收发计数、接收字节数和协议校验范围 |
| Raw Messages | `/serial_test/rx_frame_hex` | 查看解析出的完整 16 字节上行帧 |
| Raw Messages | `/serial_test/rx_bytes_hex` | 查看串口每次读取的原始字节；一次读取可能只有半帧 |
| Raw Messages | `/serial_test/tx_frame_hex` | 仅在启用发送时查看本机写出的下行帧 |
| Raw Messages | `/serial_test/rx_count`、`/serial_test/tx_count`、`/serial_test/discarded_bytes` | 查看完整帧数及解析器丢弃的字节数 |

其余字段包括 `/serial_test/color_code`、`/serial_test/mode`、`/serial_test/game_progress`、`/serial_test/hurt_type` 和 `/serial_test/reserved`。所有话题都是串口诊断数据，不会驱动现有导航节点。Foxglove Plot 使用标准 ROS 消息的 `.data` 字段。

终端显示默认限频到每秒 5 次，ROS 话题仍按实际接收频率发布；`--print-hz 0` 可让终端打印每个完整帧。Foxglove 尚未连上时，串口接收和终端统计仍会继续运行。

## 收发模式

### 只看电控上行数据

一键启动命令默认就是接收模式。正常收到反馈时，终端会打印类似内容：

```text
RX_FRAME #1 78 02 00 00 00 00 00 00 EC 42 2C 01 04 00 00 88 | pitch_deg=0 yaw_deg=118 hp=300 game=4 hurt=0 color=2 mode=1 reserved=0
```

`RX_FRAME` 表示解析到一帧 16 字节上行数据。请将角度、血量和比赛状态与电控侧实测变量对照；由于协议没有 CRC，不能仅凭帧首尾相符断定每个字段都正确。

### 同时向电控发送

确实需要验证下行链路时，可在启动命令后追加发送参数。下面会以 10 Hz 发送零速度底盘命令，同时继续接收电控反馈：

```bash
./tools/run_nav_serial_test.sh --port /dev/ttyACM0 --send chassis
./tools/run_nav_serial_test.sh --port /dev/ttyACM0 --send chassis --vx 0.2 --count 1
```

需要非零速度时才显式填写 `--vx`、`--vy`、`--wz`，单位依次为 m/s、m/s、rad/s；这会实际发送底盘命令。`--count 1` 可只发送一帧，然后继续接收数据直到停止。

云台命令必须显式提供角度，单位都是度；开火建议默认为 0：

```bash
./tools/run_nav_serial_test.sh --port /dev/ttyACM0 --send gimbal --pitch 0 --yaw 118 --distance 3 --fire 0
```

`TX_WRITTEN` 只证明小电脑已把字节写入串口。当前协议没有下行 ACK，收到电控上行反馈也不能证明它解析了下行命令；仍需在电控侧核对接收变量。发送模式退出或按 `Ctrl+C` 后会尝试补发 3 帧安全结束命令：底盘零速度，或云台保持目标角并清除开火建议。

如只想检查生成的字节、不打开串口，可使用纯 Python 命令：

```bash
python3 tools/nav_serial_test.py --send chassis --vx 1 --vy -0.5 --wz 0.25 --dry-run
```

## 终端总结与数据留存

停止测试后会保留类似以下统计：

```text
SUMMARY elapsed=12.34s tx=0 finish_tx=0 rx_bytes=320 rx_frames=20 finite_angle_frames=20 discarded_bytes=0 bad_tail_candidates=0 pending_bytes=0
RX_MARKERS 78=20 88=20 A5=0 FF=0 0D=0
RX_RESULT: 已匹配 16 字节上行帧；协议无 CRC，请与电控实测角度、血量逐项核对。
```

`tx` 是常规下行写入帧数，`finish_tx` 是退出时补发的结束帧数；`rx_bytes` 是收到的总字节数，`rx_frames` 是匹配到的完整帧数，`finite_angle_frames` 是角度为有限数值的帧数。`discarded_bytes` 和 `bad_tail_candidates` 可帮助定位错位或不匹配数据，`pending_bytes` 表示退出时还未凑齐一帧的字节数。总结末尾还会说明是否有有效上行帧，以及下行命令尚未得到电控确认。

需要保存原始接收流供后续分析时，增加一个新文件路径；采集文件使用独占创建，不覆盖旧文件：

```bash
./tools/run_nav_serial_test.sh --port /dev/ttyACM0 --capture /tmp/nav_rx.bin
python3 tools/nav_serial_test.py --replay /tmp/nav_rx.bin
```

离线解析会显示前 128 个字节、前 10 个匹配帧和汇总统计，不会打开串口。

## 排障

- **没有发现串口**：检查 USB、设备名和用户串口权限；通过 `--list-ports` 核对 `/dev/ttyUSB*`、`/dev/ttyACM*` 或 `/dev/serial/by-id/*`。
- **串口打开失败或 busy**：停止占用它的 ROS 串口驱动和其他终端工具。脚本使用独占模式，不支持和导航节点同时打开同一设备。
- **有 `RX_RAW`，但 `rx_frames=0`**：查看电控是否持续发送 16 字节 `78 ... 88` 上行帧、波特率和 UART 是否匹配。`RX_RAW len=3` 只表示一次系统读取了 3 字节，后续数据可能拼成完整帧。
- **完全收不到字节**：核对 TX/RX/GND 接线、电控 UART、固件发送任务和双方的 115200 波特率。小电脑打印 `OPEN ... baud=115200` 不能证明电控也配置了 115200。
- **角度异常或极小**：对照电控字段定义检查小端序、浮点偏移和帧长。零值也可能是实测值，应以电控变量为准。
- **`TX_WRITTEN` 后电控变量没变化**：让电控检查收到的 `FF ... 0D` 是否完整，以及命令码 `ED/EC`、数据偏移和小端解释。上行反馈不能替代下行 ACK。
- **Foxglove 已打开但没有话题**：确认 `/serial_test/status` 可见并确认 Bridge 与串口进程使用同一 ROS 环境和 `ROS_DOMAIN_ID`。话题只有在脚本带 `--ros` 或 `--foxglove` 时才发布；一键脚本已自动启用。
- **端口 8765 已占用**：关闭已有 Foxglove Bridge，再重新启动。可直接运行 `python3 tools/nav_serial_test.py --help` 查看参数；需要改端口时使用 `--foxglove-port`。

退出码：0 表示至少解析到一帧有限角度反馈；1 表示运行或串口错误；2 表示没有解析到有效上行帧；130 表示按 `Ctrl+C` 停止。`--dry-run`、列设备和成功的离线解析正常返回 0。

## 离线自测

```bash
python3 -B tools/test_nav_serial_test.py -v
```

测试通过伪终端模拟串口，不需要电控或实机串口。覆盖协议参考帧、断帧拼接、噪声恢复、上下行双向传输、结束补发、无有效反馈时的结果码、数据采集与离线回放、Foxglove Bridge 生命周期及启动参数检查。它验证的是工具自身的收发和解析行为；真实联调时请运行上面的一键命令，让电控持续提供实际上行数据。
