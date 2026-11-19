#!/usr/bin/env bash
set -euo pipefail

TOOLS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP_BASH="${ROS_SETUP_BASH:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"

if [[ ! -f "$ROS_SETUP_BASH" ]]; then
    printf '找不到 ROS 环境文件: %s\n' "$ROS_SETUP_BASH" >&2
    printf '可通过 ROS_SETUP_BASH=/path/to/setup.bash 指定安装位置。\n' >&2
    exit 1
fi

set +u
source "$ROS_SETUP_BASH"
set -u
exec python3 "$TOOLS_DIR/nav_serial_test.py" --foxglove --duration 0 "$@"
