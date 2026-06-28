#!/bin/bash
# launch_virtual_fc.sh — 启动虚拟飞控 + ROS2 全节点联调
# 用法: ./tools/launch_virtual_fc.sh [--no-voice]
#
# 自动: 1) 后台启动 virtual_fc.py → 获取 PTY 路径
#       2) 启动 ros2 launch drone_bringup，自动传入 uart_device:=PTY
#       3) Ctrl+C 退出时自动清理 virtual_fc 进程
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

ROS_SETUP="/opt/bros/humble/setup.bash"
[ ! -f "$ROS_SETUP" ] && ROS_SETUP="/opt/ros/humble/setup.bash"

source "$ROS_SETUP"
source "$PROJECT_DIR/ros2_ws/install/setup.bash"

# ── 启动虚拟飞控 ────────────────────────────────
echo "==> 启动虚拟飞控..."
python3 "$PROJECT_DIR/tools/virtual_fc.py" &
FC_PID=$!
sleep 1.5

# 从虚拟飞控输出中提取 PTY 路径
FC_LOG="/tmp/virtual_fc_pty.txt"
# virtual_fc 输出包含 "PTY: /dev/pts/N"，尝试从 /dev/pts/ 中找最新的
PTY=""
for p in /dev/pts/*; do
    if [ "$(stat -c '%U' "$p" 2>/dev/null)" = "$USER" ]; then
        PTY="$p"
    fi
done

if [ -z "$PTY" ]; then
    echo "ERROR: 未能找到虚拟飞控 PTY"
    kill $FC_PID 2>/dev/null
    exit 1
fi

echo "  虚拟飞控 PTY: $PTY (PID=$FC_PID)"

# ── 清理函数 ─────────────────────────────────────
cleanup() {
    echo ""
    echo "==> 停止虚拟飞控..."
    kill $FC_PID 2>/dev/null
    wait $FC_PID 2>/dev/null
    echo "  已停止"
}
trap cleanup EXIT INT TERM

# ── 启动 ROS2 ────────────────────────────────────
echo "==> 启动 ROS2 (连接虚拟飞控 $PTY)..."
echo ""

ros2 launch drone_bringup drone.launch.py uart_device:="$PTY"
