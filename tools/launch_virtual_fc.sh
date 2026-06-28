#!/bin/bash
# launch_virtual_fc.sh — 一键: 虚拟飞控 + ROS2 全节点联调
# 用法: ./tools/launch_virtual_fc.sh
#
# 自动: 1) virtual_fc.py 后台启动 → 写 /tmp/virtual_fc_pty.txt
#       2) 读取 PTY 路径 → ros2 launch 自动传入 uart_device:=/dev/pts/N
#       3) Ctrl+C 退出 → 自动清理 virtual_fc
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PTY_FILE="/tmp/virtual_fc_pty.txt"

ROS_SETUP="/opt/bros/humble/setup.bash"
[ ! -f "$ROS_SETUP" ] && ROS_SETUP="/opt/ros/humble/setup.bash"
source "$ROS_SETUP"
source "$PROJECT_DIR/ros2_ws/install/setup.bash" 2>/dev/null || {
    echo "请先构建: cd ~/drone_project && ./setup_ros2.sh"
    exit 1
}

# ── 清理旧 PTY 文件和残留进程 ─────────────────
rm -f "$PTY_FILE"
pkill -f 'virtual_fc.py' 2>/dev/null || true
sleep 0.5

# ── 启动虚拟飞控 ─────────────────────────────
echo "==> 启动虚拟飞控..."
python3 "$PROJECT_DIR/tools/virtual_fc.py" &
FC_PID=$!

# 等待 PTY 文件出现 (最多 5 秒)
for i in $(seq 1 10); do
    sleep 0.5
    if [ -f "$PTY_FILE" ]; then
        PTY=$(cat "$PTY_FILE")
        if [ -n "$PTY" ] && [ -e "$PTY" ]; then
            break
        fi
    fi
done

if [ -z "$PTY" ] || [ ! -e "$PTY" ]; then
    echo "ERROR: 虚拟飞控未就绪"
    kill $FC_PID 2>/dev/null
    exit 1
fi

echo "  虚拟飞控: $PTY  (PID=$FC_PID)"

# ── 清理函数 ─────────────────────────────────
cleanup() {
    echo ""
    echo "==> 停止虚拟飞控..."
    kill $FC_PID 2>/dev/null
    wait $FC_PID 2>/dev/null
    rm -f "$PTY_FILE"
    echo "  已停止"
}
trap cleanup EXIT INT TERM

# ── 启动 ROS2 ────────────────────────────────
echo "==> 启动 ROS2 (uart_device:=$PTY)"
echo ""

ros2 launch drone_bringup drone.launch.py uart_device:="$PTY"
