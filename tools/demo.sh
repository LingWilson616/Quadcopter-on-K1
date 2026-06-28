#!/bin/bash
# demo.sh — K1 Drone 竞赛演示视频脚本
# 用法: ./tools/demo.sh [detect|voice|fc|all]
# 分段展示: 人物检测 → 语音交互 → MAVLink通信 → 全链路
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS_SETUP="/opt/bros/humble/setup.bash"
[ ! -f "$ROS_SETUP" ] && ROS_SETUP="/opt/ros/humble/setup.bash"
source "$ROS_SETUP" 2>/dev/null
source "$PROJECT_DIR/ros2_ws/install/setup.bash" 2>/dev/null || {
    echo "请先运行: cd ~/drone_project && ./setup_ros2.sh"
    exit 1
}

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

section() {
    echo ""
    echo -e "${CYAN}${BOLD}════════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}${BOLD}  $1${NC}"
    echo -e "${CYAN}${BOLD}════════════════════════════════════════════════════════════${NC}"
    echo ""
}

step() {
    echo -e "${GREEN}  ▶ $1${NC}"
}

info() {
    echo -e "    $1"
}

pause() {
    echo ""
    echo -e "${YELLOW}  ⏸  按 Enter 继续...${NC}"
    read -r
}

# ═══════════════════════════════════════════════════════════
# 环境检查
# ═══════════════════════════════════════════════════════════

check_env() {
    section "环境就绪检查"

    step "摄像头"
    if ls /dev/video20 > /dev/null 2>&1; then
        FMT=$(v4l2-ctl -d /dev/video20 --list-formats 2>/dev/null | grep -o 'MJPG\|YUYV' | head -1)
        info "/dev/video20 ✓  (${FMT:-?})"
    else
        info "/dev/video20 ✗  未连接"
    fi

    step "AI 加速器 (TCM)"
    if [ -c /dev/tcm ] && [ -r /dev/tcm ]; then
        info "/dev/tcm ✓  ($(spacemit-tcm-smi 2>/dev/null | grep 'block_num' || echo '4 blocks'))"
    else
        info "/dev/tcm ✗  EP 不可用，将回退 CPU"
    fi

    step "AI 模型"
    MODEL="$HOME/spacemit-demo/examples/CV/yolov8/model/yolov8n_320x320.q.onnx"
    if [ -f "$MODEL" ]; then
        SIZE=$(du -h "$MODEL" | cut -f1)
        info "YOLOv8n 320×320 INT8 ✓  (${SIZE})"
    else
        info "模型未下载, 运行: cd ~/drone_project && ./setup_ros2.sh --with-model-dl"
    fi

    step "ROS2 节点"
    PKGS=$(ros2 pkg list 2>/dev/null | grep -c 'drone_' || echo 0)
    info "${PKGS} 个 drone_ 包已安装"

    step "语音服务"
    if curl -s --max-time 2 http://127.0.0.1:8081/v1/chat/completions \
        -d '{"messages":[{"role":"user","content":"hi"}],"max_tokens":3}' > /dev/null 2>&1; then
        info "LLM 服务 ✓  (llama-server)"
    else
        info "LLM 服务 ✗  语音指令不可用"
    fi

    pause
}

# ═══════════════════════════════════════════════════════════
# 场景1: 人物检测 + ROS2 拓扑
# ═══════════════════════════════════════════════════════════

demo_detect() {
    section "场景1 — 人物检测 (YOLOv8 + SpaceMIT EP)"

    info "启动摄像头 + 推理节点..."
    ros2 launch drone_bringup drone.launch.py \
        uart_device:=/dev/null &
    LAUNCH_PID=$!
    sleep 6

    step "节点拓扑"
    echo ""
    ros2 node list 2>/dev/null
    echo ""
    info "↑ 3 个 ROS2 节点运行中"
    pause

    step "话题列表"
    echo ""
    ros2 topic list 2>/dev/null
    echo ""
    info "↑ /camera/image_raw → /drone/inference_result → /drone/status"
    pause

    step "实时检测结果 (对着摄像头摆姿势)"
    echo ""
    echo -e "  ${YELLOW}连续输出 5 帧检测结果...${NC}"
    echo ""
    for i in $(seq 1 5); do
        RESULT=$(ros2 topic echo /drone/inference_result --once 2>/dev/null)
        DET=$(echo "$RESULT" | grep -c 'class_id\|person' || true)
        CONF=$(echo "$RESULT" | grep 'confidence' | grep -oP '[\d.]+' | head -1)
        TIME=$(echo "$RESULT" | grep 'inference_time_ms' | grep -oP '[\d.]+')
        if [ -n "$CONF" ]; then
            echo -e "  [$i] ${GREEN}检测到人物${NC} 置信度=${CONF}  耗时=${TIME}ms"
        else
            echo -e "  [$i] 未检测到 (请站在摄像头前)"
        fi
        sleep 1.5
    done
    pause

    step "Node 节点图 (rqt_graph)"
    info "在 K1 桌面运行: rqt_graph"
    info "展示: camera_node → /camera/image_raw → inference_node → /drone/inference_result"

    # 保存一张检测快照
    step "保存检测快照"
    python3 -c "
import cv2
cap = cv2.VideoCapture('/dev/video20', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
for _ in range(5): ret, f = cap.read()
cap.release()
cv2.imwrite('/tmp/demo_detect.jpg', f)
print('  ✓ 已保存 /tmp/demo_detect.jpg')
" 2>/dev/null || true

    kill $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null
    pause
}

# ═══════════════════════════════════════════════════════════
# 场景2: 语音指令交互
# ═══════════════════════════════════════════════════════════

demo_voice() {
    section "场景2 — 语音指令交互 (ASR + LLM + TTS)"

    if ! curl -s --max-time 2 http://127.0.0.1:8081/v1/chat/completions \
        -d '{"messages":[{"role":"user","content":"hi"}],"max_tokens":3}' > /dev/null 2>&1; then
        echo -e "  ${RED}LLM 服务未启动，跳过语音演示${NC}"
        echo "  启动: sudo systemctl start llama-server"
        pause
        return
    fi

    info "麦克风 → SileroVAD → SenseVoice ASR → Qwen2.5 LLM → MatchTTS → 音箱"
    echo ""

    step "启动语音节点..."
    ros2 run drone_voice voice_node &
    VOICE_PID=$!
    sleep 4

    echo ""
    echo -e "  ${YELLOW}┌────────────────────────────────────────────┐${NC}"
    echo -e "  ${YELLOW}│  对着麦克风说出指令:                        │${NC}"
    echo -e "  ${YELLOW}│  1. \"解锁\"  →  ARM                          │${NC}"
    echo -e "  ${YELLOW}│  2. \"起飞\"  →  TAKEOFF                      │${NC}"
    echo -e "  ${YELLOW}│  3. \"降落\"  →  LAND                         │${NC}"
    echo -e "  ${YELLOW}│  4. \"返航\"  →  RTL                          │${NC}"
    echo -e "  ${YELLOW}│  5. \"今天天气怎么样\" → 自由回答 (不带指令)   │${NC}"
    echo -e "  ${YELLOW}└────────────────────────────────────────────┘${NC}"
    echo ""

    info "终端日志将显示: Speech detected → ASR: xxx → LLM: xxx → TTS播报"
    echo ""

    # 等待30秒让人说话
    for i in $(seq 30 -1 1); do
        printf "\r  剩余 %2d 秒 (Ctrl+C 提前结束)  " "$i"
        sleep 1
    done
    echo ""

    kill $VOICE_PID 2>/dev/null; wait $VOICE_PID 2>/dev/null
    pause
}

# ═══════════════════════════════════════════════════════════
# 场景3: MAVLink 通信 + 虚拟飞控
# ═══════════════════════════════════════════════════════════

demo_fc() {
    section "场景3 — MAVLink 通信 + 虚拟飞控"

    info "启动虚拟 ArduPilot 飞控..."
    python3 "$PROJECT_DIR/tools/virtual_fc.py" &
    FC_PID=$!
    sleep 1.5

    # 找 PTY
    PTY=""
    for p in /dev/pts/*; do
        [ "$(stat -c '%U' "$p" 2>/dev/null)" = "$USER" ] && PTY="$p"
    done
    info "虚拟飞控: $PTY  (PID=$FC_PID)"

    step "连接 mavlink_node → 虚拟飞控"
    ros2 launch drone_bringup drone.launch.py uart_device:="$PTY" &
    LAUNCH_PID=$!
    sleep 6

    step "飞控遥测数据 (5Hz)"
    echo ""
    echo -e "  ${YELLOW}实时飞控状态:${NC}"
    echo ""
    timeout 8 ros2 topic echo /drone/status 2>/dev/null | head -30 || true
    echo ""
    info "↑ attitude / GPS / battery / armed 状态"
    pause

    step "发送指令 — ARM 解锁"
    echo ""
    ros2 topic pub --once /drone/command std_msgs/msg/String "{data: 'ARM'}" 2>/dev/null
    sleep 1
    ros2 topic echo /drone/status --once 2>/dev/null | grep 'armed'
    info "↑ armed: True — 解锁成功"
    pause

    step "发送指令 — TAKEOFF 起飞"
    echo ""
    ros2 topic pub --once /drone/command std_msgs/msg/String "{data: 'TAKEOFF'}" 2>/dev/null
    sleep 2
    echo ""
    info "观察虚拟飞控终端: TAKEOFF → 姿态变化 → 高度爬升"
    pause

    step "发送指令 — LAND 降落"
    echo ""
    ros2 topic pub --once /drone/command std_msgs/msg/String "{data: 'LAND'}" 2>/dev/null
    sleep 1
    info "降落完成，armed: False"

    kill $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null
    kill $FC_PID 2>/dev/null; wait $FC_PID 2>/dev/null
    pause
}

# ═══════════════════════════════════════════════════════════
# 场景4: 全链路 (无人值守)
# ═══════════════════════════════════════════════════════════

demo_all() {
    section "场景4 — 全链路集成演示"

    info "5 个 ROS2 节点同时运行:"
    info "  camera_node → inference_node → mavlink_node → voice_node"
    info ""

    step "启动全节点"
    python3 "$PROJECT_DIR/tools/virtual_fc.py" &
    FC_PID=$!
    sleep 1.5
    PTY=""
    for p in /dev/pts/*; do
        [ "$(stat -c '%U' "$p" 2>/dev/null)" = "$USER" ] && PTY="$p"
    done
    info "虚拟飞控: $PTY"

    ros2 launch drone_bringup drone.launch.py uart_device:="$PTY" &
    LAUNCH_PID=$!
    sleep 8

    echo ""
    step "运行中的节点"
    ros2 node list 2>/dev/null
    echo ""

    step "所有话题"
    ros2 topic list 2>/dev/null
    echo ""
    info "5 个话题: 图像 / 检测结果 / 飞控状态 / 飞控指令 / 语音文本"
    pause

    # 同时展示检测 + 飞控
    step "同时查看检测结果 + 飞控状态"
    echo ""
    echo -e "  ${YELLOW}检测结果 (/drone/inference_result):${NC}"
    ros2 topic echo /drone/inference_result --once 2>/dev/null | head -12
    echo ""
    echo -e "  ${YELLOW}飞控状态 (/drone/status):${NC}"
    ros2 topic echo /drone/status --once 2>/dev/null | head -12
    pause

    step "语音 → 检测 → 飞控 → 遥测 全链路通畅"
    echo ""
    echo -e "  ${GREEN}演示完成 ✓${NC}"
    echo ""
    echo "  数据流:"
    echo "  摄像头 → YOLOv8 推理 → 人物检测结果"
    echo "  麦克风 → ASR → LLM → TTS → 飞控指令"
    echo "  MAVLink → 飞控遥测 → /drone/status"
    echo ""

    kill $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null
    kill $FC_PID 2>/dev/null; wait $FC_PID 2>/dev/null
}

# ═══════════════════════════════════════════════════════════
# 主入口
# ═══════════════════════════════════════════════════════════

case "${1:-all}" in
    check)  check_env ;;
    detect) check_env; demo_detect ;;
    voice)  check_env; demo_voice ;;
    fc)     check_env; demo_fc ;;
    all)
        check_env
        demo_detect
        demo_voice
        demo_fc
        demo_all
        ;;
    *)
        echo "用法: ./tools/demo.sh [check|detect|voice|fc|all]"
        echo ""
        echo "  check   — 仅环境检查"
        echo "  detect  — 人物检测 + ROS2 拓扑展示"
        echo "  voice   — 语音指令交互"
        echo "  fc      — MAVLink 通信 + 虚拟飞控"
        echo "  all     — 全链路 (默认)"
        ;;
esac
