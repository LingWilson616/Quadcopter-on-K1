#!/bin/bash
# setup_ros2.sh — K1 Drone 项目一键环境部署
# 用法: cd ~/drone_project && ./setup_ros2.sh [--with-voice] [--with-model-dl]
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"

# ── 参数解析 ───────────────────────────────────
WITH_VOICE=false
WITH_MODEL_DL=false
FAST_MODE=false

for arg in "$@"; do
    case "$arg" in
        --with-voice) WITH_VOICE=true ;;
        --with-model-dl) WITH_MODEL_DL=true ;;
        --fast) FAST_MODE=true ;;
    esac
done

# ── 前置检查 ───────────────────────────────────
BROKER=$(printenv ROS_DISTRO 2>/dev/null || echo '')
if [ "$BROKER" != "humble" ]; then
    if [ -f /opt/bros/humble/setup.bash ]; then
        source /opt/bros/humble/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
fi

echo "╔══════════════════════════════════════════════╗"
echo "║   K1 Drone — 一键环境部署                     ║"
echo "║   视觉检测 + 语音交互 + MAVLink 通信            ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

# ── 1. 系统依赖 ─────────────────────────────────
echo "==> [1/5] 安装系统依赖..."

PACKAGES="python3-pip python3-serial python3-colcon-common-extensions python3-numpy"
PACKAGES="$PACKAGES ros-humble-cv-bridge ros-humble-image-transport"
PACKAGES="$PACKAGES spacemit-onnxruntime python3-spacemit-ort"

sudo apt-get update -qq 2>/dev/null
sudo apt-get install -y $PACKAGES 2>&1 | tail -3
echo "  ✓ 系统包安装完成"

# TCM 权限 (AI 加速器, 必须)
sudo chmod 666 /dev/tcm 2>/dev/null || echo "  [WARN] /dev/tcm 不存在 (非 K1 板?)"

# 永久 TCM 权限规则 (新建或检查是否已存在)
TCM_RULE='/etc/udev/rules.d/99-tcm.rules'
if [ ! -f "$TCM_RULE" ]; then
    echo 'KERNEL=="tcm", MODE="0666"' | sudo tee "$TCM_RULE" > /dev/null
    echo "  ✓ TCM udev 规则已写入"
fi

# ONNX Runtime + Python 依赖
pip3 install --user --break-system-packages -q pyserial pymavlink numpy 2>&1 | tail -1
echo "  ✓ Python 依赖安装完成"

# ── 2. AI 模型和资源 ────────────────────────────
echo ""
echo "==> [2/5] AI 模型..."

# YOLO 模型
DEMO_DIR="$HOME/spacemit-demo"
YOLO_MODEL="$DEMO_DIR/examples/CV/yolov8/model/yolov8n_320x320.q.onnx"

if [ -f "$YOLO_MODEL" ]; then
    echo "  ✓ YOLOv8 模型已存在"
elif [ "$WITH_MODEL_DL" = true ]; then
    echo "  下载 spacemit-demo..."
    if [ ! -d "$DEMO_DIR" ]; then
        git clone https://gitee.com/bianbu/spacemit-demo.git "$DEMO_DIR" 2>&1 | tail -1
    fi
    cd "$DEMO_DIR/examples/CV/yolov8/model"
    bash download_model.sh 2>&1 | tail -5
    cd "$PROJECT_DIR"
    echo "  ✓ YOLOv8 模型下载完成"
else
    echo "  跳过 (用 --with-model-dl 自动下载, 或手动:"
    echo "     git clone https://gitee.com/bianbu/spacemit-demo.git ~/spacemit-demo"
    echo "     cd ~/spacemit-demo/examples/CV/yolov8/model && bash download_model.sh)"
fi

# LLM 服务检查
echo ""
echo "==> [3/5] 语音服务检查..."

if [ "$WITH_VOICE" = true ]; then
    # LLM server
    if curl -s --max-time 2 http://127.0.0.1:8081/v1/chat/completions \
        -d '{"messages":[{"role":"user","content":"hi"}],"max_tokens":3}' > /dev/null 2>&1; then
        echo "  ✓ LLM 服务 (llama-server) 已运行"
    else
        echo "  [WARN] LLM 服务未启动, 语音指令不可用"
        echo "    启动: sudo systemctl start llama-server"
    fi

    # ASR 模型
    ASR_DIR="$HOME/spacemit-demo/examples/NLP"
    if [ -d "$ASR_DIR" ]; then
        echo "  ✓ ASR 模块 (spacemit_asr) 已就绪"
    fi

    # VAD 模型
    if [ -f "$HOME/.cache/sensevoice/silero_vad.onnx" ]; then
        echo "  ✓ VAD 模型 (silero_vad) 已就绪"
    fi

    # TTS 模型
    if [ -f "$HOME/.cache/matcha-icefall-zh-baker/model-steps-3.q.onnx" ]; then
        echo "  ✓ TTS 模型 (MatchTTS) 已就绪"
    fi
    if [ -f "$HOME/.cache/vocos_22k.q.onnx" ]; then
        echo "  ✓ Vocoder 模型 (vocos) 已就绪"
    fi

    # 音频设备
    if pactl list short sources 2>/dev/null | grep -q 'USB_PnP'; then
        echo "  ✓ USB 麦克风已连接"
    else
        echo "  [WARN] 未检测到 USB 麦克风"
    fi
else
    echo "  跳过 (用 --with-voice 检查语音环境)"
fi

# ── 4. 构建 ROS2 ────────────────────────────────
echo ""
echo "==> [4/5] 构建 ROS2 工作空间..."

WS_DIR="$PROJECT_DIR/ros2_ws"
cd "$WS_DIR"
colcon build --symlink-install 2>&1 | tail -3
echo "  ✓ colcon build 完成"

# ── 5. 启动说明 ─────────────────────────────────
echo ""
echo "==> [5/5] 启动方式"
echo ""
echo "  ┌─ 全节点一键启动 ──────────────────────────────────┐"
echo "  │ source ~/drone_project/ros2_ws/install/setup.bash  │"
echo "  │ ros2 launch drone_bringup drone.launch.py           │"
echo "  └────────────────────────────────────────────────────┘"
echo ""
echo "  ┌─ 虚拟飞控测试 (无需实体飞控) ──────────────────────┐"
echo "  │ 终端1: python3 ~/drone_project/tools/virtual_fc.py  │"
echo "  │ 终端2: ros2 launch drone_bringup drone.launch.py    │"
echo "  │        uart_device:=/dev/pts/N                      │"
echo "  │ 终端3: ros2 topic pub /drone/command std_msgs/...   │"
echo "  │        \"{data: 'ARM'}\"                              │"
echo "  └────────────────────────────────────────────────────┘"
echo ""
echo "  ┌─ 单独启动某节点 ───────────────────────────────────┐"
echo "  │ ros2 run drone_vision camera_node                   │"
echo "  │ ros2 run drone_inference inference_node             │"
echo "  │ ros2 run drone_communication mavlink_node           │"
echo "  │ ros2 run drone_voice voice_node                     │"
echo "  └────────────────────────────────────────────────────┘"
echo ""
echo "  ┌─ 调试命令 ─────────────────────────────────────────┐"
echo "  │ ros2 topic list                                     │"
echo "  │ ros2 topic echo /drone/inference_result             │"
echo "  │ ros2 topic echo /drone/status                       │"
echo "  │ rqt_graph                                           │"
echo "  │ ros2 run rqt_image_view rqt_image_view /camera/ima..│"
echo "  │ v4l2-ctl -d /dev/video20 --all                      │"
echo "  │ spacemit-tcm-smi                                    │"
echo "  └────────────────────────────────────────────────────┘"
echo ""

# ── 环境变量提示 ────────────────────────────────
if ! grep -q "drone_project/ros2_ws/install/setup.bash" ~/.bashrc 2>/dev/null; then
    echo "  [提示] 添加到 ~/.bashrc 免去每次 source:"
    echo "    echo 'source ~/drone_project/ros2_ws/install/setup.bash' >> ~/.bashrc"
fi

echo "  部署完成 ✓"
