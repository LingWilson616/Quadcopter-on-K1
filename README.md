# K1 机载边缘 AI 感知平台

基于进迭时空 SPACEMIT K1 (MUSE Pi Pro) 的 ROS2 机载电脑适配方案——视觉目标检测 + 语音交互 + MAVLink 飞控通信。

全国大学生嵌入式比赛项目。

## 项目概况

- **主控板**: K1 (MUSE Pi Pro, RISC-V 8核) — 机载边缘计算平台
- **感知能力**: YOLOv8n 人物检测（SpaceMIT NPU, 18 FPS）+ 语音指令交互（ASR+LLM+TTS）
- **飞控通信**: MAVLink v2 over UART，适配 ArduPilot/PX4 生态
- **架构**: ROS2 Humble 5 节点，模块化可裁剪
- **应用场景**: 无人机机载电脑、UGV 车载感知、边缘 AI 监控

## 架构

```
┌─────────────────────────────────────────────┐
│          K1 机载边缘 AI 感知平台               │
│                                              │
│  摄像头 ──→ drone_vision ──→ drone_inference  │
│              (V4L2取流)      (YOLOv8+NPU)     │
│                                  │            │
│  麦克风 ──→ drone_voice          │            │
│              (ASR+LLM+TTS)       │            │
│                │                 │            │
│                └──────┬──────────┘            │
│                       ↓ /drone/command        │
│                  drone_communication          │
│                   (MAVLink v2)                │
└──────────────────────┼───────────────────────┘
                       │ UART
                  ┌────┴────┐
                  │ 飞控     │  ← ArduPilot / PX4
                  │ (适配层) │
                  └─────────┘
```

**设计理念**: K1 作为机载 companion computer，通过 ROS2 模块化架构提供感知能力（视觉+语音），经标准化 MAVLink 协议与飞控解耦通信。

## ROS2 包

| 包 | 功能 | 状态 |
|---|---|---|
| drone_interfaces | Detection2D, DroneStatus, InferenceResult | 已就绪 |
| drone_vision | V4L2 USB 摄像头 → ROS Image (MJPG 1280x720@25fps) | **已实测通过** |
| drone_inference | YOLOv8n INT8 + SpaceMIT EP 人物检测 | **已实测通过** |
| drone_communication | UART ↔ MAVLink v2 (pyserial + pymavlink) | **已实测通过** |
| drone_voice | 语音交互 (VAD→ASR→LLM→TTS)，发布指令到 /drone/command | **已实测通过** |
| drone_bringup | Launch 文件 + 参数管理 | 已就绪 |

## 快速开始

### 1. 依赖安装

```bash
# ONNX Runtime + SpaceMIT EP（必须）
sudo apt-get install -y spacemit-onnxruntime python3-spacemit-ort
sudo chmod 666 /dev/tcm                     # EP 必做！否则段错误

# 拉取预量化模型
git clone https://gitee.com/bianbu/spacemit-demo.git ~/spacemit-demo
cd ~/spacemit-demo/examples/CV/yolov8/model && bash download_model.sh

# ROS2 基础依赖
sudo apt-get install -y ros-humble-cv-bridge python3-serial
pip3 install pymavlink
```

### 2. 部署与编译

```bash
# 本地 → K1
scp -r ros2_ws/src/* bianbu@10.171.220.9:~/drone_project/ros2_ws/src/
scp config/* bianbu@10.171.220.9:~/drone_project/config/

# K1 板
ssh bianbu@10.171.220.9
source /opt/bros/humble/setup.bash
cd ~/drone_project
./setup_ros2.sh    # 一键安装依赖 + colcon build
```

### 3. 启动

```bash
source /opt/bros/humble/setup.bash
source ~/drone_project/ros2_ws/install/setup.bash

# 一键启动全部 5 个节点
ros2 launch drone_bringup drone.launch.py

# 覆盖参数
ros2 launch drone_bringup drone.launch.py \
  camera_device:=/dev/video1 \
  uart_device:=/dev/ttyS1 \
  confidence_threshold:=0.5
```

---

## 功能详解

### 功能 1 — 摄像头采集 (`drone_vision`)

USB 摄像头通过 V4L2 后端采集，发布标准 ROS `sensor_msgs/Image`。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `camera_device` | `/dev/video20` | 摄像头设备节点 |
| `camera_format` | `MJPG` | 像素格式 (MJPG / YUYV) |
| `image_width` | 1280 | 分辨率宽 |
| `image_height` | 720 | 分辨率高 |
| `fps` | 25 | 采集帧率 |

**话题**:
- **发布** `/camera/image_raw` (`sensor_msgs/Image`) — 原始图像帧

**单独启动**:
```bash
ros2 run drone_vision camera_node --ros-args \
  -p camera_device:=/dev/video20 \
  -p image_width:=1280 \
  -p image_height:=720
```

**查看实时画面**:
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

**摄像头状态检查**:
```bash
v4l2-ctl -d /dev/video20 --list-formats  # 查看支持的格式
v4l2-ctl -d /dev/video20 --all            # 查看当前配置
```

**故障恢复**: 摄像头断连后节点自动重试重连，`respawn=True` 保证崩溃自动拉起。

---

### 功能 2 — 人物检测 (`drone_inference`)

YOLOv8n ONNX 模型推理，支持 SpaceMIT EP 硬件加速（TCM 正常时 ~18 FPS）或 CPU 回退。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `model_path` | `~/spacemit-demo/.../yolov8n_320x320.q.onnx` | ONNX 模型路径 |
| `confidence_threshold` | 0.3 | 置信度阈值 |
| `iou_threshold` | 0.45 | NMS IoU 阈值 |
| `num_threads` | 4 | CPU 推理线程数 |
| `person_only` | true | 仅检测 person (class_id=0) |

**话题**:
- **订阅** `/camera/image_raw` (`sensor_msgs/Image`)
- **发布** `/drone/inference_result` (`drone_interfaces/InferenceResult`) — 检测框列表 + 推理耗时

**单独启动**:
```bash
ros2 run drone_inference inference_node --ros-args \
  -p model_path:=/home/bianbu/spacemit-demo/examples/CV/yolov8/model/yolov8n_320x320.q.onnx \
  -p confidence_threshold:=0.5 \
  -p person_only:=true
```

**查看检测结果**:
```bash
ros2 topic echo /drone/inference_result
```
输出示例:
```
detections:
  - class_id: 0
    label: person
    confidence: 0.87
    x_min: 120.5, y_min: 80.3
    x_max: 310.2, y_max: 420.1
inference_time_ms: 229.3
model_name: yolov8n_320x320.q.onnx
```

**推理性能**:

| 模型 | EP 推理 | 预处理 | 后处理 | 总耗时 | FPS |
|------|---------|--------|--------|--------|-----|
| YOLOv8n 192×320 INT8 | 10ms | 8ms | 17ms | 36ms | **28** |
| YOLOv8n 320×320 INT8 | 14ms | 9ms | 24ms | 54ms | **18** |

> **注意**: 上表为 SpaceMIT EP 正常时的数据。当前 K1 DEBUG 内核下 TCM 不可用，EP 自动回退 CPU（~229ms/帧）。切回正式内核 `vmlinuz-6.6.63` 可恢复 EP 加速。

**切换模型**:
```bash
# 轻量 192×320（更快但精度略降）
model_path:=/home/bianbu/spacemit-demo/examples/CV/yolov8/model/yolov8n_192x320.q.onnx
# 标准 320×320
model_path:=/home/bianbu/spacemit-demo/examples/CV/yolov8/model/yolov8n_320x320.q.onnx
```

---

### 功能 3 — 飞控通信 (`drone_communication`)

MAVLink v2 协议通过 UART 串口与 ArduPilot/PX4 飞控双向通信。支持虚拟飞控 — 无实体飞控时也能完整测试 MAVLink 通信链路。

**虚拟飞控测试** (无需实体飞控):

```bash
# 终端 1: 启动虚拟飞控（自动创建虚拟串口）
python3 ~/drone_project/tools/virtual_fc.py
# 输出: PTY: /dev/pts/N，键盘可控制: [a]RM [t]AKEOFF [l]AND [r]TL

# 终端 2: 连接 mavlink_node 到虚拟飞控
ros2 run drone_communication mavlink_node --ros-args -p uart_device:=/dev/pts/N
# 输出: Connected + Heartbeat + Attitude 遥测

# 发送指令测试
ros2 topic pub /drone/command std_msgs/msg/String "{data: 'ARM'}"
ros2 topic pub /drone/command std_msgs/msg/String "{data: 'TAKEOFF'}"
```

虚拟飞控功能：
- 双向 MAVLink v2 通信（HEARTBEAT / ATTITUDE / GLOBAL_POSITION_INT / VFR_HUD / SYS_STATUS）
- 响应 ARM / TAKEOFF / LAND / RTL 指令（COMMAND_ACK）
- 键盘交互控制飞控状态
- 模拟飞行数据（姿态变化、高度爬升、电池消耗）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `uart_device` | `/dev/ttyACM0` | 飞控串口设备 |
| `baud_rate` | 57600 | 波特率（须与飞控一致） |
| `target_system` | 1 | 飞控 MAVLink ID |

**话题**:
- **订阅** `/drone/command` (`std_msgs/String`) — 接收飞控指令
- **发布** `/drone/status` (`drone_interfaces/DroneStatus`) — 飞控遥测状态

**支持的指令**:
| 指令 | 说明 |
|------|------|
| `ARM` | 解锁飞控 |
| `TAKEOFF` | 起飞（默认 5m） |
| `LAND` | 降落 |
| `RTL` | 返航 |
| `GUIDED lat lon alt` | 指点飞行 |

**发送指令**:
```bash
# ARM 解锁
ros2 topic pub /drone/command std_msgs/msg/String "{data: 'ARM'}"

# 起飞
ros2 topic pub /drone/command std_msgs/msg/String "{data: 'TAKEOFF'}"

# 降落
ros2 topic pub /drone/command std_msgs/msg/String "{data: 'LAND'}"

# 语音节点也会自动发布指令到同一个 topic
```

**查看飞控状态**:
```bash
ros2 topic echo /drone/status
```
输出: `armed`, `mode`, `roll`, `pitch`, `yaw`, `altitude`, `heading`, `ground_speed`, `battery_voltage`, `battery_current`, `latitude`, `longitude`

**单独启动**:
```bash
ros2 run drone_communication mavlink_node --ros-args \
  -p uart_device:=/dev/ttyACM0 \
  -p baud_rate:=57600
```

**故障恢复**: 串口断开自动重连（1.5s 间隔），不休眠不退出。

---

### 功能 4 — 语音交互 (`drone_voice`)

五段式语音管线：VAD 语音检测 → ASR 语音识别 → LLM 意图理解 → TTS 语音合成 → 指令执行。

**前置条件**:
```bash
# 1. USB 麦克风已连接
pactl list short sources | grep USB_PnP_Sound

# 2. LLM 服务已启动（Qwen2.5-0.5B）
curl -s http://127.0.0.1:8081/v1/chat/completions \
  -d '{"messages":[{"role":"user","content":"hi"}],"max_tokens":5}'

# 3. 语音模型已下载到 ~/.cache/
ls ~/.cache/sensevoice/silero_vad.onnx                # VAD
ls ~/.cache/matcha-icefall-zh-baker/model-steps-3.q.onnx  # TTS
ls ~/.cache/vocos_22k.q.onnx                           # Vocoder
```

**话题**:
- **发布** `/drone/command` (`std_msgs/String`) — LLM 解析出的飞控指令 (ARM/TAKEOFF/LAND/RTL)
- **发布** `/drone/voice_text` (`std_msgs/String`) — ASR 识别文本 + LLM 回复文本（调试用）

**支持的语音指令**: 解锁/起飞/降落/返航 — LLM 自动识别意图并映射到 ARM/TAKEOFF/LAND/RTL

**使用方法**:
```bash
# 对着 USB 麦克风说出指令（如"起飞"）
# 终端日志会显示完整链路:
#   Speech detected          ← VAD 检测到语音
#   ASR: 起飞               ← SenseVoice 识别
#   LLM (0.8s): 收到，正在起飞 [CMD:TAKEOFF]  ← Qwen2.5 理解意图
#   TTS 播报: "收到，正在起飞"               ← MatchTTS 语音合成
#   /drone/command ← TAKEOFF               ← 自动下发飞控指令
```

**也可以说自然语言**（非飞控指令）:
```
"今天天气怎么样" → LLM 自由回答（不加 CMD 标签），TTS 朗读回复
"你是什么"       → "我是K1无人机语音助手"
```

**单独启动**（调试用）:
```bash
source ~/drone_project/ros2_ws/install/setup.bash
ros2 run drone_voice voice_node
```

**语音管线架构**:
```
parec (48kHz, PipeWire source 75)
  → scipy 降采样到 16kHz
  → SileroVAD 端检 (ONNX, CPU)
  → SenseVoice Small ASR (spacemit_asr, RTF=0.38)
  → llama-server (Qwen2.5-0.5B-Q4_0, 11 tok/s)
  → 提取 [CMD:XXX] → 发布 /drone/command
  → MatchTTS (pypinyin → Matcha + Vocos → Griffin-Lim, RTF ~0.7)
  → aplay -D plughw:0,0 播放
```

**音频设备拓扑**:
- **USB 声卡** (C-Media, card 0): 直连 K1 USB → 耳机/音箱输出，`aplay -D plughw:0,0`
- **USB 麦克风** (TI PCM2902, card 2): 在扩展坞上 → 录音输入，PipeWire source 75
- **板载 ES8326** (card 1): 不用（无物理麦克风，ADC 全零）

**TTS 模型**:
- MatchTTS (`matcha-icefall-zh-baker`) + Vocos vocoder，中文自然语音
- 模型路径: `~/.cache/matcha-icefall-zh-baker/` + `~/.cache/vocos_22k.q.onnx`
- 下载: `archive.spacemit.com/spacemit-ai/model_zoo/tts/matcha-tts/matcha-icefall-zh-baker.tar.gz`

---

### 功能 5 — 系统调试

**查看所有话题**:
```bash
ros2 topic list
# /camera/image_raw          — 摄像头原始图像
# /drone/inference_result    — 检测结果
# /drone/status              — 飞控遥测
# /drone/command             — 飞控指令
# /drone/voice_text          — 语音识别/LLM 回复文本
# /parameter_events          — 参数变更事件
# /rosout                    — 日志聚合
```

**查看节点图**:
```bash
ros2 node list          # 列出所有节点
ros2 node info /inference_node  # 查看单个节点的 pub/sub
rqt_graph               # 图形化拓扑
```

**查看日志**:
```bash
# 实时日志（过滤某个节点）
ros2 topic echo /rosout | grep inference_node

# 日志文件
ls ~/.ros/log/
```

**单独启动某个节点**（不用 launch）:
```bash
ros2 run drone_vision camera_node
ros2 run drone_inference inference_node
ros2 run drone_communication mavlink_node
ros2 run drone_voice voice_node
```

**设备就绪检查**:
```bash
# 摄像头
ls /dev/video20 && v4l2-ctl -d /dev/video20 --list-formats
# TCM（AI 加速器）
ls -la /dev/tcm  # 期望 crw-rw-rw-
spacemit-tcm-smi # 查看 TCM block 状态
# 飞控串口
ls /dev/ttyACM0
# 麦克风
pactl list short sources | grep USB_PnP
# LLM 服务
curl -s --max-time 3 http://127.0.0.1:8081/v1/chat/completions \
  -d '{"messages":[{"role":"user","content":"hi"}],"max_tokens":5}'
```

---

## ⚠️ 关键坑

1. **导入顺序**: `import onnxruntime` 必须在 `import spacemit_ort` **之前**，否则段错误
2. **TCM 权限**: `/dev/tcm` 默认 root-only，需 `sudo chmod 666 /dev/tcm`
   - 永久修复: 已写 `/etc/udev/rules.d/99-tcm.rules`: `KERNEL=="tcm", MODE="0666"`
3. **单 Session**: K1 只有 4 个 TCM block (共 512KB)，同一时刻只能一个 EP Session
4. **TCM 内核兼容性**: 当前 K1 运行 `6.6.63+` DEBUG 内核，TCM block 状态 N/A 不可用，EP 自动回退 CPU。切回正式内核 `vmlinuz-6.6.63` 可恢复 EP 加速。详见 `/boot/env_k1-x.txt`

## TODO

- TTS 合成优化（Griffin-Lim 降到 15 轮或修复 Vocos x/y ISTFT，目标 <2s）
- LLM 响应稳定性（0.5B 中英文混杂，考虑 FC 模型做意图分派）
- 后处理加速（DFL+NMS 占 47% 耗时，C++/Numba 可提升至 30+ FPS）
- 检测框坐标映射回原图（当前是模型输入空间坐标）
- MAVLink 节点断线重连上限 + 故障恢复策略

## 摄像头

- **设备**: Microdia Integrated Camera (USB UVC, `/dev/video20`)
- **格式**: MJPG 1280x720@25fps | YUYV 1280x720@10fps
- **后端**: V4L2（OpenCV + RVV 加速）

## 关键资源

- [本地知识库](docs/) — K1 AI 计算栈 / Demo 仓库 / 文档导航 / 技术速查
- 进迭时空文档中心: https://www.spacemit.com/community/document
- SpaceMIT EP: https://github.com/spacemit-com/onnxruntime
- AI Demo 仓库: https://gitee.com/bianbu/spacemit-demo
- 社区论坛: https://forum.spacemit.com
- ROS2 镜像: https://archive.spacemit.com/ros2/
