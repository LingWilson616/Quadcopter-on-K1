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

**设计理念**: K1 作为机载 companion computer，通过 ROS2 模块化架构提供感知能力（视觉+语音），经标准化 MAVLink 协议与飞控解耦通信。平台不绑定特定机型——更换飞控或载机只需调整 MAVLink 参数。

## ROS2 包

| 包 | 功能 | 状态 |
|---|---|---|
| drone_interfaces | Detection2D, DroneStatus, InferenceResult | 已就绪 |
| drone_vision | V4L2 USB 摄像头 → ROS Image (MJPG 1280x720@25fps) | **已实测通过** |
| drone_inference | YOLOv8n INT8 + SpaceMIT EP 人物检测 | **已实测通过** |
| drone_communication | UART ↔ MAVLink v2 (pyserial + pymavlink) | **已实测通过** |
| drone_voice | 语音交互 (VAD→ASR→LLM→TTS)，发布指令到 /drone/command | **已实测通过** |
| drone_bringup | Launch 文件 + 参数管理 | 已就绪 |

## 推理性能

| 模型 | EP 推理 | 预处理 | 后处理 | 总耗时 | FPS |
|------|---------|--------|--------|--------|-----|
| YOLOv8n 192×320 INT8 | 10ms | 8ms | 17ms | 36ms | **28** |
| YOLOv8n 320×320 INT8 | 14ms | 9ms | 24ms | 54ms | **18** |

模型文件: `~/spacemit-demo/examples/CV/yolov8/model/yolov8n_*.q.onnx`（预量化，开箱即用）

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
colcon build --symlink-install
```

### 3. 启动

```bash
source /opt/bros/humble/setup.bash
source ~/drone_project/ros2_ws/install/setup.bash

# 全节点
ros2 launch drone_bringup drone.launch.py

# 覆盖参数
ros2 launch drone_bringup drone.launch.py \
  model_path:=/path/to/model.onnx \
  confidence_threshold:=0.5 \
  uart_device:=/dev/ttyACM0
```

### 4. 调试命令

```bash
# 推理结果
ros2 topic echo /drone/inference_result

# 飞控状态
ros2 topic echo /drone/status

# 发送指令
ros2 topic pub /drone/command std_msgs/msg/String "{data: 'ARM'}"

# 节点图
rqt_graph
```

### 5. 语音交互（需外接麦克风）

```bash
# K1 板载 ES8326 需要外接 I2S 麦克风，或使用 USB 麦克风
# 确保 llama-server 已启动
sudo systemctl start llama-server

# 启动语音节点
cd ~/drone_project
source install/setup.bash
~/drone_project/install/drone_voice/bin/voice_node

# 对着麦克风说出指令（如"起飞""降落"），LLM 自动生成飞控命令
```

**语音管线**: PyAudio 48kHz 录音 → scipy 降采样 → SileroVAD 检测 → SenseVoice Small ASR → llama-server (Qwen2.5-0.5B) → 提取 [CMD:XXX] → 发布 /drone/command → espeak-ng TTS 回复

**支持的语音指令**: 解锁/起飞/降落/返航（LLM 自动识别意图并映射到 ARM/TAKEOFF/LAND/RTL）

**硬件要求**: ES8326 I2S 麦克风模块 或 USB 麦克风。USB 声卡（C-Media）仅有播放功能，不包含麦克风。

## 推理节点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `model_path` | `~/spacemit-demo/.../yolov8n_320x320.q.onnx` | ONNX 模型路径 |
| `confidence_threshold` | 0.3 | 置信度阈值 |
| `iou_threshold` | 0.45 | NMS IoU 阈值 |
| `num_threads` | 4 | EP 推理线程数 |
| `person_only` | true | 仅检测 person (class_id=0) |

## ⚠️ 关键坑

1. **导入顺序**: `import onnxruntime` 必须在 `import spacemit_ort` **之前**，否则段错误
2. **TCM 权限**: `/dev/tcm` 默认 root-only，需 `sudo chmod 666 /dev/tcm`
   - 永久修复: 已写 `/etc/udev/rules.d/99-tcm.rules`: `KERNEL=="tcm", MODE="0666"`
3. **单 Session**: K1 只有 4 个 TCM block (共 512KB)，同一时刻只能一个 EP Session
4. **EP 不带 provider_options**: 和官方 demo 一致，不传 `provider_options` 参数

## TODO

- 外接麦克风完成端到端语音控制闭环测试
- 后处理加速（DFL+NMS 占 47% 耗时，C++/Numba 可提升至 30+ FPS）
- 检测框坐标映射回原图（当前是模型输入空间坐标）
- MAVLink 节点断线重连上限 + 故障恢复策略
- 实机挂载测试（K1 + 飞控 + 载机联调）

## 摄像头

- **设备**: Microdia Integrated Camera (USB UVC, `/dev/video20`)
- **格式**: MJPG 1280x720@25fps | YUYV 1280x720@10fps
- **后端**: V4L2（OpenCV + RVV 加速）
- **截图**: `python3 ~/snap.py` → `/home/bianbu/camera_latest.jpg`

## 关键资源

- [本地知识库](docs/) — K1 AI 计算栈 / Demo 仓库 / 文档导航 / 技术速查
- 进迭时空文档中心: https://www.spacemit.com/community/document
- SpaceMIT EP: https://github.com/spacemit-com/onnxruntime
- AI Demo 仓库: https://gitee.com/bianbu/spacemit-demo
- 社区论坛: https://forum.spacemit.com
- ROS2 镜像: https://archive.spacemit.com/ros2/
