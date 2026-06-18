# Quadcopter-on-K1

基于进迭时空 SPACEMIT K1 (MUSE Pi Pro) 的 ROS2 无人机视觉系统。

全国大学生嵌入式比赛项目。

## 项目概况

- **主控板**: K1 (MUSE Pi Pro) — YOLOv8 人物检测 + UART 通信（ROS2 Humble）
- **队友**: 黎同学（ArduPilot 飞控, GPIO UART）
- **飞控**: ArduPilot on MicoAir743-AIO，通过 UART 连接 K1
- **推理**: YOLOv8n 320×320 INT8 + SpaceMIT Execution Provider（~18 FPS ROS2 管道）

## 数据流

```
摄像头(USB UVC MJPG 1280x720@25fps /dev/video20)
    ↓
drone_vision (camera_node)
    ↓ /camera/image_raw
drone_inference (inference_node)  ← YOLOv8n INT8 + SpaceMIT EP
    ↓ /drone/inference_result
drone_communication (mavlink_node)
    ↓ UART (MAVLink v2)
ArduPilot 飞控（黎同学）
    │
    ↑ 遥测数据 (ATTITUDE, GLOBAL_POSITION_INT, SYS_STATUS ...)
drone_communication (mavlink_node)
    ↓ /drone/status (5Hz)
控制台 / 其他 ROS2 节点
```

## ROS2 包

| 包 | 功能 | 状态 |
|---|---|---|
| drone_interfaces | Detection2D, DroneStatus, InferenceResult | 已就绪 |
| drone_vision | V4L2 USB 摄像头 → ROS Image (MJPG 1280x720@25fps) | **已实测通过** |
| drone_inference | YOLOv8n INT8 + SpaceMIT EP 人物检测 | **已实测通过** |
| drone_communication | UART ↔ MAVLink v2 (pyserial + pymavlink) | **已实测通过** |
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

- 后处理加速（DFL+NMS 占 47% 耗时）
- 检测框坐标映射回原图（当前是模型输入空间坐标）
- MAVLink 节点最大重连次数限制

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
