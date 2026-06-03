# Quadcopter-on-K1

基于进迭时空 SPACEMIT K1 (MUSE Pi Pro) 的四轴无人机飞控系统

全国大学生嵌入式比赛项目。

## 项目概况

- **主控板**: K1 (MUSE Pi Pro) — 模型量化 + 无人机通信
- **队友**: 纪同学（气象识别模型 PyTorch）、黎同学（ArduPilot 飞控 GPIO UART）
- **系统架构**: Bianbu OS → ROS2 (Humble) → MediaEngine → RDK

## 数据流

```
摄像头(MIPI/USB)
    ↓
drone_vision (camera_node)
    ↓ /camera/image_raw
drone_inference (inference_node)
    │ ├─ 目标检测 (YOLOv8n INT8)
    │ └─ 气象分类 (weather_model)
    ↓ /drone/inference_result
drone_communication (mavlink_node)
    ↓ UART (MAVLink v2)
ArduPilot 飞控（黎同学）
```

## ROS2 包

| 包 | 类型 | 功能 |
|---|---|---|
| drone_interfaces | msg | Detection2D, DroneStatus, InferenceResult |
| drone_communication | Python | UART ↔ MAVLink 解析 ↔ 飞控指令 |
| drone_inference | Python | ONNX Runtime 推理（NPU加速预留） |
| drone_vision | Python | V4L2 摄像头采集 → ROS Image |
| drone_bringup | launch | 一键启动所有节点 |

## 快速开始

### 1. ROS2 安装（K1 板）

```bash
# 添加 noble-ros 源（如尚未添加）
sudo sed -i 's/bianbu-v2.3-updates$/bianbu-v2.3-updates noble-ros/' /etc/apt/sources.list.d/bianbu.sources
sudo apt update

# 安装 ROS2 Humble
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions

# 安装项目依赖
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport python3-serial

# 安装进迭时空 bros（可选，提供 ByteTrack 等额外包）
curl -L https://archive.spacemit.com/ros2/bros.tar.gz -o /tmp/bros.tar.gz
sudo tar -xzf /tmp/bros.tar.gz -C /opt/
echo 'source /opt/bros/humble/setup.bash' >> ~/.bashrc
```

### 2. 部署与编译

```bash
# 本地（Windows 主机）— 将代码传到板子
scp -r ros2_ws/src/* bianbu@<board-ip>:~/drone_project/ros2_ws/src/

# K1 板 — 编译
ssh bianbu@<board-ip>
source /opt/ros/humble/setup.bash
cd ~/drone_project/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 启动
ros2 launch drone_bringup drone.launch.py
```

### 3. 调参（无需重新编译）

```bash
# 通过 launch 参数覆盖
ros2 launch drone_bringup drone.launch.py \
  camera_device:=/dev/video1 \
  uart_device:=/dev/ttyS1 \
  baud_rate:=115200
```

硬件参数在 `config/hardware.yaml` 中配置。

### 4. 调试命令

```bash
ros2 topic list                           # 查看话题
ros2 topic echo /drone/inference_result   # 查看推理结果
ros2 topic echo /drone/status             # 查看无人机状态
ros2 topic pub /drone/command std_msgs/String "{data: 'ARM'}"  # 发送指令
rqt_graph                                 # 查看节点图
```

## TODO（代码待填充）

- `drone_inference/inference_node.py` — YOLO 后处理（模型输出转检测框）
- `drone_communication/mavlink_node.py` — MAVLink 协议解析与命令发送
- 模型量化流程搭建
- 纪同学气象模型集成

## 关键资源

- 进迭时空 ROS2 镜像: https://archive.spacemit.com/ros2/
- ROS2 K1 文档: https://cdn-resource.spacemit.com/software/SDK/ros/docs-ros/zh/k1/intro.md
- 进迭时空 AI Robot: MediaEngine（零拷贝/OpenCL加速）、RDK（量化/Model Zoo）
