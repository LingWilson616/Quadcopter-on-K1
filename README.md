# Quadcopter-on-K1

# Quadcopter-on-K1

基于进迭时空 SPACEMIT K1 (MUSE Pi Pro) 的四轴无人机系统

全国大学生嵌入式比赛项目。

## 项目概况

- **主控板**: K1 (MUSE Pi Pro) — 模型量化 + 无人机通信（ROS2 Humble）
- **队友**: 纪同学（气象识别模型 PyTorch）、黎同学（ArduPilot 飞控）
- **飞控**: MicoAir743-AIO (ArduPilot)，通过 USB CDC ACM 连接 K1
- **系统架构**: Bianbu OS → ROS2 Humble → pyserial + pymavlink

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
    ↓ /dev/ttyACM0 (MAVLink v2)
ArduPilot 飞控（黎同学）
    │
    ↑ 反馈数据
    │ ATTITUDE, GLOBAL_POSITION_INT, SYS_STATUS ...
drone_communication (mavlink_node)
    ↓ /drone/status (5Hz 发布)
控制台 / 其他 ROS2 节点
```

**双向通信**：mavlink_node 不仅往下发指令（ARM/TAKEOFF/LAND/RTL），还持续接收飞控遥测（姿态角、GPS位置、速度、电池），通过 `/drone/status` 话题发布。

## ROS2 包

| 包 | 类型 | 功能 | 状态 |
|---|---|---|---|
| drone_interfaces | msg | Detection2D, DroneStatus, InferenceResult 消息定义 | 已就绪 |
| drone_communication | Python | UART ↔ MAVLink v2 ↔ ArduPilot（pyserial + pymavlink） | **已实测通过** |
| drone_inference | Python | ONNX Runtime 推理（目标检测+气象分类） | 框架就绪，待模型 |
| drone_vision | Python | V4L2 摄像头采集 → ROS Image | 已就绪 |
| drone_bringup | launch | 一键启动所有节点 + 参数管理 | 已就绪 |

### MAVLink 节点详情

`drone_communication/mavlink_node.py`（~255 行）— 已通过 K1 + MicoAir743-AIO 实测验证。

**设计决策**：不使用 pymavlink 的 `mavutil`（K1 USB CDC ACM 驱动间歇性空读导致 mavutil 频繁误判断连），改用 pyserial 直读 + `parse_char` 逐字节解析，容忍 ACM 抽风。

**错误处理分层**：
1. ACM 空读 → 重试 → 重建串口
2. CRC 噪声 → 跳过坏字节 → parse_char 内部自动重同步
3. 未预期异常 → 打 traceback → 线程不死

**关键特性**：
- 心跳保持（1Hz）→ 飞控数据流不超时
- 自动断线重连（1.5s 恢复）
- MAVLink 命令支持：ARM / TAKEOFF / LAND / RTL / GUIDED
- 5Hz ROS2 发布，数据实时反映飞控角度变化

## 快速开始

### 1. 环境准备（K1 板）

```bash
# ROS2 Humble（K1 预装或手动安装）
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions

# 项目依赖
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport python3-serial
pip3 install pymavlink
```

### 2. 部署与编译

```bash
# 本地（Windows 主机）→ K1
scp -r ros2_ws/src/* bianbu@<board-ip>:~/drone_project/ros2_ws/src/

# K1 板
ssh bianbu@<board-ip>
source /opt/ros/humble/setup.bash
cd ~/drone_project/ros2_ws
colcon build --symlink-install
```

### 3. 手动测试 MAVLink

```bash
source /opt/ros/humble/setup.bash
source ~/drone_project/ros2_ws/install/setup.bash

# 启动节点（如遇权限问题：sudo chmod 666 /dev/ttyACM0）
ros2 run drone_communication mavlink_node \
  --ros-args -p uart_device:=/dev/ttyACM0 -p baud_rate:=57600

# 另一个终端：查看飞控实时数据
source /opt/ros/humble/setup.bash
source ~/drone_project/ros2_ws/install/setup.bash
ros2 topic echo /drone/status

# 单次查看
timeout 5 ros2 topic echo /drone/status --once

# 发送指令
ros2 topic pub /drone/command std_msgs/msg/String "{data: 'ARM'}"
ros2 topic pub /drone/command std_msgs/msg/String "{data: 'TAKEOFF'}"
ros2 topic pub /drone/command std_msgs/msg/String "{data: 'LAND'}"
ros2 topic pub /drone/command std_msgs/msg/String "{data: 'RTL'}"
ros2 topic pub /drone/command std_msgs/msg/String "{data: 'GUIDED 22.5 113.9 50'}"
```

### 4. Launch 方式启动（全部节点）

```bash
ros2 launch drone_bringup drone.launch.py uart_device:=/dev/ttyACM0
```

### 5. 调参（无需重新编译）

```bash
ros2 launch drone_bringup drone.launch.py \
  camera_device:=/dev/video1 \
  uart_device:=/dev/ttyACM0 \
  baud_rate:=115200
```

硬件参数在 `config/hardware.yaml` 中配置，launch 文件运行时从 `params.yaml` 加载。

## TODO

- `drone_inference/inference_node.py` — YOLO 后处理（模型输出转检测框）
- 模型量化流程（Spacemit NPU 执行提供者）
- 纪同学气象模型集成（`weather_node` 中确认 weather_classes）

## 关键资源

- 进迭时空 ROS2 镜像: https://archive.spacemit.com/ros2/
- ROS2 K1 文档: https://cdn-resource.spacemit.com/software/SDK/ros/docs-ros/zh/k1/intro.md
- 进迭时空 AI Robot: MediaEngine（零拷贝/OpenCL加速）、RDK（量化/Model Zoo）
