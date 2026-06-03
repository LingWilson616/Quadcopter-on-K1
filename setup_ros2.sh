#!/bin/bash
# setup_ros2.sh - Build and setup ROS2 workspace for K1 drone project
# Usage: ./setup_ros2.sh

set -e

cd ~/drone_project/ros2_ws

# Source ROS2 (bros first, then standard humble)
if [ -f /opt/bros/humble/setup.bash ]; then
    source /opt/bros/humble/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "ROS2 not found. Install with: sudo apt install -y ros-humble-ros-base"
    exit 1
fi

# Install dependencies
echo "Installing dependencies..."
sudo apt install -y python3-pip python3-serial python3-colcon-common-extensions
pip3 install --user --break-system-packages pyserial onnxruntime opencv-python pymavlink

# Build
echo "Building ROS2 workspace..."
colcon build --symlink-install

echo "Source setup script:"
echo "  source ~/drone_project/ros2_ws/install/setup.bash"

echo "Run:"
echo "  ros2 launch drone_bringup drone.launch.py"
