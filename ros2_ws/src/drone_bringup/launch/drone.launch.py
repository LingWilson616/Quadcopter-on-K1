from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_bringup')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('baud_rate', default_value='57600'),
        DeclareLaunchArgument('uart_device', default_value='/dev/ttyS0'),

        # Camera node
        Node(
            package='drone_vision',
            executable='camera_node',
            name='camera_node',
            parameters=[params_file],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # Inference node (object detection)
        Node(
            package='drone_inference',
            executable='inference_node',
            name='inference_node',
            parameters=[params_file],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # Communication node (MAVLink over UART)
        Node(
            package='drone_communication',
            executable='mavlink_node',
            name='mavlink_node',
            parameters=[params_file],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # Weather detection node
        Node(
            package='drone_inference',
            executable='weather_node',
            name='weather_node',
            parameters=[params_file],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
