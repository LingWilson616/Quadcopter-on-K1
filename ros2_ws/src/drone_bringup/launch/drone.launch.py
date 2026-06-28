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
        DeclareLaunchArgument('camera_device', default_value='/dev/video20'),
        DeclareLaunchArgument('camera_format', default_value='MJPG'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        DeclareLaunchArgument('fps', default_value='25'),
        DeclareLaunchArgument('uart_device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='57600'),

        Node(
            package='drone_vision',
            executable='camera_node',
            name='camera_node',
            parameters=[params_file, {
                'camera_device': LaunchConfiguration('camera_device'),
                'camera_format': LaunchConfiguration('camera_format'),
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='drone_inference',
            executable='inference_node',
            name='inference_node',
            parameters=[params_file],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='drone_communication',
            executable='mavlink_node',
            name='mavlink_node',
            parameters=[params_file, {
                'uart_device': LaunchConfiguration('uart_device'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # Voice interaction node
        Node(
            package='drone_voice',
            executable='voice_node',
            name='voice_node',
            output='screen',
        ),
    ])
