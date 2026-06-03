from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('baud_rate', default_value='57600'),
        DeclareLaunchArgument('uart_device', default_value='/dev/ttyS0'),

        # Camera node
        Node(
            package='drone_vision',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'camera_device': LaunchConfiguration('camera_device'),
                'camera_format': 'YUYV',
                'image_width': 640,
                'image_height': 480,
                'fps': 30,
            }],
            output='screen',
        ),

        # Inference node (object detection)
        Node(
            package='drone_inference',
            executable='inference_node',
            name='inference_node',
            parameters=[{
                'model_path': '/home/bianbu/drone_project/models/spacemit_demo/yolov8n_int8.onnx',
                'confidence_threshold': 0.5,
                'use_npu': True,
            }],
            output='screen',
        ),

        # Communication node (MAVLink over UART)
        Node(
            package='drone_communication',
            executable='mavlink_node',
            name='mavlink_node',
            parameters=[{
                'uart_device': LaunchConfiguration('uart_device'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            }],
            output='screen',
        ),

        # Weather detection node
        Node(
            package='drone_inference',
            executable='weather_node',
            name='weather_node',
            parameters=[{
                'model_path': '/home/bianbu/drone_project/models/weather/weather_model.onnx',
            }],
            output='screen',
        ),
    ])
