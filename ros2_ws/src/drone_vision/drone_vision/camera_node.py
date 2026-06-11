#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('camera_format', 'YUYV')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 30)

        device = self.get_parameter('camera_device').value
        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        fps = self.get_parameter('fps').value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # Open camera via V4L2
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera: {device}')
        else:
            self.get_logger().info(f'Camera opened: {device}')

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub.publish(msg)

    def destroy(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()
