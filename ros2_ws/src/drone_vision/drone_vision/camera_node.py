#!/usr/bin/env python3
"""
Camera node for K1 Drone — USB UVC camera via V4L2 backend.
Publishes sensor_msgs/Image on /camera/image_raw.
"""
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def _fourcc(s):
    """Convert four-char string like 'MJPG' to OpenCV FOURCC integer."""
    return cv2.VideoWriter_fourcc(*s)


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('camera_device', '/dev/video20')
        self.declare_parameter('camera_format', 'MJPG')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('fps', 25)

        device = self.get_parameter('camera_device').value
        fmt_str = self.get_parameter('camera_format').value
        req_w = self.get_parameter('image_width').value
        req_h = self.get_parameter('image_height').value
        fps = self.get_parameter('fps').value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self.cap = None
        self._camera_ok = False
        self._consecutive_failures = 0
        self._max_failures = 30

        self.cap = self._open_camera(device, fmt_str, req_w, req_h, fps, fourcc=_fourcc(fmt_str))
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error(
                f'Camera {device} failed to open — node idle, no frames published'
            )
            return

        self._camera_ok = True
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Camera OK: {device} {fmt_str} '
            f'{actual_w:.0f}x{actual_h:.0f}@{actual_fps:.0f}fps '
            f'(requested {req_w}x{req_h}@{fps}fps)'
        )

        self.timer = self.create_timer(1.0 / max(fps, 1), self.timer_callback)

    def _open_camera(self, device, fmt_str, width, height, fps, *, fourcc):
        """Open camera: V4L2 backend first, GStreamer fallback.

        Checks cap.set() return values and warns on mismatch.
        """
        backends = [
            (cv2.CAP_V4L2, 'V4L2'),
            (cv2.CAP_ANY, 'default'),
        ]
        for backend_id, backend_name in backends:
            cap = cv2.VideoCapture(device, backend_id) if backend_id == cv2.CAP_V4L2 \
                else cv2.VideoCapture(device)
            if not cap.isOpened():
                cap.release()
                continue

            # FourCC
            ok = cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            if not ok:
                self.get_logger().warn(
                    f'CAP_PROP_FOURCC not accepted by {backend_name} backend'
                )

            # Resolution
            ok_w = cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            ok_h = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if not (ok_w and ok_h):
                actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                self.get_logger().warn(
                    f'Resolution: requested {width}x{height}, '
                    f'backend negotiated {actual_w:.0f}x{actual_h:.0f}'
                )

            # FPS
            ok_fps = cap.set(cv2.CAP_PROP_FPS, fps)
            if not ok_fps:
                actual_fps = cap.get(cv2.CAP_PROP_FPS)
                self.get_logger().warn(
                    f'FPS: requested {fps}, backend negotiated {actual_fps:.0f}'
                )

            # Read one frame to verify the pipeline works
            ret, _ = cap.read()
            if ret:
                self.get_logger().info(f'Camera opened with {backend_name} backend')
                return cap

            self.get_logger().warn(f'{backend_name} backend opened but read failed')
            cap.release()

        return None

    def timer_callback(self):
        if not self._camera_ok or self.cap is None:
            return

        ret, frame = self.cap.read()
        if ret:
            self._consecutive_failures = 0
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            if rclpy.ok():
                self.pub.publish(msg)
        else:
            self._consecutive_failures += 1
            self.get_logger().warn(
                f'Frame read failed ({self._consecutive_failures}/{self._max_failures})',
                throttle_duration_sec=2.0,
            )
            if self._consecutive_failures >= self._max_failures:
                self.get_logger().error('Camera disconnected — attempting reopen')
                self._reopen()

    def _reopen(self):
        device = self.get_parameter('camera_device').value
        fmt_str = self.get_parameter('camera_format').value
        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        fps = self.get_parameter('fps').value

        if self.cap is not None:
            self.cap.release()

        self.cap = self._open_camera(device, fmt_str, width, height, fps, fourcc=_fourcc(fmt_str))
        self._consecutive_failures = 0

        if self.cap is None or not self.cap.isOpened():
            self._camera_ok = False
            self.get_logger().error('Reopen failed — will retry on next timer tick')
        else:
            self._camera_ok = True
            self.get_logger().info('Camera reopened successfully')

    def destroy(self):
        if self.cap is not None:
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
