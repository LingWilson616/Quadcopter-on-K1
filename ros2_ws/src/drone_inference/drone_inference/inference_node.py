#!/usr/bin/env python3
"""
ONNX inference node for object detection / weather classification.
Uses ONNX Runtime with Spacemit NPU acceleration.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from drone_interfaces.msg import InferenceResult, Detection2D
import numpy as np
import cv2
import time
import pathlib

class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('use_npu', True)
        self.declare_parameter('task_type', 'object_detection')  # or 'weather_classification'

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.use_npu = self.get_parameter('use_npu').value
        self.task_type = self.get_parameter('task_type').value

        self.session = None
        self.input_shape = (640, 640)  # default YOLO input
        self.class_names = []

        # Load model
        self.load_model(model_path)

        # Publishers
        self.result_pub = self.create_publisher(InferenceResult, '/drone/inference_result', 10)

        # Subscribers
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.get_logger().info(f'Inference node started (task: {self.task_type})')

    def load_model(self, path):
        try:
            import onnxruntime as ort
            providers = ['CPUExecutionProvider']
            if self.use_npu:
                # TODO: Add Spacemit NPU execution provider when available
                pass
            self.session = ort.InferenceSession(path, providers=providers)
            self.input_shape = self.session.get_inputs()[0].shape[2:4]
            self.get_logger().info(f'Model loaded: {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')

    def image_callback(self, msg):
        if self.session is None:
            return

        # Convert ROS Image to OpenCV format (handles any encoding)
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess
        t0 = time.time()
        input_tensor = self.preprocess(img)

        # Inference
        outputs = self.session.run(None, {self.session.get_inputs()[0].name: input_tensor})
        inference_time = (time.time() - t0) * 1000  # ms

        # Postprocess
        result = self.postprocess(outputs)

        # Publish
        result.inference_time_ms = inference_time
        result.task_type = self.task_type
        result.model_name = str(pathlib.Path(self.get_parameter('model_path').value).name)
        self.result_pub.publish(result)

    def preprocess(self, img):
        """Letterbox resize + normalize, convert to NCHW format."""
        h, w = self.input_shape
        # Letterbox: resize preserving aspect ratio, pad to target size
        ih, iw = img.shape[:2]
        scale = min(w / iw, h / ih)
        nw, nh = int(iw * scale), int(ih * scale)
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        # Create padded canvas
        canvas = np.full((h, w, 3), 114, dtype=np.uint8)
        dy, dx = (h - nh) // 2, (w - nw) // 2
        canvas[dy:dy + nh, dx:dx + nw] = resized
        # Normalize
        normalized = canvas.astype(np.float32) / 255.0
        # HWC -> NCHW, add batch dim
        return np.transpose(normalized, (2, 0, 1))[np.newaxis, ...].astype(np.float32)

    def postprocess(self, outputs):
        """Parse model outputs into Detection2D messages."""
        result = InferenceResult()
        # TODO: Implement per-model postprocessing
        return result


class WeatherNode(InferenceNode):
    """Specialized inference node for weather detection."""
    def __init__(self):
        super().__init__()
        self.task_type = 'weather_classification'
        self.weather_classes = ['clear', 'rainy', 'foggy', 'stormy', 'cloudy']


def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


def weather_main(args=None):
    rclpy.init(args=args)
    node = WeatherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()
