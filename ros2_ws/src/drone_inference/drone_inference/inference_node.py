#!/usr/bin/env python3
"""YOLOv8 ONNX inference node for person detection on K1."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from drone_interfaces.msg import InferenceResult, Detection2D
import numpy as np
import cv2
import time

# CRITICAL: onnxruntime must be imported BEFORE spacemit_ort (order matters for EP patch)
import onnxruntime
import spacemit_ort
ort = onnxruntime

COCO_NAMES = {
    0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 5: 'bus',
    7: 'truck', 15: 'cat', 16: 'dog', 39: 'bottle', 56: 'chair',
    63: 'laptop', 67: 'cell phone',
}


class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('num_threads', 4)
        self.declare_parameter('person_only', True)

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        num_threads = self.get_parameter('num_threads').value
        self.person_only = self.get_parameter('person_only').value

        self.session = None
        self.input_name = None
        self.output_names = None
        self.input_shape = None

        self._load_model(model_path, num_threads)

        self.bridge = CvBridge()
        self.result_pub = self.create_publisher(InferenceResult, '/drone/inference_result', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.get_logger().info(
            f'Inference node ready (model={model_path.split("/")[-1]}, '
            f'input={self.input_shape}, person_only={self.person_only})'
        )

    # ── model ─────────────────────────────────────────────────

    def _load_model(self, path, num_threads):
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = num_threads
        self.session = ort.InferenceSession(
            path, opts, ['SpaceMITExecutionProvider'],
        )
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = tuple(self.session.get_inputs()[0].shape[2:4])
        self.output_names = [o.name for o in self.session.get_outputs()]
        self.get_logger().info(f'Model loaded: {path}')
        self.get_logger().info(f'Input: {self.input_name} {self.session.get_inputs()[0].shape}')

    # ── callback ──────────────────────────────────────────────

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        t0 = time.time()
        tensor = self._preprocess(img)
        outputs = self.session.run(self.output_names, {self.input_name: tensor})
        boxes, classes, scores = self._postprocess(outputs)
        elapsed = (time.time() - t0) * 1000

        result = InferenceResult()
        result.inference_time_ms = float(elapsed)
        result.task_type = 'object_detection'
        result.model_name = self.get_parameter('model_path').value.split('/')[-1]

        if boxes is not None and len(boxes) > 0:
            result.detections = self._to_detections(boxes, classes, scores)

        self.result_pub.publish(result)

    # ── preprocess (OpenCV RVV-accelerated) ───────────────────

    def _preprocess(self, image):
        ih, iw = image.shape[:2]
        th, tw = self.input_shape

        r = min(tw / iw, th / ih)
        nw, nh = int(round(iw * r)), int(round(ih * r))
        dw, dh = (tw - nw) / 2, (th - nh) / 2

        if (nw, nh) != (iw, ih):
            image = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        image = cv2.copyMakeBorder(image, top, bottom, left, right,
                                    cv2.BORDER_CONSTANT, value=(0, 0, 0))
        image = cv2.normalize(image, None, 0, 1, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        image = np.expand_dims(np.transpose(image, (2, 0, 1)), axis=0)
        return image

    # ── postprocess ───────────────────────────────────────────

    def _postprocess(self, outputs):
        n_branches = 3
        per = len(outputs) // n_branches

        all_boxes, all_scores, all_cls_conf = [], [], []
        for i in range(n_branches):
            all_boxes.append(self._box_decode(outputs[per * i]))
            all_cls_conf.append(outputs[per * i + 1])
            all_scores.append(
                np.ones_like(outputs[per * i + 1][:, :1, :, :], dtype=np.float32)
            )

        def flatten(x):
            return x.transpose(0, 2, 3, 1).reshape(-1, x.shape[1])

        boxes = np.concatenate([flatten(v) for v in all_boxes])
        cls_conf = np.concatenate([flatten(v) for v in all_cls_conf])
        scores = np.concatenate([flatten(v) for v in all_scores])

        boxes, classes, scores = self._filter_boxes(boxes, scores, cls_conf)

        if boxes is None:
            return None, None, None

        # Per-class NMS
        nboxes, nclasses, nscores = [], [], []
        for c in set(classes):
            inds = np.where(classes == c)
            keep = self._nms(boxes[inds], scores[inds])
            if len(keep):
                nboxes.append(boxes[inds][keep])
                nclasses.append(classes[inds][keep])
                nscores.append(scores[inds][keep])

        if not nboxes:
            return None, None, None

        return np.concatenate(nboxes), np.concatenate(nclasses), np.concatenate(nscores)

    def _box_decode(self, pos):
        th, tw = self.input_shape
        gh, gw = pos.shape[2:4]
        col, row = np.meshgrid(np.arange(gw), np.arange(gh))
        grid = np.concatenate(
            (col.reshape(1, 1, gh, gw), row.reshape(1, 1, gh, gw)), axis=1
        )
        stride = np.array([tw / gw, th / gh]).reshape(1, 2, 1, 1)

        pos = self._dfl(pos)
        box_xy = grid + 0.5 - pos[:, 0:2, :, :]
        box_xy2 = grid + 0.5 + pos[:, 2:4, :, :]
        return np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)

    def _dfl(self, pos):
        n, c, h, w = pos.shape
        p_num = 4
        mc = c // p_num
        y = pos.reshape(n, p_num, mc, h, w)
        y = np.exp(y - np.max(y, axis=2, keepdims=True))
        y = y / np.sum(y, axis=2, keepdims=True)
        acc = np.arange(mc, dtype=np.float32).reshape(1, 1, mc, 1, 1)
        return (y * acc).sum(axis=2)

    def _filter_boxes(self, boxes, box_conf, cls_probs):
        box_conf = box_conf.reshape(-1)
        cls_max_score = np.max(cls_probs, axis=-1)
        classes = np.argmax(cls_probs, axis=-1)

        if self.person_only:
            person_mask = classes == 0
            cls_max_score = cls_max_score * person_mask

        score = cls_max_score * box_conf
        keep = np.where(score >= self.conf_threshold)

        if len(keep[0]) == 0:
            return None, None, None

        return boxes[keep], classes[keep], score[keep]

    def _nms(self, boxes, scores):
        x, y = boxes[:, 0], boxes[:, 1]
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]

        areas = w * h
        order = scores.argsort()[::-1]
        keep = []

        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x[i], x[order[1:]])
            yy1 = np.maximum(y[i], y[order[1:]])
            xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
            yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

            inter = np.maximum(0.0, xx2 - xx1 + 1e-5) * np.maximum(0.0, yy2 - yy1 + 1e-5)
            iou = inter / (areas[i] + areas[order[1:]] - inter)
            order = order[np.where(iou <= self.iou_threshold)[0] + 1]

        return np.array(keep)

    # ── helpers ───────────────────────────────────────────────

    def _to_detections(self, boxes, classes, scores):
        dets = []
        for i in range(len(boxes)):
            d = Detection2D()
            d.class_id = int(classes[i])
            d.confidence = float(scores[i])
            d.x_min = float(boxes[i][0])
            d.y_min = float(boxes[i][1])
            d.x_max = float(boxes[i][2])
            d.y_max = float(boxes[i][3])
            d.label = COCO_NAMES.get(d.class_id, f'class_{d.class_id}')
            dets.append(d)
        return dets

    def destroy(self):
        if self.session is not None:
            del self.session
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
