#!/usr/bin/env python3

import argparse
import json
import time
from datetime import datetime, timedelta, timezone

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import hbm_runtime
except ImportError:
    hbm_runtime = None

try:
    from hobot_dnn import pyeasy_dnn as easy_dnn
except ImportError:
    easy_dnn = None


COCO_SKELETON = [
    [16, 14], [14, 12], [17, 15], [15, 13], [12, 13], [6, 12], [7, 13], [6, 7],
    [6, 8], [7, 9], [8, 10], [9, 11], [2, 3], [1, 2], [1, 3], [2, 4], [3, 5],
    [4, 6], [5, 7],
]

COCO_KEYPOINT_NAMES = [
    'nose',
    'left_eye',
    'right_eye',
    'left_ear',
    'right_ear',
    'left_shoulder',
    'right_shoulder',
    'left_elbow',
    'right_elbow',
    'left_wrist',
    'right_wrist',
    'left_hip',
    'right_hip',
    'left_knee',
    'right_knee',
    'left_ankle',
    'right_ankle',
]

CN_TZ = timezone(timedelta(hours=8))


def parse_args():
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument('--config', default='')
    config_args, _ = config_parser.parse_known_args()
    config = load_simple_yaml(config_args.config) if config_args.config else {}

    parser = argparse.ArgumentParser(
        description='Official RDK X5 YOLOv8 pose ROS2 inference node.',
        parents=[config_parser],
    )
    parser.add_argument('--model', default=config.get('model', '/root/models/yolo/yolov8n-pose_bayese_640x640_nv12.bin'))
    parser.add_argument('--image-topic', default=config.get('image_topic', '/camera/image_raw'))
    parser.add_argument('--result-topic', default=config.get('result_topic', '/person_pose/result_image'))
    parser.add_argument('--detections-topic', default=config.get('detections_topic', '/person_pose/detections'))
    parser.add_argument('--status-topic', default=config.get('status_topic', '/person_pose/status'))
    parser.add_argument('--score-thres', type=float, default=float(config.get('score_thres', 0.25)))
    parser.add_argument('--nms-thres', type=float, default=float(config.get('nms_thres', 0.70)))
    parser.add_argument('--kpt-conf-thres', type=float, default=float(config.get('kpt_conf_thres', 0.50)))
    parser.add_argument('--resize-type', type=int, default=int(config.get('resize_type', 1)), choices=(0, 1))
    parser.add_argument('--input-size', type=int, default=int(config.get('input_size', 640)))
    parser.add_argument('--max-detections', type=int, default=int(config.get('max_detections', 50)))
    parser.add_argument('--debug-output', action='store_true', default=bool(config.get('debug_output', False)))
    return parser.parse_args()


def iso_now():
    return datetime.now(CN_TZ).isoformat(timespec='milliseconds')


def parse_scalar(value):
    value = value.strip().strip('"').strip("'")
    lower = value.lower()
    if lower in ('true', 'yes', 'on'):
        return True
    if lower in ('false', 'no', 'off'):
        return False
    try:
        if any(char in value for char in ('.', 'e', 'E')):
            return float(value)
        return int(value)
    except ValueError:
        return value


def load_simple_yaml(path):
    config = {}
    with open(path, 'r', encoding='utf-8') as file:
        for line in file:
            line = line.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, value = line.split(':', 1)
            config[key.strip().replace('-', '_')] = parse_scalar(value)
    return config


def sigmoid(value):
    return 1.0 / (1.0 + np.exp(-value))


def softmax(value, axis=-1):
    value = value - np.max(value, axis=axis, keepdims=True)
    exp = np.exp(value)
    return exp / np.sum(exp, axis=axis, keepdims=True)


def resized_image(image, input_w, input_h, resize_type=1):
    height, width = image.shape[:2]
    if resize_type == 0:
        return cv2.resize(image, (input_w, input_h), interpolation=cv2.INTER_NEAREST)

    scale = min(input_h / height, input_w / width)
    new_w = int(width * scale)
    new_h = int(height * scale)
    resized = cv2.resize(image, (new_w, new_h))
    pad_w = input_w - new_w
    pad_h = input_h - new_h
    left = pad_w // 2
    right = pad_w - left
    top = pad_h // 2
    bottom = pad_h - top
    return cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(127, 127, 127))


def bgr_to_nv12_planes(image):
    height, width = image.shape[:2]
    area = height * width
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = yuv420p[:area].reshape((height, width))
    u = yuv420p[area:area + area // 4].reshape((height // 2, width // 2))
    v = yuv420p[area + area // 4:].reshape((height // 2, width // 2))
    uv = np.stack((u, v), axis=-1)
    return y[np.newaxis, :, :, np.newaxis], uv[np.newaxis, :, :, :]


def gen_anchor(grid_size):
    x = np.tile(np.linspace(0.5, grid_size - 0.5, grid_size), reps=grid_size)
    y = np.repeat(np.linspace(0.5, grid_size - 0.5, grid_size), grid_size)
    return np.stack([x, y], axis=1)


def filter_classification(cls_output, conf_thres_raw):
    cls_output = cls_output.reshape(-1, cls_output.shape[-1])
    max_scores = np.max(cls_output, axis=1)
    valid_indices = np.flatnonzero(max_scores >= conf_thres_raw)
    ids = np.argmax(cls_output[valid_indices], axis=1)
    scores = sigmoid(max_scores[valid_indices])
    return scores, ids, valid_indices


def decode_boxes(box_output, valid_indices, grid_size, stride, weights_static):
    boxes = box_output.reshape(-1, box_output.shape[-1])
    boxes = boxes[valid_indices]
    ltrb = np.sum(softmax(boxes.reshape(-1, 4, 16), axis=2) * weights_static, axis=2)
    anchor = gen_anchor(grid_size)[valid_indices]
    x1y1 = anchor - ltrb[:, 0:2]
    x2y2 = anchor + ltrb[:, 2:4]
    return np.hstack([x1y1, x2y2]) * stride


def decode_kpts(kpt_output, valid_indices, grid_size, stride, anchor=None):
    kpt_output = kpt_output.reshape(-1, kpt_output.shape[-1])
    kpts = kpt_output[valid_indices].reshape(-1, 17, 3)
    if anchor is None:
        anchor = gen_anchor(grid_size)[valid_indices]
    kpts_xy = (kpts[:, :, :2] * 2.0 + (anchor[:, None, :] - 0.5)) * stride
    return kpts_xy, kpts[:, :, 2:3]


def nms(boxes, scores, iou_thresh):
    if len(boxes) == 0:
        return []

    x1, y1, x2, y2 = boxes.T
    area = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        iou = inter / np.maximum(area[i] + area[order[1:]] - inter, 1e-9)
        order = order[1:][iou < iou_thresh]

    return keep


def scale_coords_back(boxes, img_w, img_h, input_w, input_h, resize_type):
    if resize_type == 0:
        boxes[:, [0, 2]] *= img_w / input_w
        boxes[:, [1, 3]] *= img_h / input_h
    else:
        scale = min(input_w / img_w, input_h / img_h)
        pad_w = (input_w - img_w * scale) / 2
        pad_h = (input_h - img_h * scale) / 2
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_w) / scale
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_h) / scale
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, img_w)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, img_h)
    return boxes


def scale_keypoints_back(kpts_xy, kpts_score, img_w, img_h, input_w, input_h, resize_type):
    if resize_type == 0:
        kpts_xy[:, :, 0] *= img_w / input_w
        kpts_xy[:, :, 1] *= img_h / input_h
    else:
        scale = min(input_w / img_w, input_h / img_h)
        pad_w = (input_w - img_w * scale) / 2
        pad_h = (input_h - img_h * scale) / 2
        kpts_xy[:, :, 0] = (kpts_xy[:, :, 0] - pad_w) / scale
        kpts_xy[:, :, 1] = (kpts_xy[:, :, 1] - pad_h) / scale
    kpts_xy[:, :, 0] = np.clip(kpts_xy[:, :, 0], 0, img_w)
    kpts_xy[:, :, 1] = np.clip(kpts_xy[:, :, 1], 0, img_h)
    return np.concatenate([kpts_xy, kpts_score], axis=-1)


def draw_pose(image, boxes, scores, keypoints, kpt_conf_thres):
    result = image.copy()
    for index, (box, score, kpts) in enumerate(zip(boxes, scores, keypoints), start=1):
        target_id = f'Victim-{index:02d}'
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(result, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(result, f'{target_id} person {score:.2f}', (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        for x, y, conf in kpts:
            if conf >= kpt_conf_thres:
                cv2.circle(result, (int(x), int(y)), 3, (0, 0, 255), -1)
        for start, end in COCO_SKELETON:
            idx1 = start - 1
            idx2 = end - 1
            if kpts[idx1, 2] >= kpt_conf_thres and kpts[idx2, 2] >= kpt_conf_thres:
                p1 = (int(kpts[idx1, 0]), int(kpts[idx1, 1]))
                p2 = (int(kpts[idx2, 0]), int(kpts[idx2, 1]))
                cv2.line(result, p1, p2, (255, 0, 0), 1)
    return result


class OfficialPersonPoseNode(Node):
    def __init__(self, args):
        super().__init__('person_pose_node_official')
        self.args = args
        self.bridge = CvBridge()
        self.busy = False
        self.last_log_time = 0.0
        self.conf_thres_raw = -np.log(1.0 / args.score_thres - 1.0)
        self.weights_static = np.arange(16, dtype=np.float32)[None, None, :]

        self.get_logger().info(f'loading official protocol model: {args.model}')
        self.runtime = 'hbm_runtime' if hbm_runtime is not None else 'pyeasy_dnn'
        if self.runtime == 'hbm_runtime':
            self.model = hbm_runtime.HB_HBMRuntime(args.model)
            self.model_name = self.model.model_names[0]
            self.input_names = self.model.input_names[self.model_name]
            self.output_names = self.model.output_names[self.model_name]
            input_shape = self.model.input_shapes[self.model_name][self.input_names[0]]
            if input_shape[1] == 3:
                self.input_h = input_shape[2]
                self.input_w = input_shape[3]
            else:
                self.input_h = input_shape[1]
                self.input_w = input_shape[2]

            if len(self.output_names) != 9:
                raise RuntimeError(f'Expected 9 outputs [cls, box, keypoints] * 3, got {len(self.output_names)}: {self.output_names}')
            self.get_logger().info(f'model loaded with hbm_runtime: input={input_shape}, outputs={self.output_names}')
        else:
            if easy_dnn is None:
                raise RuntimeError('Neither hbm_runtime nor hobot_dnn.pyeasy_dnn is available')
            self.model = easy_dnn.load(args.model)[0]
            self.model_name = 'pyeasy_dnn_model'
            self.input_names = ['images']
            self.output_names = [f'output_{index}' for index in range(9)]
            self.input_h = args.input_size
            self.input_w = args.input_size
            self.get_logger().info(
                f'model loaded with pyeasy_dnn fallback: input=NV12 {self.input_w}x{self.input_h}, '
                f'expected outputs=[cls, box, keypoints] * 3'
            )
        self.sub = self.create_subscription(Image, args.image_topic, self.image_callback, 1)
        self.result_pub = self.create_publisher(Image, args.result_topic, 1)
        self.detections_pub = self.create_publisher(String, args.detections_topic, 10)
        self.status_pub = self.create_publisher(String, args.status_topic, 10)
        self.last_status_pub_time = 0.0
        self.get_logger().info(f'subscribed: {args.image_topic}')
        self.get_logger().info(f'publishing result image: {args.result_topic}')
        self.get_logger().info(f'publishing detections JSON: {args.detections_topic}')
        self.get_logger().info(f'publishing status JSON: {args.status_topic}')

    def preprocess(self, image):
        resized = resized_image(image, self.input_w, self.input_h, self.args.resize_type)
        y, uv = bgr_to_nv12_planes(resized)
        return np.concatenate([y.reshape(-1), uv.reshape(-1)]).astype(np.uint8)

    def forward(self, input_tensor):
        if self.runtime == 'hbm_runtime':
            return self.model.run({self.model_name: {self.input_names[0]: input_tensor}})

        try:
            outputs = self.model.forward(input_tensor)
        except TypeError:
            outputs = self.model.forward([input_tensor])

        if len(outputs) != 9:
            raise RuntimeError(f'Expected 9 outputs [cls, box, keypoints] * 3, got {len(outputs)}')

        raw_outputs = {}
        for name, output in zip(self.output_names, outputs):
            raw_outputs[name] = output.buffer if hasattr(output, 'buffer') else output
        return {self.model_name: raw_outputs}

    def postprocess(self, outputs, image_shape):
        raw_outputs = outputs[self.model_name]
        boxes_all = []
        scores_all = []
        kpts_xy_all = []
        kpts_score_all = []

        for level_index, stride in enumerate((8, 16, 32)):
            base_idx = level_index * 3
            cls_output = raw_outputs[self.output_names[base_idx]].reshape(-1, 1)
            box_output = raw_outputs[self.output_names[base_idx + 1]]
            kpt_output = raw_outputs[self.output_names[base_idx + 2]]

            scores, _, valid_indices = filter_classification(cls_output, self.conf_thres_raw)
            if valid_indices.size == 0:
                continue

            grid_size = self.input_h // stride
            boxes = decode_boxes(box_output, valid_indices, grid_size, stride, self.weights_static)
            anchor = gen_anchor(grid_size)[valid_indices]
            kpts_xy, kpts_score = decode_kpts(kpt_output, valid_indices, grid_size, stride, anchor)

            boxes_all.append(boxes)
            scores_all.append(scores)
            kpts_xy_all.append(kpts_xy)
            kpts_score_all.append(sigmoid(kpts_score))

        if not boxes_all:
            return np.empty((0, 4), dtype=np.float32), np.empty((0,), dtype=np.float32), np.empty((0, 17, 3), dtype=np.float32)

        boxes = np.concatenate(boxes_all, axis=0).astype(np.float32)
        scores = np.concatenate(scores_all, axis=0).astype(np.float32)
        kpts_xy = np.concatenate(kpts_xy_all, axis=0).astype(np.float32)
        kpts_score = np.concatenate(kpts_score_all, axis=0).astype(np.float32)

        keep = nms(boxes, scores, self.args.nms_thres)[:self.args.max_detections]
        boxes = boxes[keep]
        scores = scores[keep]
        kpts_xy = kpts_xy[keep]
        kpts_score = kpts_score[keep]

        height, width = image_shape[:2]
        boxes = scale_coords_back(boxes, width, height, self.input_w, self.input_h, self.args.resize_type)
        keypoints = scale_keypoints_back(kpts_xy, kpts_score, width, height, self.input_w, self.input_h, self.args.resize_type)
        return boxes, scores, keypoints

    def publish_json(self, publisher, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        publisher.publish(msg)

    def make_detection_payload(self, msg, image, boxes, scores, keypoints, fps, latency_ms):
        height, width = image.shape[:2]
        detections = []
        for index, (box, score, kpts) in enumerate(zip(boxes, scores, keypoints), start=1):
            x1, y1, x2, y2 = [int(round(float(value))) for value in box]
            kpt_items = []
            for name, (u, v, confidence) in zip(COCO_KEYPOINT_NAMES, kpts):
                kpt_items.append({
                    'name': name,
                    'u': int(round(float(u))),
                    'v': int(round(float(v))),
                    'confidence': max(0.0, min(1.0, float(confidence))),
                })
            detections.append({
                'id': f'Victim-{index:02d}',
                'track_id': index,
                'label': 'person',
                'confidence': max(0.0, min(1.0, float(score))),
                'bbox': {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2},
                'center': {'u': int(round((x1 + x2) / 2.0)), 'v': int(round((y1 + y2) / 2.0))},
                'keypoints': kpt_items,
                'status': 'detected',
            })
        return {
            'timestamp': iso_now(),
            'frame_id': msg.header.frame_id or 'hik_camera',
            'image': {
                'width': int(width),
                'height': int(height),
                'encoding': 'bgr8',
            },
            'inference': {
                'model': 'yolov8n-pose',
                'fps': round(float(fps), 2),
                'latency_ms': round(float(latency_ms), 2),
            },
            'detections': detections,
        }

    def make_status_payload(self, image=None, fps=0.0, latency_ms=0.0, running=True, last_error=''):
        width = int(image.shape[1]) if image is not None else 0
        height = int(image.shape[0]) if image is not None else 0
        return {
            'timestamp': iso_now(),
            'camera': {
                'online': image is not None,
                'fps': round(float(fps), 2),
                'width': width,
                'height': height,
            },
            'inference': {
                'running': bool(running),
                'model': 'yolov8n-pose',
                'fps': round(float(fps), 2),
                'latency_ms': round(float(latency_ms), 2),
                'last_error': str(last_error or ''),
            },
        }

    def publish_status_throttled(self, payload, force=False):
        now = time.time()
        if force or now - self.last_status_pub_time >= 1.0:
            self.publish_json(self.status_pub, payload)
            self.last_status_pub_time = now

    def image_callback(self, msg):
        if self.busy:
            return
        self.busy = True
        start = time.time()
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            input_tensor = self.preprocess(image)
            outputs = self.forward(input_tensor)
            boxes, scores, keypoints = self.postprocess(outputs, image.shape)
            result = draw_pose(image, boxes, scores, keypoints, self.args.kpt_conf_thres)
            latency_ms = (time.time() - start) * 1000.0
            fps = 1000.0 / max(latency_ms, 1e-6)

            out_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
            out_msg.header = msg.header
            self.result_pub.publish(out_msg)
            self.publish_json(
                self.detections_pub,
                self.make_detection_payload(msg, image, boxes, scores, keypoints, fps, latency_ms),
            )
            self.publish_status_throttled(
                self.make_status_payload(image, fps, latency_ms, running=True, last_error='')
            )

            now = time.time()
            if now - self.last_log_time > 2.0:
                if self.args.debug_output:
                    self.get_logger().info(f'detected={len(boxes)} fps={fps:.1f} score_max={scores.max(initial=0.0):.4f}')
                else:
                    self.get_logger().info(f'detected={len(boxes)} fps={fps:.1f}')
                self.last_log_time = now
        except Exception as exc:
            self.get_logger().error(f'inference failed: {exc}')
            self.publish_status_throttled(
                self.make_status_payload(None, running=False, last_error=str(exc)),
                force=True,
            )
        finally:
            self.busy = False


def main():
    args = parse_args()
    rclpy.init()
    node = OfficialPersonPoseNode(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
