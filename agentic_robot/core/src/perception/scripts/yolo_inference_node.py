#!/usr/bin/env python3
"""
YOLO-E GPU Inference Node - Python 3.8 (conda: holoagent_py38)
通过 Redis 中转与 ROS2 节点通信
不依赖 ROS，只依赖 ultralytics + torch
"""

import redis
import pickle
import time
import numpy as np
import torch
import sys
import os
import yaml


class YoloInferenceNode:
    def __init__(self, config_path: str = None):
        # Redis 连接
        self.redis_client = redis.Redis(
            host='localhost',
            port=6379,
            db=0,
            decode_responses=False
        )
        self.pubsub = self.redis_client.pubsub()
        self.pubsub.subscribe('perception_image')

        # 加载配置
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(
                    os.path.abspath(__file__)),
                '../config/config.yaml')

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # 模型配置
        model_config = self.config.get('model', {})
        model_dir = model_config.get('model_dir', 'models_yoloe')
        model_name = model_config.get('model_name_lrpc', 'yoloe-11l-seg-pf.pt')
        prompt_mode = model_config.get('prompt_mode', 'lrpc')
        text_prompts = model_config.get(
            'text_prompts', [
                'person', 'chair', 'table', 'bottle', 'cup', 'laptop'])

        # 设备配置
        if torch.cuda.is_available():
            self.device = 'cuda'
            print(f"[YOLO-Inference] CUDA available, using GPU")
        else:
            self.device = 'cpu'
            print("WARNING: CUDA not available, using CPU")

        print(f"[YOLO-Inference] Initializing on device: {self.device}")

        # 模型路径
        if not os.path.isabs(model_dir):
            # 相对路径，基于 config 文件位置
            config_dir = os.path.dirname(config_path)
            model_dir = os.path.join(config_dir, model_dir)

        model_path = os.path.join(model_dir, model_name)
        print(f"[YOLO-Inference] Loading model from: {model_path}")

        # 直接使用 ultralytics YOLO 类
        from ultralytics import YOLO

        # 加载 YOLO-E 模型
        self.model = YOLO(model_path)

        # 移动到 GPU
        if self.device == 'cuda':
            self.model.to(self.device)

        # 检测参数 (从配置读取)
        self.conf_threshold = self.config.get(
            'detection', {}).get(
            'conf_threshold', 0.7)
        self.iou_threshold = self.config.get(
            'detection', {}).get(
            'iou_threshold', 0.5)

        # 额外的过滤参数
        max_area_ratio = self.config.get(
            'detection', {}).get(
            'max_area_ratio', 0.25)
        self.max_area_ratio = max_area_ratio
        image_area = 1280 * 720  # 假设图像尺寸
        self.max_bbox_area = image_area * max_area_ratio

        print(f"[YOLO-Inference] Model loaded successfully on {self.device}")
        self.processing = False

    def run(self):
        print("[YOLO-Inference] Waiting for image data...")
        for message in self.pubsub.listen():
            if message['type'] == 'message':
                try:
                    # 反序列化图像数据
                    data = pickle.loads(message['data'])
                    image = data['image']  # numpy array (H, W, C) BGR
                    timestamp = data['timestamp']
                    frame_id = data['frame_id']
                    sequence_id = data.get('sequence_id', 0)

                    if self.processing:
                        print(
                            f"[YOLO-Inference] Skipping frame {frame_id}, still processing previous")
                        continue
                    self.processing = True

                    # GPU 推理
                    start_time = time.time()

                    # YOLO-E 推理
                    results = self.model.predict(
                        image,
                        conf=self.conf_threshold,
                        iou=self.iou_threshold,
                        verbose=False
                    )

                    inference_time = time.time() - start_time

                    # 解析结果
                    detections = self._parse_results(
                        results[0], image.shape[1], image.shape[0])

                    # 额外的后处理过滤
                    detections = self._filter_detections(
                        detections, image.shape[1], image.shape[0])

                    print(
                        f"[YOLO-Inference] Frame {frame_id}: {len(detections)} detections, {inference_time*1000:.1f}ms")

                    # 序列化结果
                    result_data = {
                        'detections': detections,
                        'timestamp': timestamp,
                        'frame_id': frame_id,
                        'sequence_id': sequence_id,
                        'inference_time': inference_time,
                        'device': self.device,
                        'image_width': image.shape[1],
                        'image_height': image.shape[0],
                    }

                    # 发布结果回 Redis
                    self.redis_client.publish(
                        'perception_results',
                        pickle.dumps(result_data)
                    )

                    self.processing = False

                except Exception as e:
                    print(f"[YOLO-Inference] Error: {e}")
                    import traceback
                    traceback.print_exc()
                    self.processing = False

    def _parse_results(self, result, image_width: int, image_height: int):
        """解析 ultralytics 结果."""
        serialized = []

        if result.boxes is None or len(result.boxes) == 0:
            return serialized

        boxes = result.boxes.xyxy.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy().astype(int)

        # 获取类别名称
        if hasattr(result, 'names') and result.names:
            class_names = result.names
        else:
            # 默认名称
            class_names = {i: f'class_{i}' for i in range(100)}

        # 掩码处理
        masks = result.masks

        for i in range(len(boxes)):
            x1, y1, x2, y2 = boxes[i].astype(int)

            # 序列化 mask
            mask_data = None
            mask_shape = None
            if masks is not None and i < len(masks.data):
                mask = masks.data[i].cpu().numpy()
                mask_data = mask.tobytes()
                mask_shape = mask.shape

            serialized.append({
                'class_id': int(class_ids[i]),
                'class_name': class_names.get(int(class_ids[i]), f'class_{int(class_ids[i])}'),
                'confidence': float(confidences[i]),
                'bbox': (int(x1), int(y1), int(x2), int(y2)),
                'track_id': None,
                'mask': mask_data,
                'mask_shape': mask_shape,
            })

        return serialized

    def _filter_detections(
            self,
            detections,
            image_width: int,
            image_height: int):
        """过滤检测结果，去除过大或过小的bbox."""
        if not detections:
            return detections

        image_area = image_width * image_height
        filtered = []

        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            # 计算 bbox 面积
            bbox_area = (x2 - x1) * (y2 - y1)

            # 过滤条件:
            # 1. 面积不超过最大比例
            # 2. 宽高比合理 (0.1 < ratio < 10)
            # 3. 最小面积 > 100像素
            area_ratio = bbox_area / image_area if image_area > 0 else 0
            bbox_w = x2 - x1
            bbox_h = y2 - y1
            aspect_ratio = bbox_w / bbox_h if bbox_h > 0 else 0

            if (area_ratio <= self.max_area_ratio and
                bbox_area > 100 and
                    0.1 < aspect_ratio < 10):
                filtered.append(det)

        return filtered


def main():
    import argparse
    parser = argparse.ArgumentParser(description='YOLO-E GPU Inference Node')
    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='Config file path')
    args = parser.parse_args()

    node = YoloInferenceNode(config_path=args.config)
    node.run()


if __name__ == '__main__':
    main()
