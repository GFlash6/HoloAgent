"""
Redis Bridge - 连接 ROS2 节点和 GPU 推理节点
"""

import redis
import pickle
import numpy as np
import threading
import time
from typing import Optional, Callable, Dict, Any, List
from dataclasses import dataclass


@dataclass
class DetectionResult:
    """简化的检测结果数据结构."""

    class_id: int
    class_name: str
    confidence: float
    bbox: tuple  # (x_min, y_min, x_max, y_max)
    track_id: Optional[int] = None
    mask: Optional[np.ndarray] = None


class RedisBridge:
    """Redis 中转桥接器."""

    IMAGE_CHANNEL = 'perception_image'
    RESULT_CHANNEL = 'perception_results'

    def __init__(self, host: str = 'localhost', port: int = 6379):
        self.redis_client = redis.Redis(
            host=host,
            port=port,
            db=0,
            decode_responses=False
        )
        self.pubsub = None
        self.result_thread = None
        self.callback: Optional[Callable] = None
        self._running = False

    def start_listening(
            self, callback: Callable[[List[DetectionResult], Dict], None]):
        """
        启动监听 GPU 推理结果.

        Args:
            callback: 回调函数，接收 (detections: List[DetectionResult], metadata: Dict)
        """
        self.callback = callback
        self.pubsub = self.redis_client.pubsub()
        self.pubsub.subscribe(self.RESULT_CHANNEL)
        self._running = True

        self.result_thread = threading.Thread(
            target=self._listen_results, daemon=True)
        self.result_thread.start()
        print("[RedisBridge] Started listening for results")

    def _listen_results(self):
        """监听推理结果."""
        for message in self.pubsub.listen():
            if not self._running:
                break
            if message['type'] == 'message' and self.callback:
                try:
                    data = pickle.loads(message['data'])
                    detections = self._deserialize_detections(
                        data['detections'])
                    metadata = {
                        'timestamp': data.get(
                            'timestamp', 0), 'frame_id': data.get(
                            'frame_id', 0), 'sequence_id': data.get(
                            'sequence_id', 0), 'inference_time': data.get(
                            'inference_time', 0), 'device': data.get(
                            'device', 'cpu'), 'image_width': data.get(
                            'image_width', 0), 'image_height': data.get(
                                'image_height', 0), 'timestamp_from_result': data.get(
                                    'timestamp_from_result', None), }
                    self.callback(detections, metadata)
                except Exception as e:
                    print(f"[RedisBridge] Error processing result: {e}")

    def publish_image(
            self,
            image: np.ndarray,
            frame_id: str,
            sequence_id: int = 0) -> bool:
        """
        发布图像数据到 GPU 推理节点.

        Args:
            image: numpy array (H, W, C)
            frame_id: 帧 ID
            sequence_id: 序列号

        Returns:
            bool: 是否成功发布
        """
        try:
            data = {
                'image': image,
                'timestamp': time.time(),
                'frame_id': frame_id,
                'sequence_id': sequence_id
            }
            self.redis_client.publish(self.IMAGE_CHANNEL, pickle.dumps(data))
            return True
        except Exception as e:
            print(f"[RedisBridge] Error publishing image: {e}")
            return False

    def _deserialize_detections(
            self, serialized: List[Dict]) -> List[DetectionResult]:
        """反序列化检测结果."""
        detections = []
        for d in serialized:
            mask = None
            if d.get('mask') is not None and d.get('mask_shape') is not None:
                mask = np.frombuffer(
                    d['mask'], dtype=np.uint8).reshape(
                    d['mask_shape'])

            det = DetectionResult(
                class_id=d['class_id'],
                class_name=d['class_name'],
                confidence=d['confidence'],
                bbox=tuple(d['bbox']),
                track_id=d.get('track_id'),
                mask=mask
            )
            detections.append(det)
        return detections

    def stop(self):
        """停止监听."""
        self._running = False
        if self.pubsub:
            self.pubsub.close()
        print("[RedisBridge] Stopped")


# 全局单例
_redis_bridge: Optional[RedisBridge] = None


def get_redis_bridge() -> RedisBridge:
    """获取 Redis 桥接器单例."""
    global _redis_bridge
    if _redis_bridge is None:
        _redis_bridge = RedisBridge()
    return _redis_bridge
