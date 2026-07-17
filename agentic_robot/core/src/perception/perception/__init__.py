"""
G1 Perception Node - Main Entry Point.

Integrates YOLO-E 2D detection with 3D point cloud detection.
Uses ROS2 message filters for synchronized input from ZED camera.

GPU 加速版本: 通过 Redis 中转将图像发送到 Python 3.8 GPU 环境进行推理，
然后接收结果并通过原有 ROS2 接口发布。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import yaml
import os
import time
import threading
from typing import Optional, Dict, Any, List

from perception.detectors.yoloe_detector import Yoloe2DDetector, YoloePromptMode, ImageDetections2D, Detection2DResult
from perception.modules.detection_2d import Detection2DModule
from perception.modules.detection_3d import Detection3DModule
from perception.redis_bridge import get_redis_bridge, DetectionResult as RedisDetectionResult


class G1PerceptionNode(Node):
    """
    G1 Perception Node for YOLO-E based open-vocabulary 3D detection.

    Subscribes to ZED2i topics:
    - /zed/zed_node/rgb/color/rect/image (RGB image)
    - /zed/zed_node/rgb/color/rect/image/camera_info (RGB camera info)
    - /zed/zed_node/depth/depth_registered (Depth image)
    - /zed/zed_node/depth/depth_registered/camera_info (Depth camera info)
    - /zed/zed_node/pose (World pose)

    Publishes:
    - /perception/detections_2d/image (2D detection visualization)
    - /perception/detections_3d/pointcloud (3D point cloud visualization)

    GPU 加速模式: 通过 Redis 中转将图像发送到 Python 3.8 环境进行 GPU 推理。
    """

    def __init__(self, config_path: Optional[str] = None) -> None:
        """Initialize G1 Perception Node."""
        super().__init__('perception_node')

        # Load configuration
        self.config = self._load_config(config_path)

        # Check GPU bridge mode
        self.use_gpu_bridge = self.config.get('use_gpu_bridge', False)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Store latest data
        self.latest_rgb_image: Optional[Image] = None
        self.latest_rgb_camera_info: Optional[CameraInfo] = None
        self.latest_depth_image: Optional[Image] = None
        self.latest_depth_camera_info: Optional[CameraInfo] = None
        self.latest_pose: Optional[PoseStamped] = None
        self.latest_rgb_image_width: int = 0
        self.latest_rgb_image_height: int = 0

        # 序列号管理
        self.latest_sequence: int = 0
        self.pending_frames: Dict[int, Dict] = {}

        if self.use_gpu_bridge:
            # GPU 桥接模式: 使用 Redis 中转
            self.get_logger().info(
                '[GPU Bridge] Enabled - will use Redis for GPU inference')
            self.redis_bridge = get_redis_bridge()
            self.redis_bridge.start_listening(self._on_gpu_result)

            # 初始化 3D 模块 (2D 模块不需要 YOLO-E detector，因为 GPU 节点做)
            self.detection_2d = None
            self.detection_3d = Detection3DModule(self, self.config)

            # 创建 2D 可视化发布者 (不创建 Detection2DModule，自己发布)
            topics_config = self.config.get('topics', {})
            self.det_2d_pub = self.create_publisher(Image, topics_config.get(
                'detections_2d_image', '/perception/detections_2d/image'), 10)
        else:
            # 普通模式: 直接在本地运行 YOLO-E
            self.get_logger().info(
                '[GPU Bridge] Disabled - using local CPU inference')
            self.detection_2d = Detection2DModule(self, self.config)
            self.detection_3d = Detection3DModule(self, self.config)

        # Setup subscribers
        self._setup_subscribers()

        # Setup publishers (if not already created)
        if self.use_gpu_bridge:
            self._setup_publishers()

        # Log initialization
        self.get_logger().info('G1 Perception Node initialized')
        self.get_logger().info(
            f"Prompt mode: {self.config.get('model', {}).get('prompt_mode', 'lrpc')}")
        self.get_logger().info(f"Device: {self.config.get('device', 'auto')}")
        self.get_logger().info(
            f"GPU Bridge: {'enabled' if self.use_gpu_bridge else 'disabled'}")

    def _load_config(
            self, config_path: Optional[str] = None) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        if config_path is None:
            # Try default locations
            possible_paths = [
                os.path.join(
                    os.path.dirname(__file__),
                    '../config/config.yaml'),
                'config/config.yaml',
            ]
            for path in possible_paths:
                if os.path.exists(path):
                    config_path = path
                    break

        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f'Loaded config from {config_path}')
            return config

        # Return default configuration
        self.get_logger().warn('No config file found, using defaults')
        return self._default_config()

    def _default_config(self) -> Dict[str, Any]:
        """Return default configuration."""
        return {
            'model': {
                'model_dir': 'models_yoloe',
                'model_name_lrpc': 'yoloe-11l-seg-pf.pt',
                'model_name_prompt': 'yoloe-11l-seg.pt',
                'prompt_mode': 'lrpc',
                'text_prompts': ['person', 'chair', 'table', 'bottle', 'cup', 'laptop'],
            },
            'device': 'cuda',
            'use_gpu_bridge': True,  # 默认启用 GPU 桥接
            'detection': {
                'conf_threshold': 0.6,
                'iou_threshold': 0.6,
                'max_area_ratio': 0.3,
                'exclude_class_ids': [],
            },
            'topics': {
                'rgb_image': '/zed/zed_node/rgb/color/rect/image',
                'rgb_camera_info': '/zed/zed_node/rgb/color/rect/image/camera_info',
                'depth_image': '/zed/zed_node/depth/depth_registered',
                'depth_camera_info': '/zed/zed_node/depth/depth_registered/camera_info',
                'pose': '/zed/zed_node/pose',
                'detections_2d_image': '/perception/detections_2d/image',
                'detections_3d_pointcloud': '/perception/detections_3d/pointcloud',
            },
            'sync': {
                'queue_size': 10,
                'slop_seconds': 0.1,
            },
            'visualization': {
                'draw_bboxes': True,
                'draw_labels': True,
                'draw_confidence': True,
                'pointcloud_color': 'class',
            },
            'detection_3d': {
                'min_points': 10,
                'outlier_threshold': 3.0,
                'max_range': 10.0,
            },
        }

    def _setup_subscribers(self) -> None:
        """Setup synchronized subscribers."""
        topics_config = self.config.get('topics', {})
        sync_config = self.config.get('sync', {})

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # RGB subscribers with synchronization
        self.rgb_sub = Subscriber(
            self,
            Image,
            topics_config.get(
                'rgb_image',
                '/zed/zed_node/rgb/color/rect/image'),
            qos_profile=qos)
        self.rgb_info_sub = Subscriber(
            self,
            CameraInfo,
            topics_config.get(
                'rgb_camera_info',
                '/zed/zed_node/rgb/color/rect/image/camera_info'),
            qos_profile=qos)

        # Time synchronizer for RGB
        self.rgb_sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.rgb_info_sub],
            queue_size=sync_config.get('queue_size', 10),
            slop=sync_config.get('slop_seconds', 0.1)
        )
        self.rgb_sync.registerCallback(self._rgb_callback)

        # Depth subscribers with synchronization
        self.depth_sub = Subscriber(
            self,
            Image,
            topics_config.get(
                'depth_image',
                '/zed/zed_node/depth/depth_registered'),
            qos_profile=qos)
        self.depth_info_sub = Subscriber(
            self,
            CameraInfo,
            topics_config.get(
                'depth_camera_info',
                '/zed/zed_node/depth/depth_registered/camera_info'),
            qos_profile=qos)

        # Time synchronizer for depth
        self.depth_sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.depth_info_sub],
            queue_size=sync_config.get('queue_size', 10),
            slop=sync_config.get('slop_seconds', 0.1)
        )
        self.depth_sync.registerCallback(self._depth_callback)

        # Pose subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped,
            topics_config.get('pose', '/zed/zed_node/pose'),
            self._pose_callback,
            qos
        )

        self.get_logger().info('Subscribers initialized')

    def _setup_publishers(self) -> None:
        """Setup publishers for GPU bridge mode."""
        topics_config = self.config.get('topics', {})

        # 3D detection point cloud (2D publisher already created in __init__)
        self.det_3d_pub = self.create_publisher(PointCloud2, topics_config.get(
            'detections_3d_pointcloud', '/perception/detections_3d/pointcloud'), 10)

        self.get_logger().info('Publishers initialized')

    def _rgb_callback(
            self,
            image_msg: Image,
            camera_info_msg: CameraInfo) -> None:
        """Handle synchronized RGB image and camera info."""
        try:
            self.latest_rgb_image = image_msg
            self.latest_rgb_camera_info = camera_info_msg

            # 转换图像为 numpy
            rgb_np = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding='bgr8')
            self.latest_rgb_image_width = rgb_np.shape[1]
            self.latest_rgb_image_height = rgb_np.shape[0]

            if self.use_gpu_bridge:
                # GPU 模式: 发送图像到 Redis
                self.latest_sequence += 1
                sequence_id = self.latest_sequence
                frame_id = f"{image_msg.header.stamp.sec}.{image_msg.header.stamp.nanosec}"

                # 存储待匹配的数据
                self.pending_frames[sequence_id] = {
                    'image': rgb_np,
                    'camera_info': camera_info_msg,
                    'image_msg': image_msg,
                    'timestamp': time.time(),
                    'frame_id': frame_id,
                }

                # 发送到 GPU 节点
                self.redis_bridge.publish_image(rgb_np, frame_id, sequence_id)

            else:
                # 普通模式: 直接在本地处理
                self.detection_2d._callback(image_msg, camera_info_msg)

                # Trigger 3D detection
                if self.detection_2d.latest_detections is not None:
                    if self.detection_3d.latest_depth is None or self.detection_3d.camera_matrix is None:
                        self.get_logger().warn('Depth not available for 3D processing, skipping')
                    else:
                        frame_id = self.config.get('topics', {}).get(
                            'camera_frame', 'zed_left_camera_frame')
                        detections_3d = self.detection_3d.process_detections(
                            self.detection_2d.latest_detections
                        )
                        if detections_3d:
                            if self.latest_depth_image is not None:
                                self.detection_3d.publish_pointcloud(
                                    detections_3d, frame_id=frame_id)

        except Exception as e:
            import traceback
            self.get_logger().error(
                f'RGB callback error: {e}\n{traceback.format_exc()}')

    def _depth_callback(
            self,
            depth_msg: Image,
            camera_info_msg: CameraInfo) -> None:
        """Handle synchronized depth image and camera info."""
        try:
            self.latest_depth_image = depth_msg
            self.latest_depth_camera_info = camera_info_msg

            # 转发给 3D 模块
            if self.use_gpu_bridge:
                self.detection_3d._depth_callback(depth_msg, camera_info_msg)
            else:
                self.detection_3d._depth_callback(depth_msg, camera_info_msg)

        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    def _pose_callback(self, pose_msg: PoseStamped) -> None:
        """Handle pose updates."""
        self.latest_pose = pose_msg

    def _on_gpu_result(
            self,
            detections: List[RedisDetectionResult],
            metadata: Dict):
        """处理 GPU 节点返回的推理结果."""
        try:
            sequence_id = metadata.get('sequence_id', 0)
            frame_id = metadata.get('frame_id', '')
            inference_time = metadata.get('inference_time', 0)
            device = metadata.get('device', 'cpu')
            image_width = metadata.get('image_width', 0)
            image_height = metadata.get('image_height', 0)

            self.get_logger().info(
                f'[GPU] Frame {frame_id}: {len(detections)} detections, '
                f'{inference_time*1000:.1f}ms, device={device}'
            )

            # 检查是否有对应的待处理帧
            if sequence_id not in self.pending_frames:
                self.get_logger().warn(
                    f'[GPU] No pending frame for sequence {sequence_id}')
                return

            pending = self.pending_frames.pop(sequence_id)

            # 转换为原有格式
            image_dets = ImageDetections2D(
                detections=[
                    Detection2DResult(
                        class_id=d.class_id,
                        class_name=d.class_name,
                        confidence=d.confidence,
                        bbox=d.bbox,
                        track_id=d.track_id,
                        mask=d.mask
                    )
                    for d in detections
                ],
                image_width=image_width,
                image_height=image_height,
                timestamp=metadata.get('timestamp', 0)
            )

            # 发布 2D 可视化
            vis_image = self._visualize(pending['image'], image_dets)
            self._publish_2d_visualization(
                vis_image, pending['image_msg'].header)

            # 处理 3D 点云
            if self.detection_3d.latest_depth is not None and self.detection_3d.camera_matrix is not None:
                detections_3d = self.detection_3d.process_detections(
                    image_dets)
                if detections_3d:
                    frame_id_3d = self.config.get('topics', {}).get(
                        'camera_frame', 'zed_left_camera_frame')
                    self.detection_3d.publish_pointcloud(
                        detections_3d, frame_id=frame_id_3d)
                    self.get_logger().info(
                        f'Published 3D pointcloud with {len(detections_3d)} detections')

        except Exception as e:
            import traceback
            self.get_logger().error(
                f'[GPU Bridge] Error processing result: {e}\n{traceback.format_exc()}')

    def _visualize(self, image, detections: ImageDetections2D):
        """可视化检测结果."""
        import cv2
        viz_config = self.config.get('visualization', {})
        draw_bboxes = viz_config.get('draw_bboxes', True)
        draw_labels = viz_config.get('draw_labels', True)
        draw_confidence = viz_config.get('draw_confidence', True)

        # 颜色映射
        colors = {
            'person': (255, 0, 0),
            'chair': (0, 255, 0),
            'table': (0, 0, 255),
            'bottle': (255, 255, 0),
            'cup': (255, 0, 255),
            'laptop': (0, 255, 255),
            'phone': (128, 0, 128),
            'book': (128, 128, 0),
            'bag': (0, 128, 128),
            'box': (128, 128, 128),
        }

        vis_image = image.copy()

        for det in detections.detections:
            x_min, y_min, x_max, y_max = det.bbox
            color = colors.get(det.class_name, (0, 255, 0))

            if draw_bboxes:
                cv2.rectangle(vis_image, (x_min, y_min),
                              (x_max, y_max), color, 2)

            if draw_labels or draw_confidence:
                label_parts = []
                if draw_labels:
                    label_parts.append(det.class_name)
                if draw_confidence:
                    label_parts.append(f'{det.confidence:.2f}')
                label = ' '.join(label_parts)

                if draw_labels:
                    (label_w, label_h), baseline = cv2.getTextSize(
                        label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1
                    )
                    cv2.rectangle(
                        vis_image,
                        (x_min, y_min - label_h - 12),
                        (x_min + label_w + 6, y_min),
                        (0, 0, 0),
                        -1
                    )
                    cv2.putText(
                        vis_image, label, (x_min + 3, y_min - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
                    )

        return vis_image

    def _publish_2d_visualization(self, image, header):
        """发布 2D 可视化图像."""
        from builtin_interfaces.msg import Time

        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        ros_image.header.stamp = header.stamp
        ros_image.header.frame_id = header.frame_id

        self.det_2d_pub.publish(ros_image)

    def get_camera_info(self) -> Optional[CameraInfo]:
        """Get latest RGB camera info."""
        return self.latest_rgb_camera_info

    def get_depth_camera_info(self) -> Optional[CameraInfo]:
        """Get latest depth camera info."""
        return self.latest_depth_camera_info

    def get_pose(self) -> Optional[PoseStamped]:
        """Get latest world pose."""
        return self.latest_pose

    def stop(self) -> None:
        """Stop and cleanup."""
        if self.use_gpu_bridge:
            if self.redis_bridge:
                self.redis_bridge.stop()
        else:
            if self.detection_2d:
                self.detection_2d.stop()

        if self.detection_3d:
            pass  # Detection3DModule 没有 stop 方法

        self.get_logger().info('G1 Perception Node stopped')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    # Check for config path argument
    config_path = None
    if args and len(args) > 1:
        for i, arg in enumerate(args):
            if arg == '--config' and i + 1 < len(args):
                config_path = args[i + 1]

    node = G1PerceptionNode(config_path=config_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
