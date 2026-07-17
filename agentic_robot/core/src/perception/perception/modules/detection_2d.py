"""
2D Detection Module for G1 Perception.

Reference: dimos/dimos/perception/detection/module2D.py Converts YOLO-E
detections to ROS2 messages.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge

import cv2
import numpy as np
from typing import Optional, Dict, Any

from perception.detectors.yoloe_detector import (
    Yoloe2DDetector,
    YoloePromptMode,
    ImageDetections2D,
    Detection2DResult,
)


# Class color map for visualization - person: red, others: blue
CLASS_COLORS: Dict[str, tuple[int, int, int]] = {
    'person': (255, 0, 0),        # Red
    'person.': (255, 0, 0),       # Red
    'Person': (255, 0, 0),        # Red
    'chair': (0, 100, 255),       # Blue
    'Chair': (0, 100, 255),       # Blue
    'table': (0, 100, 255),       # Blue
    'Table': (0, 100, 255),       # Blue
    'bottle': (0, 100, 255),      # Blue
    'Bottle': (0, 100, 255),      # Blue
    'cup': (0, 100, 255),         # Blue
    'Cup': (0, 100, 255),         # Blue
    'laptop': (0, 100, 255),      # Blue
    'Laptop': (0, 100, 255),      # Blue
    'phone': (0, 100, 255),       # Blue
    'Phone': (0, 100, 255),       # Blue
    'book': (0, 100, 255),        # Blue
    'Book': (0, 100, 255),        # Blue
    'bag': (0, 100, 255),         # Blue
    'Bag': (0, 100, 255),         # Blue
    'box': (0, 100, 255),         # Blue
    'Box': (0, 100, 255),         # Blue
    'monitor': (0, 100, 255),     # Blue
    'Monitor': (0, 100, 255),     # Blue
    'keyboard': (0, 100, 255),    # Blue
    'Keyboard': (0, 100, 255),    # Blue
    'mouse': (0, 100, 255),       # Blue
    'Mouse': (0, 100, 255),       # Blue
}

# Default color for unknown classes
DEFAULT_COLOR = (0, 100, 255)  # Blue


def get_color(class_name: str) -> tuple[int, int, int]:
    """Get color for class name."""
    return CLASS_COLORS.get(class_name, DEFAULT_COLOR)


def get_random_color(seed: int) -> tuple[int, int, int]:
    """Get a deterministic random color based on seed."""
    np.random.seed(seed)
    return tuple(np.random.randint(0, 255, 3).tolist())


class Detection2DModule:
    """
    2D Detection Module using YOLO-E.

    Subscribes to synchronized RGB image and camera info, runs YOLO-E
    detection, and publishes visualization results.
    """

    def __init__(self, node: Node, config: Dict[str, Any]) -> None:
        """
        Initialize 2D detection module.

        Args:
            node: ROS2 node instance.
            config: Configuration dictionary loaded from YAML.
        """
        self.node = node
        self.config = config

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Parse configuration
        model_config = config.get('model', {})
        topics_config = config.get('topics', {})
        sync_config = config.get('sync', {})
        viz_config = config.get('visualization', {})

        # Determine prompt mode
        prompt_mode_str = model_config.get('prompt_mode', 'lrpc')
        prompt_mode = YoloePromptMode.LRPC if prompt_mode_str == 'lrpc' else YoloePromptMode.PROMPT

        # Initialize detector
        self.detector = Yoloe2DDetector(
            model_dir=model_config.get(
                'model_dir',
                'models_yoloe'),
            model_name_lrpc=model_config.get(
                'model_name_lrpc',
                'yoloe-11l-seg-pf.pt'),
            model_name_prompt=model_config.get(
                'model_name_prompt',
                'yoloe-11l-seg.pt'),
            device=config.get(
                'device',
                'auto'),
            prompt_mode=prompt_mode,
            exclude_class_ids=config.get(
                'detection',
                {}).get(
                'exclude_class_ids',
                []),
            max_area_ratio=config.get(
                'detection',
                {}).get(
                'max_area_ratio',
                0.3),
            text_prompts=model_config.get(
                'text_prompts',
                []),
            conf_threshold=config.get(
                'detection',
                {}).get(
                'conf_threshold',
                0.6),
            iou_threshold=config.get(
                'detection',
                {}).get(
                'iou_threshold',
                0.6),
        )

        self.draw_bboxes = viz_config.get('draw_bboxes', True)
        self.draw_labels = viz_config.get('draw_labels', True)
        self.draw_confidence = viz_config.get('draw_confidence', True)

        # Camera info storage
        self.camera_info: Optional[CameraInfo] = None
        # Latest 2D detection result (shared with 3D module)
        self.latest_detections: Optional[ImageDetections2D] = None

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers with time synchronization
        self.image_sub = Subscriber(
            node,
            Image,
            topics_config.get(
                'rgb_image',
                '/zed/zed_node/rgb/color/rect/image'),
            qos_profile=qos)
        self.camera_info_sub = Subscriber(
            node,
            CameraInfo,
            topics_config.get(
                'rgb_camera_info',
                '/zed/zed_node/rgb/color/rect/image/camera_info'),
            qos_profile=qos)

        # Time synchronizer
        self.time_sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub],
            queue_size=sync_config.get('queue_size', 10),
            slop=sync_config.get('slop_seconds', 0.1)
        )
        self.time_sync.registerCallback(self._callback)

        # Publishers
        self.image_pub = node.create_publisher(Image, topics_config.get(
            'detections_2d_image', '/perception/detections_2d/image'), 10)

        self.node.get_logger().info('Detection2DModule initialized')

    def _callback(self, image_msg: Image, camera_info_msg: CameraInfo) -> None:
        """Handle synchronized image and camera info."""
        try:
            # Store camera info
            self.camera_info = camera_info_msg

            # Convert image - handle header issues gracefully
            try:
                cv_image = self.bridge.imgmsg_to_cv2(
                    image_msg, desired_encoding='bgr8')
            except Exception as cv_err:
                self.node.get_logger().warn(
                    f'CV bridge error: {cv_err}, trying mono8')
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(
                        image_msg, desired_encoding='mono8')
                except BaseException:
                    return

            # Run detection
            detections = self.detector.process_image(
                image=cv_image,
                image_width=cv_image.shape[1],
                image_height=cv_image.shape[0],
            )

            if not detections.detections:
                return

            # Store latest detections for 3D module
            self.latest_detections = detections

            # Visualize and publish
            vis_image = self._visualize(cv_image, detections)
            self._publish_visualization(vis_image, image_msg.header)

        except Exception as e:
            import traceback
            self.node.get_logger().error(
                f'Detection2D callback error: {e}\n{traceback.format_exc()}')

    def _visualize(
        self,
        image: np.ndarray,
        detections: ImageDetections2D
    ) -> np.ndarray:
        """Visualize detections on image."""
        vis_image = image.copy()

        for det in detections.detections:
            x_min, y_min, x_max, y_max = det.bbox
            # Get color: person=red, others=blue
            color = get_color(det.class_name)

            if self.draw_bboxes:
                cv2.rectangle(vis_image, (x_min, y_min),
                              (x_max, y_max), color, 2)

            # Build label text
            if self.draw_labels or self.draw_confidence:
                label_parts = []
                if self.draw_labels:
                    label_parts.append(det.class_name)
                if self.draw_confidence:
                    label_parts.append(f'{det.confidence:.2f}')
                if det.track_id is not None:
                    label_parts.insert(0, f'ID:{det.track_id}')
                label = ' '.join(label_parts)

                # Draw label background (black background + white bold text)
                if self.draw_labels:
                    (label_w, label_h), baseline = cv2.getTextSize(
                        label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1
                    )
                    cv2.rectangle(
                        vis_image,
                        (x_min, y_min - label_h - 12),
                        (x_min + label_w + 6, y_min),
                        (0, 0, 0),  # Black background
                        -1
                    )
                    cv2.putText(
                        vis_image, label, (x_min + 3, y_min - 5),
                        # White text, bold (thickness=2)
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
                    )

        return vis_image

    def _publish_visualization(
            self,
            image: np.ndarray,
            header: Any = None) -> None:
        """Publish visualization image."""
        from builtin_interfaces.msg import Time
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        # Use current time for stamp
        now = self.node.get_clock().now().to_msg()
        ros_image.header.stamp = now
        # Use camera frame from config (should be zed_left_camera_frame)
        frame_id = self.node.config.get(
            'topics', {}).get(
            'camera_frame', 'zed_left_camera_frame')
        ros_image.header.frame_id = frame_id
        self.image_pub.publish(ros_image)

    def get_camera_info(self) -> Optional[CameraInfo]:
        """Get latest camera info."""
        return self.camera_info

    def stop(self) -> None:
        """Stop and cleanup."""
        self.detector.stop()
