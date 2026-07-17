"""
3D Detection Module for G1 Perception.

Converts 2D detections + depth image to 3D point cloud detections. Projects
point clouds within 2D bounding boxes to create object-centric point clouds.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
from numpy.typing import NDArray
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass

from perception.detectors.yoloe_detector import ImageDetections2D, Detection2DResult


@dataclass
class Detection3DResult:
    """3D detection result with point cloud."""

    detection_2d: Detection2DResult
    pointcloud: NDArray[np.float32]  # (N, 3) - xyz points
    center: NDArray[np.float32]      # (3,) - centroid xyz
    min_corner: NDArray[np.float32]  # (3,) - bounding box min
    max_corner: NDArray[np.float32]  # (3,) - bounding box max
    color: Tuple[int, int, int]      # RGB color for visualization


class Detection3DModule:
    """
    3D Detection Module.

    Takes 2D detections and depth image, projects to 3D point clouds. Publishes
    colored point cloud visualization of detections.
    """

    def __init__(self, node: Node, config: Dict[str, Any]) -> None:
        """
        Initialize 3D detection module.

        Args:
            node: ROS2 node instance.
            config: Configuration dictionary.
        """
        self.node = node
        self.config = config

        # Parse configuration
        topics_config = config.get('topics', {})
        sync_config = config.get('sync', {})
        detection_3d_config = config.get('detection_3d', {})
        viz_config = config.get('visualization', {})

        # Get camera frame from config
        self.camera_frame = topics_config.get(
            'camera_frame', 'zed_left_camera_frame')

        # 3D detection parameters
        self.min_points = detection_3d_config.get('min_points', 10)
        self.outlier_threshold = detection_3d_config.get(
            'outlier_threshold', 3.0)
        self.max_range = detection_3d_config.get('max_range', 10.0)

        # Visualization settings
        self.pointcloud_color_mode = viz_config.get(
            'pointcloud_color', 'class')

        # Camera parameters (from camera info)
        self.camera_matrix: Optional[NDArray[np.float64]] = None
        self.image_width: int = 0
        self.image_height: int = 0

        # Latest depth image
        self.latest_depth: Optional[NDArray[np.float32]] = None
        self.latest_depth_camera_info: Optional[CameraInfo] = None

        # Latest pose
        self.latest_pose: Optional[PoseStamped] = None

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers for depth + camera info synchronization
        self.depth_sub = Subscriber(
            node,
            Image,
            topics_config.get(
                'depth_image',
                '/zed/zed_node/depth/depth_registered'),
            qos_profile=qos)
        self.depth_info_sub = Subscriber(
            node,
            CameraInfo,
            topics_config.get(
                'depth_camera_info',
                '/zed/zed_node/depth/depth_registered/camera_info'),
            qos_profile=qos)

        # Pose subscriber
        self.pose_sub = node.create_subscription(
            PoseStamped,
            topics_config.get('pose', '/zed/zed_node/pose'),
            self._pose_callback,
            qos
        )

        # Time synchronizer for depth
        self.depth_sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.depth_info_sub],
            queue_size=sync_config.get('queue_size', 10),
            slop=sync_config.get('slop_seconds', 0.1)
        )
        self.depth_sync.registerCallback(self._depth_callback)

        # Publisher for 3D point cloud
        self.pointcloud_pub = node.create_publisher(PointCloud2, topics_config.get(
            'detections_3d_pointcloud', '/perception/detections_3d/pointcloud'), 10)

        # Publisher for 3D bounding box markers with labels
        self.marker_pub = node.create_publisher(
            MarkerArray,
            '/perception/detections_3d/markers',
            10
        )

        # Class color map - person: red, others: blue
        # Also add variations to handle different class name formats
        self._class_colors: Dict[str, Tuple[int, int, int]] = {
            'person': (255, 0, 0),
            'person.': (255, 0, 0),
            'Person': (255, 0, 0),
            'chair': (0, 100, 255),
            'Chair': (0, 100, 255),
            'table': (0, 100, 255),
            'Table': (0, 100, 255),
            'bottle': (0, 100, 255),
            'Bottle': (0, 100, 255),
            'cup': (0, 100, 255),
            'Cup': (0, 100, 255),
            'laptop': (0, 100, 255),
            'Laptop': (0, 100, 255),
            'phone': (0, 100, 255),
            'Phone': (0, 100, 255),
            'book': (0, 100, 255),
            'Book': (0, 100, 255),
            'bag': (0, 100, 255),
            'Bag': (0, 100, 255),
            'box': (0, 100, 255),
            'Box': (0, 100, 255),
            'monitor': (0, 100, 255),
            'Monitor': (0, 100, 255),
            'keyboard': (0, 100, 255),
            'Keyboard': (0, 100, 255),
            'mouse': (0, 100, 255),
            'Mouse': (0, 100, 255),
        }
        # Default color for unknown classes
        self._default_color = (0, 100, 255)  # Blue

        self.node.get_logger().info('Detection3DModule initialized')

    def _pose_callback(self, pose_msg: PoseStamped) -> None:
        """Handle pose updates (world frame)."""
        self.latest_pose = pose_msg

    def _depth_callback(
            self,
            depth_msg: Image,
            camera_info_msg: CameraInfo) -> None:
        """Handle synchronized depth and camera info."""
        try:
            from cv_bridge import CvBridge
            bridge = CvBridge()

            # Convert depth image
            cv_depth = bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='32FC1')
            self.latest_depth = cv_depth.astype(np.float32)

            # Store camera info
            self.latest_depth_camera_info = camera_info_msg

            # Extract camera matrix
            self.camera_matrix = np.array(camera_info_msg.k).reshape(3, 3)
            self.image_width = camera_info_msg.width
            self.image_height = camera_info_msg.height

            self.node.get_logger().debug(
                f'Depth received: shape={cv_depth.shape}, camera_matrix set')

        except Exception as e:
            self.node.get_logger().error(f'Depth callback error: {e}')

    def set_camera_params(
        self,
        fx: float, fy: float, cx: float, cy: float,
        width: int, height: int
    ) -> None:
        """Manually set camera parameters if not from camera info."""
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float64)
        self.image_width = width
        self.image_height = height

    def process_detections(
        self,
        detections_2d: ImageDetections2D
    ) -> List[Detection3DResult]:
        """
        Process 2D detections with depth to create 3D detections.

        Uses YOLO-E segmentation mask to filter out background points.

        Args:
            detections_2d: 2D detection results.

        Returns:
            List of 3D detection results.
        """
        if self.camera_matrix is None or self.latest_depth is None:
            return []

        detections_3d: List[Detection3DResult] = []

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        for det_2d in detections_2d.detections:
            x_min, y_min, x_max, y_max = det_2d.bbox

            # Crop depth region
            depth_crop = self.latest_depth[y_min:y_max, x_min:x_max]

            if depth_crop.size == 0:
                continue

            # Get valid depth points within bbox
            h, w = depth_crop.shape
            y_coords, x_coords = np.where(
                (depth_crop > 0.1) & (depth_crop < self.max_range)
            )

            if len(y_coords) < self.min_points:
                continue

            # If segmentation mask available, use it to filter points
            if det_2d.mask is not None:
                # Crop mask to bounding box region
                mask_crop = det_2d.mask[y_min:y_max, x_min:x_max]
                if mask_crop.shape == (h, w):
                    # Keep only points inside the mask
                    mask_valid = mask_crop[y_coords, x_coords] > 0
                    x_coords = x_coords[mask_valid]
                    y_coords = y_coords[mask_valid]

            # Recalculate after mask filtering
            if len(y_coords) < self.min_points:
                continue

            depths = depth_crop[y_coords, x_coords]

            # Filter by depth validity
            valid_mask = depths > 0.1
            x_coords = x_coords[valid_mask]
            y_coords = y_coords[valid_mask]
            depths = depths[valid_mask]

            if len(depths) < self.min_points:
                continue

            # Project to 3D
            u = x_coords + x_min
            v = y_coords + y_min

            x = (u - cx) * depths / fx
            y = (v - cy) * depths / fy
            z = depths

            # Stack to point cloud (N, 3)
            points = np.column_stack([x, y, z]).astype(np.float32)

            # Filter outliers
            if len(points) > 10:
                points = self._filter_outliers(points)

            if len(points) < self.min_points:
                continue

            # Calculate statistics
            center = np.mean(points, axis=0)
            min_corner = np.min(points, axis=0)
            max_corner = np.max(points, axis=0)

            # Get color
            color = self._get_color(det_2d.class_name)

            detections_3d.append(Detection3DResult(
                detection_2d=det_2d,
                pointcloud=points,
                center=center,
                min_corner=min_corner,
                max_corner=max_corner,
                color=color,
            ))

        return detections_3d

    def _filter_outliers(self,
                         points: NDArray[np.float32]) -> NDArray[np.float32]:
        """Filter statistical outliers from point cloud."""
        if len(points) < 10:
            return points

        mean = np.mean(points, axis=0)
        distances = np.linalg.norm(points - mean, axis=1)
        std = np.std(distances)

        if std < 1e-6:
            return points

        threshold = self.outlier_threshold * std
        valid = distances < threshold

        return points[valid]

    def _get_color(self, class_name: str) -> Tuple[int, int, int]:
        """Get color for class."""
        return self._class_colors.get(class_name, self._default_color)

    def publish_pointcloud(
        self,
        detections_3d: List[Detection3DResult],
        frame_id: str = "zed_left_camera_frame_optical"
    ) -> None:
        """
        Publish 3D detections as colored point cloud.

        Args:
            detections_3d: List of 3D detection results.
            frame_id: Frame ID for the point cloud (default: zed_left_camera_frame_optical).
        """
        if not detections_3d:
            return

        import sensor_msgs_py.point_cloud2 as pc2
        from std_msgs.msg import Header

        # Collect all points with colors
        all_points = []
        for det_3d in detections_3d:
            r, g, b = det_3d.color
            for point in det_3d.pointcloud:
                all_points.append((
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                    int(r),
                    int(g),
                    int(b),
                    255  # alpha
                ))

        if not all_points:
            return

        # Create structured array
        points_array = np.array(all_points, dtype=[
            ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
            ('r', 'u1'), ('g', 'u1'), ('b', 'u1'), ('a', 'u1')
        ])

        # Create PointCloud2 message
        fields = [
            pc2.PointField(name='x', offset=0,
                           datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4,
                           datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8,
                           datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12,
                           datatype=pc2.PointField.UINT32, count=1),
        ]

        # Pack RGB into uint32
        packed_points = np.empty(
            len(points_array), dtype=[
                ('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'u4')])
        packed_points['x'] = points_array['x']
        packed_points['y'] = points_array['y']
        packed_points['z'] = points_array['z']
        packed_points['rgb'] = (
            (points_array['r'].astype('u4') << 16) |
            (points_array['g'].astype('u4') << 8) |
            points_array['b'].astype('u4')
        )

        # Create proper Header with stamp and frame_id
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = frame_id

        pc_msg = pc2.create_cloud(
            header=header,
            fields=fields,
            points=packed_points
        )

        self.pointcloud_pub.publish(pc_msg)
        self.node.get_logger().debug(
            f'Published pointcloud with {len(all_points)} points')

        # Publish 3D bounding boxes and labels
        self._publish_markers(detections_3d, frame_id)

    def _publish_markers(
        self,
        detections_3d: List[Detection3DResult],
        frame_id: str
    ) -> None:
        """Publish 3D bounding box markers and labels."""
        from builtin_interfaces.msg import Duration
        marker_array = MarkerArray()
        current_time = self.node.get_clock().now().to_msg()

        # Lifetime for markers (prevent flickering)
        lifetime = Duration()
        lifetime.sec = 1
        lifetime.nanosec = 0

        for idx, det_3d in enumerate(detections_3d):
            # Get bounding box corners
            min_c = det_3d.min_corner
            max_c = det_3d.max_corner
            class_name = det_3d.detection_2d.class_name

            # Get color
            r, g, b = det_3d.color
            color_r = float(r) / 255.0
            color_g = float(g) / 255.0
            color_b = float(b) / 255.0

            # === 1. Transparent solid box (CUBE) ===
            cube_marker = Marker()
            cube_marker.header.frame_id = frame_id
            cube_marker.header.stamp = current_time
            cube_marker.ns = 'bbox_solid'
            cube_marker.id = idx
            cube_marker.type = Marker.CUBE
            cube_marker.action = Marker.ADD
            cube_marker.lifetime = lifetime

            # Position at center of bbox
            center_x = (float(min_c[0]) + float(max_c[0])) / 2
            center_y = (float(min_c[1]) + float(max_c[1])) / 2
            center_z = (float(min_c[2]) + float(max_c[2])) / 2
            cube_marker.pose.position.x = center_x
            cube_marker.pose.position.y = center_y
            cube_marker.pose.position.z = center_z
            cube_marker.pose.orientation.w = 1.0

            # Size of the box
            size_x = float(max_c[0]) - float(min_c[0])
            size_y = float(max_c[1]) - float(min_c[1])
            size_z = float(max_c[2]) - float(min_c[2])
            cube_marker.scale.x = size_x
            cube_marker.scale.y = size_y
            cube_marker.scale.z = size_z

            # Transparent color (alpha = 0.3)
            cube_marker.color.r = color_r
            cube_marker.color.g = color_g
            cube_marker.color.b = color_b
            cube_marker.color.a = 0.3

            marker_array.markers.append(cube_marker)

            # === 2. Solid edges (LINE_STRIP) ===
            edge_marker = Marker()
            edge_marker.header.frame_id = frame_id
            edge_marker.header.stamp = current_time
            edge_marker.ns = 'bbox_edges'
            edge_marker.id = idx
            edge_marker.type = Marker.LINE_STRIP
            edge_marker.action = Marker.ADD
            edge_marker.lifetime = lifetime

            edge_marker.scale.x = 0.015  # Line width
            edge_marker.color.r = color_r
            edge_marker.color.g = color_g
            edge_marker.color.b = color_b
            edge_marker.color.a = 1.0  # Solid edges

            # 12 edges of the bounding box as closed loops
            # Bottom face (counter-clockwise)
            edge_marker.points.append(
                self._make_point(
                    min_c[0], min_c[1], min_c[2]))
            edge_marker.points.append(
                self._make_point(
                    max_c[0], min_c[1], min_c[2]))
            edge_marker.points.append(
                self._make_point(
                    max_c[0], max_c[1], min_c[2]))
            edge_marker.points.append(
                self._make_point(
                    min_c[0], max_c[1], min_c[2]))
            edge_marker.points.append(
                self._make_point(
                    min_c[0],
                    min_c[1],
                    min_c[2]))  # Close

            # Top face (counter-clockwise)
            edge_marker.points.append(
                self._make_point(
                    min_c[0], min_c[1], max_c[2]))
            edge_marker.points.append(
                self._make_point(
                    max_c[0], min_c[1], max_c[2]))
            edge_marker.points.append(
                self._make_point(
                    max_c[0], max_c[1], max_c[2]))
            edge_marker.points.append(
                self._make_point(
                    min_c[0], max_c[1], max_c[2]))
            edge_marker.points.append(
                self._make_point(
                    min_c[0],
                    min_c[1],
                    max_c[2]))  # Close

            # Close the bottom face again
            edge_marker.points.append(
                self._make_point(
                    min_c[0], min_c[1], min_c[2]))

            marker_array.markers.append(edge_marker)

            # === 3. Vertical edge markers (LINE_LIST for 4 vertical edges) ===
            vert_marker = Marker()
            vert_marker.header.frame_id = frame_id
            vert_marker.header.stamp = current_time
            vert_marker.ns = 'bbox_vertical'
            vert_marker.id = idx
            vert_marker.type = Marker.LINE_LIST
            vert_marker.action = Marker.ADD
            vert_marker.lifetime = lifetime

            vert_marker.scale.x = 0.015
            vert_marker.color.r = color_r
            vert_marker.color.g = color_g
            vert_marker.color.b = color_b
            vert_marker.color.a = 1.0

            # 4 vertical edges
            vert_marker.points.append(
                self._make_point(
                    min_c[0], min_c[1], min_c[2]))
            vert_marker.points.append(
                self._make_point(
                    min_c[0], min_c[1], max_c[2]))

            vert_marker.points.append(
                self._make_point(
                    max_c[0], min_c[1], min_c[2]))
            vert_marker.points.append(
                self._make_point(
                    max_c[0], min_c[1], max_c[2]))

            vert_marker.points.append(
                self._make_point(
                    max_c[0], max_c[1], min_c[2]))
            vert_marker.points.append(
                self._make_point(
                    max_c[0], max_c[1], max_c[2]))

            vert_marker.points.append(
                self._make_point(
                    min_c[0], max_c[1], min_c[2]))
            vert_marker.points.append(
                self._make_point(
                    min_c[0], max_c[1], max_c[2]))

            marker_array.markers.append(vert_marker)

            # === 4. Text label with black background - positioned to not be occluded ===
            # Position label to the side of the box (positive Y direction = left in camera view)
            # This prevents the label from being hidden by the box itself
            label_x = float(max_c[0]) + 0.1  # Right side of box
            label_y = float(min_c[1])         # Same Y as front
            label_z = float(max_c[2])         # Same height as top

            # First create a black background box for the label (CUBE)
            bg_marker = Marker()
            bg_marker.header.frame_id = frame_id
            bg_marker.header.stamp = current_time
            bg_marker.ns = 'label_bg'
            bg_marker.id = idx
            bg_marker.type = Marker.CUBE
            bg_marker.action = Marker.ADD
            bg_marker.lifetime = lifetime

            bg_marker.pose.position.x = label_x + 0.15
            bg_marker.pose.position.y = label_y
            bg_marker.pose.position.z = label_z
            bg_marker.pose.orientation.w = 1.0

            # Background size (fixed size)
            bg_marker.scale.x = 0.35
            bg_marker.scale.y = 0.12
            bg_marker.scale.z = 0.08

            # Black background
            bg_marker.color.r = 0.0
            bg_marker.color.g = 0.0
            bg_marker.color.b = 0.0
            bg_marker.color.a = 1.0

            marker_array.markers.append(bg_marker)

            # Then create the text label
            label_marker = Marker()
            label_marker.header.frame_id = frame_id
            label_marker.header.stamp = current_time
            label_marker.ns = 'labels'
            label_marker.id = idx
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.lifetime = lifetime

            label_marker.pose.position.x = label_x + 0.15
            label_marker.pose.position.y = label_y
            # Slightly in front to avoid z-fighting
            label_marker.pose.position.z = label_z + 0.001
            label_marker.pose.orientation.w = 1.0

            # Text scale
            label_marker.scale.x = 0.1
            label_marker.scale.y = 0.1
            label_marker.scale.z = 0.1

            # White text
            label_marker.color.r = 1.0
            label_marker.color.g = 1.0
            label_marker.color.b = 1.0
            label_marker.color.a = 1.0

            # Include confidence in label
            conf = det_3d.detection_2d.confidence
            label_marker.text = f"{class_name} {conf:.2f}"

            marker_array.markers.append(label_marker)

        self.marker_pub.publish(marker_array)
        self.node.get_logger().debug(
            f'Published {len(detections_3d)} 3D bounding boxes with labels')

    def _make_point(self, x: float, y: float, z: float):
        """Create a geometry_msgs Point."""
        from geometry_msgs.msg import Point
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def get_latest_pose(self) -> Optional[PoseStamped]:
        """Get latest world pose."""
        return self.latest_pose

    def get_latest_depth(self) -> Optional[NDArray[np.float32]]:
        """Get latest depth image."""
        return self.latest_depth
