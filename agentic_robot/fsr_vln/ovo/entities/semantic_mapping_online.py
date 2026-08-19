from __future__ import annotations
from typing import Any, Dict, Deque, Optional, Tuple
import os
import time
import gc
from pathlib import Path
from collections import deque
import threading  # 新增
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ..utils import vis_utils

import numpy as np
import torch
import torch.multiprocessing as mp
import open3d as o3d
import matplotlib.pyplot as plt
# ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

# ...existing code...
from .logger import Logger
from .obj_detect_track import ObjDetectTrack
from .visualizer import stream_pcd
from ..utils import io_utils
from ..slam.vanilla_mapper import VanillaMapper
# 如果你还有 gaussian/orbslam2，可按原来的 get_slam_backbone 逻辑合并


def get_slam_backbone(config: Dict[str, Any], cam_intrinsics: torch.Tensor):
    module = config["slam"].get("slam_module", "vanilla")
    if module == "gaussian_slam":
        from ..slam.gaussian_slam import WrapperGaussianSLAM
        return WrapperGaussianSLAM(config, None)
    elif module == "orbslam2":
        from ..slam.orbslam2 import WrapperORBSLAM2
        # 这里需要世界参考位姿，可按需要修改
        return WrapperORBSLAM2(config, cam_intrinsics, world_ref=torch.eye(4))
    elif module == "livwo":
        return VanillaMapper(config, cam_intrinsics)
    else:
        return VanillaMapper(config, cam_intrinsics)
# ...existing code...


class SemanticMapping(Node):  # 修复拼写错误
    """
    ROS2 在线语义建图：

    - 订阅 RGB / Depth / CameraInfo
    - 通过 TF 查询 map -> camera，避免传感器桥反向依赖定位
    - 队列缓存帧，主循环按 skip 频率处理
    """

    def __init__(self, config: Dict[str, Any], output_path: str, scene: str):
        super().__init__("semantic_mapper")
        self.bridge = CvBridge()

        self._setup_output_path(output_path)
        io_utils.save_dict_to_yaml(
            config, "config.yaml", directory=self.output_path)
        config["output_path"] = str(self.output_path)
        self.config = config
        self.scene = scene

        # 频率控制
        self.track_every = config.get("tracking", {}).get("track_every", 1)
        self.get_logger().info(f"Tracking every {self.track_every} frames")
        self.map_every = config.get("mapping", {}).get("map_every", 10)
        self.get_logger().info(f"Mapping every {self.map_every} frames")
        self.segment_every = config.get(
            "semantic", {}).get(
            "segment_every", 10)
        self.get_logger().info(f"Segmenting every {self.segment_every} frames")

        # 设备
        self.device = config.get(
            "device", "cuda" if torch.cuda.is_available() else "cpu")

        # 深度尺度
        self.depth_scale = config["cam"].get("depth_scale", 1000.0)
        self.depth_th = config["cam"].get("depth_th", 4.0)  # 深度过滤阈值（米）
        self.get_logger().info(f"Depth scale: {self.depth_scale}")

        # 可视化
        self.stream = config["vis"].get("stream", False)
        self.show_stream = config["vis"].get("show_stream", False)

        # 队列与帧状态
        self.frame_queue: Deque[Tuple] = deque(
            maxlen=config.get(
                "ros", {}).get(
                "max_queue", 30000))
        self.frame_id = -1
        # ---- 新增并发控制属性 ----
        self.queue_lock = threading.Lock()
        self.semantic_state_lock = threading.RLock()
        self.shutdown_event = threading.Event()
        self.processing_thread = None
        self.processing_error = None
        self.query_server = None
        self.query_server_thread = None
        self.start_time = time.time()
        self.seg_frames = 0

        # 相机内参
        self.cam_info_received = False
        self.intrinsics_np: Optional[np.ndarray] = None
        self.height = None
        self.width = None

        # 直接从 config['cam'] 读取相机内参与尺寸
        cam_cfg = config.get("cam", {})
        fx = cam_cfg.get("fx")
        fy = cam_cfg.get("fy")
        cx = cam_cfg.get("cx")
        cy = cam_cfg.get("cy")
        self.height = cam_cfg.get("H")
        self.width = cam_cfg.get("W")
        if None in (fx, fy, cx, cy, self.height, self.width):
            self.get_logger().warning(
                "cam config missing required fields, intrinsics not fully initialized")
        else:
            self.intrinsics_np = np.array(
                [[fx, 0.0, cx],
                 [0.0, fy, cy],
                 [0.0, 0.0, 1.0]],
                dtype=np.float32
            )
            self.cam_info_received = True
            # 若 data.H/W 未设置，使用 cam 中值作为回退
            data_cfg = self.config.setdefault("data", {})
            data_cfg.setdefault("H", self.height)
            data_cfg.setdefault("W", self.width)
            self.get_logger().info(
                f"Camera intrinsics: "
                f"fx={fx if fx is not None else 'None'}, "
                f"fy={fy if fy is not None else 'None'}, "
                f"cx={cx if cx is not None else 'None'}, "
                f"cy={cy if cy is not None else 'None'}, "
                f"H={self.height if self.height is not None else 'None'}, "
                f"W={self.width if self.width is not None else 'None'}"
            )

        # 模块
        self.slam_backbone = None
        self.logger = Logger(
            self.output_path,
            os.getpid(),
            config.get(
                "use_wandb",
                False))
        self.obj_detect_track = None  # ObjDetectTrack 实例
        self.dataset_name = config["data"].get(
            "dataset_name", "unknown_dataset")

        # 初始化模块
        try:
            cam_intrinsics = torch.from_numpy(
                self.intrinsics_np).float().to(
                self.device)
            self.slam_backbone = get_slam_backbone(self.config, cam_intrinsics)
            self.obj_detect_track = ObjDetectTrack(
                self.config["semantic"],
                self.logger,
                self.dataset_name,
                self.config["data"]["scene_name"],
                cam_intrinsics,
                device=self.device)
            self.get_logger().info("SLAM Initialization done.")
        except Exception as e:
            self.get_logger().error(f"Module initialization failed: {e}")
            raise RuntimeError("Semantic mapping module initialization failed") from e
        # ROS 同步订阅
        ros_cfg = config.get("topic", {})
        rgb_topic = ros_cfg.get(
            "rgb_topic", "/zed/zed_node/rgb/color/rect/image")
        depth_topic = ros_cfg.get(
            "depth_topic",
            "/zed/zed_node/depth/depth_registered")
        camera_info_topic = ros_cfg.get(
            "camera_info_topic", "/zed/zed_node/rgb/camera_info")
        self.world_frame = ros_cfg.get("world_frame", "map")
        self.get_logger().info(
            f"Subscribing to topics: {rgb_topic}, {depth_topic}, {camera_info_topic}")
        # QoS 缓存深度（单独可调）
        qos_rgb_depth = config.get("ros", {}).get("rgb_qos_depth", 30)
        qos_depth_depth = config.get("ros", {}).get("depth_qos_depth", 30)
        qos_info_depth = config.get("ros", {}).get("camera_info_qos_depth", 30)

        # # 选择可靠性策略（如需与发布端一致可做条件判断）
        # reliability = QoSReliabilityPolicy.BEST_EFFORT
        # 如果需要可靠传输改成：
        reliability = QoSReliabilityPolicy.RELIABLE

        qos_rgb = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_rgb_depth
        )
        qos_depth = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth_depth
        )
        qos_info = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_info_depth
        )

        rgb_sub = Subscriber(self, Image, rgb_topic, qos_profile=qos_rgb)
        depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos_depth)
        camera_info_sub = Subscriber(
            self,
            CameraInfo,
            camera_info_topic,
            qos_profile=qos_info)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sync_slop = ros_cfg.get("sync_slop", 0.05)
        queue_size = ros_cfg.get("sync_queue_size", 30)
        self.ts = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, camera_info_sub],
            queue_size=queue_size,
            slop=sync_slop)
        self.ts.registerCallback(self._synced_cb)

        # 关键帧局部点云发布
        ros_runtime_cfg = config.get("ros", {})
        self.publish_keyframe_pcd = ros_runtime_cfg.get(
            "publish_keyframe_pcd", True)
        self.keyframe_pcd_pub = None
        self.keyframe_pcd_topic = ros_runtime_cfg.get(
            "keyframe_pcd_topic", "/ovo/keyframe_points")
        self.keyframe_pcd_frame_id = ros_runtime_cfg.get(
            "keyframe_pcd_frame_id", "zed_map")
        keyframe_pcd_qos_depth = ros_runtime_cfg.get(
            "keyframe_pcd_qos_depth", 5)
        if self.publish_keyframe_pcd:
            qos_keyframe_pcd = QoSProfile(
                reliability=reliability,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=keyframe_pcd_qos_depth
            )
            self.keyframe_pcd_pub = self.create_publisher(
                PointCloud2, self.keyframe_pcd_topic, qos_keyframe_pcd)
            self.get_logger().info(
                f"Publishing keyframe point cloud to {self.keyframe_pcd_topic} "
                f"with frame_id={self.keyframe_pcd_frame_id}")

        # 新增：全局语义点云发布（带 id）
        self.publish_global_pcd = ros_runtime_cfg.get(
            "publish_global_pcd", True)
        self.global_pcd_pub = None
        self.global_pcd_topic = ros_runtime_cfg.get(
            "global_pcd_topic", "/ovo/global_points")
        self.global_pcd_frame_id = ros_runtime_cfg.get(
            "global_pcd_frame_id", "zed_map")
        self.global_pcd_qos_depth = ros_runtime_cfg.get(
            "global_pcd_qos_depth", 1)
        self.global_pcd_every = max(
            1, int(
                ros_runtime_cfg.get(
                    "global_pcd_every", self.segment_every)))
        if self.publish_global_pcd:
            qos_global_pcd = QoSProfile(
                reliability=reliability,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=self.global_pcd_qos_depth
            )
            self.global_pcd_pub = self.create_publisher(
                PointCloud2, self.global_pcd_topic, qos_global_pcd)
            self.get_logger().info(
                f"Publishing global point cloud to {self.global_pcd_topic} "
                f"with frame_id={self.global_pcd_frame_id}, every={self.global_pcd_every} frames")

        # 可视化进程资源
        self.vis_process = None
        self.mpqueue = None
        self.query_flag = None
        self.query_pipe = None
        self.vis_pipe = None

        self._start_query_server()
        self.get_logger().info("SemanticMapping node initialized, waiting for data...")

    # ---------------- 同步帧回调 ----------------

    def _synced_cb(
            self,
            rgb_msg: Image,
            depth_msg: Image,
            camera_info_msg: CameraInfo):
        if not self.cam_info_received:
            self.get_logger().warning("Camera info not received yet, skipping frame.")
            return
        self.get_logger().info(f"Received frame {self.frame_id + 1}")
        rgb_full = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        depth_raw = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding="passthrough")
        depth = depth_raw.astype(np.float32) / self.depth_scale  # 米尺度
        # filter depth
        if self.depth_th is not None:
            depth[depth > self.depth_th] = 0.0
        # filter by camera y-axis (remove points with Y > -2.0 m)
        if self.intrinsics_np is not None:
            fy = float(self.intrinsics_np[1, 1])
            cy = float(self.intrinsics_np[1, 2])
            H, W = depth.shape[:2]
            rows = np.arange(H, dtype=np.float32).reshape(H, 1)
            # Y = (v - cy) * Z / fy
            Y = (rows - cy) * depth / fy
            depth[Y < -1.0] = 0.0
        incoming_intrinsics = np.asarray(camera_info_msg.k, dtype=np.float32).reshape(3, 3)
        if not np.allclose(incoming_intrinsics, self.intrinsics_np, atol=1e-3):
            self.get_logger().error("CameraInfo does not match semantic mapper intrinsics")
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                rgb_msg.header.frame_id,
                Time.from_msg(rgb_msg.header.stamp),
                timeout=Duration(seconds=0.05))
        except TransformException as exc:
            self.get_logger().warning(f"Camera TF unavailable for frame: {exc}")
            return
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        tx, ty, tz = translation.x, translation.y, translation.z
        R = self._quat_to_rot(np.array([qw, qx, qy, qz], dtype=np.float32))
        c2w_np = np.eye(4, dtype=np.float32)
        c2w_np[:3, :3] = R
        c2w_np[:3, 3] = [tx, ty, tz]
        # c2w_np = np.linalg.inv(c2w_np)  # cam to world
        default_post_transform = [[0, 0, 1, -0.01],
                                  [-1, 0, 0, 0.06],
                                  [0, -1, 0, 0.015],
                                  [0, 0, 0, 1]]
        post_transform = np.asarray(
            self.config.get("pose_post_transform", default_post_transform),
            dtype=np.float32)
        if post_transform.shape != (4, 4) or not np.isfinite(post_transform).all():
            raise ValueError("pose_post_transform must be a finite 4x4 matrix")
        c2w_np = c2w_np @ post_transform

        # 按配置低分辨率
        target_W = self.config["data"].get("W", self.width)
        target_H = self.config["data"].get("H", self.height)
        if (target_W, target_H) != (self.width, self.height):
            import cv2
            rgb_lr = cv2.resize(
                rgb_full, (target_W, target_H), interpolation=cv2.INTER_LINEAR)
            depth_lr = cv2.resize(
                depth, (target_W, target_H), interpolation=cv2.INTER_NEAREST)
        else:
            rgb_lr, depth_lr = rgb_full, depth

        self.frame_id += 1
        fid = self.frame_id

        # 保持与 ScanNet 一致：提供 full-res => 5 元组
        #
        frame_data = (fid, rgb_lr, depth_lr, c2w_np, rgb_full)
        # 若不需要 full-res，可改为 (fid, rgb_lr, depth_lr, c2w_np)
        with self.queue_lock:  # 新增锁，防止与消费线程竞争
            self.frame_queue.append(frame_data)

    # ---------------- 后台处理线程循环（新增） ----------------
    def _processing_loop(self):
        while not self.shutdown_event.is_set():
            with self.queue_lock:
                if not self.frame_queue:
                    frame_data = None
                else:
                    frame_data = self.frame_queue.popleft()
            if frame_data is None:
                time.sleep(0.002)
                continue

            fid, rgb_lr, depth_lr, c2w_np, rgb_full = frame_data
            self.get_logger().info(
                f"Frame queue size: {len(self.frame_queue)}")
            self.get_logger().info("Processing frame %d" % fid)
            self.get_logger().info("Camera pose: %s" % c2w_np[:3, 3])

            # 跳帧：仅当某种处理需要才继续（保持原逻辑）
            if (fid % self.track_every != 0 and
                fid % self.map_every != 0 and
                    fid % self.segment_every != 0):
                continue

            self._process_frame(frame_data)

            if fid % self.segment_every == 0:
                self.seg_frames += 1
            if fid % 50 == 0:
                gc.collect()

        # 线程退出前尝试处理剩余帧（可选，快速清空）
        while True:
            with self.queue_lock:
                if not self.frame_queue:
                    break
                frame_data = self.frame_queue.popleft()
            fid = frame_data[0]
            if (fid % self.track_every != 0 and
                fid % self.map_every != 0 and
                    fid % self.segment_every != 0):
                continue
            self._process_frame(frame_data)
            if fid % self.segment_every == 0:
                self.seg_frames += 1

    def _processing_loop_guarded(self):
        try:
            self._processing_loop()
        except BaseException as exc:
            self.processing_error = exc
            self.shutdown_event.set()
            self.get_logger().error(
                f"Frame processor failed: {type(exc).__name__}: {exc}")

    # ---------------- 主循环（重写，轻量化，仅 spin ROS） ----------------
    def spin(self):
        try:
            if self.stream:
                self._start_stream()
            # 启动后台处理线程
            self.processing_thread = threading.Thread(
                target=self._processing_loop_guarded,
                name="FrameProcessor",
                daemon=True)
            self.processing_thread.start()

            while rclpy.ok() and not self.shutdown_event.is_set():
                rclpy.spin_once(self, timeout_sec=0.01)
        except KeyboardInterrupt:
            self.get_logger().info("Ctrl-C 中断，准备退出...")
        finally:
            # 通知线程退出
            self.shutdown_event.set()
            if self.processing_thread is not None:
                self.processing_thread.join()
            self._stop_query_server()

            if self.processing_error is None:
                if self.obj_detect_track:
                    self.obj_detect_track.complete_semantic_info()
                if self.seg_frames > 0:
                    fps = self.seg_frames / (time.time() - self.start_time)
                    self.logger.log_fps(fps)
                self._drain_queries()
                self._finalize()

            if self.stream and self.vis_process is not None:
                try:
                    self.vis_process.terminate()
                except Exception:
                    pass

        if self.processing_error is not None:
            raise RuntimeError("Semantic mapping frame processor failed") from self.processing_error

    def query_live_object(self, query: str) -> Dict[str, Any]:
        """Return the best current OVO instance and its map-frame centroid."""
        query = query.strip()
        if not query:
            raise ValueError("query is empty")
        with self.semantic_state_lock, torch.no_grad():
            object_ids = list(self.obj_detect_track.objects)
            if not object_ids:
                raise LookupError("online semantic map has no objects")
            scores = self.obj_detect_track.query([query])[0].detach().float().cpu().numpy()
            objects = [self.obj_detect_track.objects[object_id] for object_id in object_ids]
            order = np.argsort(scores)[::-1]
            best_index = int(order[0])
            best_score = float(scores[best_index])
            second_score = float(scores[int(order[1])]) if len(order) > 1 else float("-inf")
            min_score = float(self.config.get("ros", {}).get("query_min_score", 0.1))
            min_margin = float(self.config.get("ros", {}).get("query_min_margin", 0.03))
            if not np.isfinite(best_score) or best_score < min_score:
                raise LookupError(f"best score {best_score:.4f} is below {min_score:.4f}")
            if np.isfinite(second_score) and best_score - second_score < min_margin:
                raise RuntimeError(
                    f"ambiguous result: margin {best_score - second_score:.4f} is below {min_margin:.4f}")

            object_id = object_ids[best_index]
            pcd, _, pcd_object_ids = self.slam_backbone.get_map()
            pcd = pcd.detach().cpu().numpy()
            pcd_object_ids = pcd_object_ids.detach().cpu().numpy().reshape(-1)
            if len(pcd) != len(pcd_object_ids):
                raise RuntimeError("online map changed during query")
            points = pcd[pcd_object_ids == object_id]
            points = points[np.isfinite(points).all(axis=1)]
            if not len(points):
                raise LookupError(f"object {object_id} has no finite map points")
            center = np.median(points, axis=0)
            obj = objects[best_index]
        return {
            "status": "FOUND",
            "object_query": query,
            "online_object_id": int(object_id),
            "score": best_score,
            "second_score": second_score if np.isfinite(second_score) else None,
            "center_map": center.tolist(),
            "observation_count": len(obj.kfs_ids),
            "last_frame_id": max(obj.kfs_ids) if obj.kfs_ids else None,
            "source_timestamp_ms": int(time.time() * 1000),
            "frame_id": self.world_frame,
        }

    def _start_query_server(self) -> None:
        ros_config = self.config.get("ros", {})
        if not ros_config.get("query_server_enabled", True):
            return
        node = self

        class QueryHandler(BaseHTTPRequestHandler):
            def do_POST(self):
                if self.path != "/query":
                    self.send_error(404)
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    payload = json.loads(self.rfile.read(length) or b"{}")
                    result = node.query_live_object(str(payload.get("object_query", "")))
                    body, status = json.dumps(result).encode("utf-8"), 200
                except ValueError as exc:
                    body, status = json.dumps({"detail": str(exc)}).encode("utf-8"), 400
                except LookupError as exc:
                    body, status = json.dumps({"detail": str(exc)}).encode("utf-8"), 404
                except RuntimeError as exc:
                    body, status = json.dumps({"detail": str(exc)}).encode("utf-8"), 409
                except Exception as exc:
                    node.get_logger().error(f"Live semantic query failed: {exc}")
                    body, status = json.dumps({"detail": "live query failed"}).encode("utf-8"), 500
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format, *args):
                return

        host = str(ros_config.get("query_server_host", "127.0.0.1"))
        port = int(ros_config.get("query_server_port", 8121))
        self.query_server = ThreadingHTTPServer((host, port), QueryHandler)
        self.query_server_thread = threading.Thread(
            target=self.query_server.serve_forever,
            name="SemanticQueryServer",
            daemon=True)
        self.query_server_thread.start()
        self.get_logger().info(f"Live semantic query endpoint: http://{host}:{port}/query")

    def _stop_query_server(self) -> None:
        if self.query_server is None:
            return
        self.query_server.shutdown()
        self.query_server.server_close()
        if self.query_server_thread is not None:
            self.query_server_thread.join(timeout=2.0)
        self.query_server = None

    # ---------------- 单帧处理 ----------------
    def _process_frame(self, frame_data):
        with self.semantic_state_lock:
            return self._process_frame_unlocked(frame_data)

    def _process_frame_unlocked(self, frame_data):
        """frame_data: (fid, rgb_lr, depth_lr, c2w_np[, rgb_full])"""
        fid, rgb_lr, depth_lr, c2w_np, *optional = frame_data
        # 直接传给 SLAM
        self.slam_backbone.track_camera(frame_data)

        estimated_c2w = self.slam_backbone.get_c2w(fid)
        depth_valid = (depth_lr > 0).any()
        if estimated_c2w is None or not depth_valid:
            return

        if fid % self.map_every == 0 or self.config["slam"].get(
                "slam_module") == "livwo":
            self.slam_backbone.map(frame_data, estimated_c2w)
            if self.slam_backbone.map_updated:
                map_data = self.slam_backbone.get_map()
                kfs = self.slam_backbone.get_kfs()
                updated_ids = self.obj_detect_track.update_map(map_data, kfs)
                if updated_ids is not None:
                    self.slam_backbone.update_pcd_obj_ids(updated_ids)
                self.slam_backbone.map_updated = False

        # Segment
        if fid % self.segment_every == 0:
            with torch.no_grad(), torch.autocast(
                device_type=self.device.split(":", 1)[0],
                dtype=torch.bfloat16,
                enabled=self.device.startswith("cuda")
            ):
                image = optional[0] if optional else rgb_lr

                # 判断是否需要 rgb_depth_ratio
                # 这里 dataset 的基准分辨率来自 config["data"]["H"/"W"]
                base_H = self.config["data"]["H"]
                base_W = self.config["data"]["W"]
                if image.shape[0] != base_H or image.shape[1] != base_W:
                    rgb_depth_ratio = (image.shape[0] / base_H,
                                       image.shape[1] / base_W,
                                       0)   # 原 crop_edge 可按需替换
                else:
                    rgb_depth_ratio = ()

                scene_data = [fid, image, depth_lr, rgb_depth_ratio]
                map_data = self.slam_backbone.get_map()
                updated_ids, _, _ = self.obj_detect_track.detect_and_track_objects(
                    scene_data, map_data, estimated_c2w)
                if updated_ids is not None:
                    self.slam_backbone.update_pcd_obj_ids(updated_ids)
                # clip feature
                self.obj_detect_track.compute_semantic_info()
                self.logger.log_memory_usage(fid)

            if self.publish_keyframe_pcd:
                self._publish_keyframe_pointcloud(fid)

            if self.stream:
                self._push_stream(fid)

        # # 新增：定期发布全局语义点云（全局 map）
        # if self.publish_global_pcd and (fid % self.global_pcd_every == 0):
        #     self._publish_global_pointcloud()

    def _build_xyzrgb_cloud(
            self,
            points: np.ndarray,
            obj_ids: np.ndarray) -> np.ndarray:
        if obj_ids.ndim > 1:
            obj_ids = np.squeeze(obj_ids, axis=-1)

        semantic_colors = vis_utils.get_pcd_colors(
            obj_ids, vis_utils.get_cmap())
        rgb_u8 = np.clip(
            semantic_colors *
            255.0,
            0,
            255).astype(
            np.uint8,
            copy=False)
        packed_rgb = (
            (rgb_u8[:, 0].astype(np.uint32) << 16)
            | (rgb_u8[:, 1].astype(np.uint32) << 8)
            | rgb_u8[:, 2].astype(np.uint32)
        )

        cloud_np = np.empty(
            points.shape[0],
            dtype=[
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("rgb", np.uint32),
            ],
        )
        cloud_np["x"] = points[:, 0]
        cloud_np["y"] = points[:, 1]
        cloud_np["z"] = points[:, 2]
        cloud_np["rgb"] = packed_rgb
        return cloud_np

    def _build_keyframe_pointcloud2(self, fid: int) -> Optional[PointCloud2]:
        if self.slam_backbone is None:
            return None
        kfs = self.slam_backbone.get_kfs()
        kf_info = kfs.get(fid, None)
        if kf_info is None:
            return None

        pcd_idxs = kf_info.get("pcd_idxs", None)
        if pcd_idxs is None or len(pcd_idxs) != 2:
            return None

        start_idx, end_idx = int(pcd_idxs[0]), int(pcd_idxs[1])
        if end_idx <= start_idx:
            return None

        pcd, _, pcd_obj_ids = self.slam_backbone.get_map()
        n_points = int(pcd.shape[0])
        start_idx = max(0, min(start_idx, n_points))
        end_idx = max(0, min(end_idx, n_points))
        if end_idx <= start_idx:
            return None

        kf_points = pcd[start_idx:end_idx].detach(
        ).cpu().numpy().astype(np.float32, copy=False)
        kf_obj_ids = pcd_obj_ids[start_idx:end_idx].detach(
        ).cpu().numpy().astype(np.int32, copy=False)
        cloud_np = self._build_xyzrgb_cloud(kf_points, kf_obj_ids)

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.keyframe_pcd_frame_id
        msg.height = 1
        msg.width = int(cloud_np.shape[0])
        msg.fields = [
            PointField(name="x", offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12,
                       datatype=PointField.UINT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        msg.data = cloud_np.tobytes()
        return msg

    def _publish_keyframe_pointcloud(self, fid: int) -> None:
        if self.keyframe_pcd_pub is None:
            return
        msg = self._build_keyframe_pointcloud2(fid)
        if msg is None or msg.width == 0:
            return
        self.keyframe_pcd_pub.publish(msg)

    def _publish_global_pointcloud(self) -> None:
        if self.global_pcd_pub is None:
            return
        msg = self._build_global_pointcloud2()
        if msg is None or msg.width == 0:
            return
        self.global_pcd_pub.publish(msg)

    def _build_global_pointcloud2(self) -> Optional[PointCloud2]:
        if self.slam_backbone is None:
            return None

        pcd, _, pcd_obj_ids = self.slam_backbone.get_map()
        if pcd is None or int(pcd.shape[0]) == 0:
            return None

        points = pcd.detach().cpu().numpy().astype(np.float32, copy=False)
        obj_ids = pcd_obj_ids.detach().cpu().numpy().astype(np.int32, copy=False)
        cloud_np = self._build_xyzrgb_cloud(points, obj_ids)

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_pcd_frame_id
        msg.height = 1
        msg.width = int(cloud_np.shape[0])
        msg.fields = [
            PointField(name="x", offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12,
                       datatype=PointField.UINT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        msg.data = cloud_np.tobytes()
        return msg

    # ---------------- 可视化 ----------------
    def _start_stream(self):
        cam_data = {
            "height": self.height,
            "width": self.width,
            "intrinsic": self.intrinsics_np
        }
        self.mpqueue = mp.Queue()
        self.query_flag = mp.Value('i', 0)
        self.query_pipe, self.vis_pipe = mp.Pipe()
        self.vis_process = mp.Process(
            target=stream_pcd,
            args=(self.obj_detect_track,
                  self.mpqueue,
                  [self.query_flag, self.vis_pipe],
                  cam_data,
                  self.config["data"]["scene_name"],
                  self.logger.output_path,
                  self.show_stream),
            name="O3DVisualizer"
        )
        self.vis_process.start()

    def _push_stream(self, fid: int):
        pcd, _, pcd_obj_ids = self.slam_backbone.get_map()
        c2w = self.slam_backbone.get_c2w(fid)
        if c2w is None:
            return
        colors = self.slam_backbone.get_pcd_colors()
        self.mpqueue.put([
            pcd.cpu().numpy().astype(np.float16),
            pcd_obj_ids.cpu().numpy().astype(np.int16),
            colors,
            c2w.cpu().numpy().astype(np.float16)
        ])
        if self.query_flag.value == 1:
            query = self.query_pipe.recv()
            query_map = self.obj_detect_track.query(query).cpu().numpy()
            with self.query_flag.get_lock():
                self.query_pipe.send(query_map)
                self.query_flag.value = 2

    def _drain_queries(self):
        if not self.stream or not self.vis_process or not self.vis_process.is_alive():
            return
        while self.mpqueue.qsize() > 0:
            if self.query_flag.value == 1:
                query = self.query_pipe.recv()
                result = self.obj_detect_track.query(query).cpu().numpy()
                with self.query_flag.get_lock():
                    self.query_pipe.send(result)
                    self.query_flag.value = 2
            time.sleep(0.2)
        time.sleep(0.5)

    # ---------------- 保存与清理 ----------------
    def _finalize(self):
        if self.slam_backbone is None:
            return

        self.logger.log_max_memory_usage()
        self.logger.write_stats()
        self.logger.print_final_stats()
        # self.save_representation()
        # self.get_logger().info("Representation saved.")

        # pcd_pred, points_obj_ids, obj_ids = self.slam_backbone.get_map()
        # pcd_pred = pcd_pred.cpu().numpy().astype(np.float16)
        # obj_ids = obj_ids.cpu().numpy().astype(np.int16)
        # if len(obj_ids) > 0:
        #     # 使用 tab20 colormap（兼容 tuple/list 返回，不依赖 shape）
        #     cmap_raw = plt.get_cmap("tab20").colors
        #     # 转成 np.array，并截取前 3 通道 (去掉 alpha)
        #     cmap = np.array(cmap_raw, dtype=np.float32)[..., :3]
        #     # 兼容 obj_ids 可能为 tuple 的情况
        #     mapped_ids = np.array(obj_ids, copy=True)
        #     # 仅对有效 id 做取模
        #     valid_mask = mapped_ids > -1
        #     if valid_mask.any():
        #         mapped_ids[valid_mask] = mapped_ids[valid_mask] % len(cmap)
        #     # 按索引采样颜色
        #     pcd_colors = np.zeros((len(mapped_ids), 3), dtype=np.float32)
        #     if len(cmap) > 0 and valid_mask.any():
        #         pcd_colors[valid_mask] = cmap[mapped_ids[valid_mask]]
        #     n_cls = len(np.unique(mapped_ids[valid_mask])) if valid_mask.any() else 0
        #     self.get_logger().info(f"Found {n_cls} unique object IDs in the map.")
        # else:
        #     pcd_colors = np.zeros((len(obj_ids), 3), dtype=np.float32)

        # self.get_logger().info(f"point cloud size: {len(pcd_pred)}")
        # self.get_logger().info(f"obj ids size: {len(pcd_colors)}")
        # pcd_o3d = o3d.geometry.PointCloud()
        # pcd_o3d.points = o3d.utility.Vector3dVector(pcd_pred)
        # pcd_o3d.colors = o3d.utility.Vector3dVector(pcd_colors)  # 归一化到 [0,
        # 1]，float32

        # # downsample to 0.01m
        # pcd_o3d = pcd_o3d.voxel_down_sample(voxel_size=0.1)
        # out_path = str(self.output_path / "pred.pcd")
        # o3d.io.write_point_cloud(out_path, pcd_o3d)
        # self.get_logger().info(f"Semantic pointcloud save : {out_path}")

        self.save_representation()
        # save map_data[0]
        pcd_pred, points_obj_ids, pcd_obj_ids = self.slam_backbone.get_map()
        pcd_pred = pcd_pred.cpu().numpy().astype(np.float32)
        points_obj_ids = points_obj_ids.cpu().numpy().astype(np.int32)
        pcd_obj_ids = pcd_obj_ids.cpu().numpy().astype(np.int32)
        # # save to pcd file
        # # 创建 Open3D 点云对象
        pcd_colors = self.slam_backbone.get_pcd_colors()
        pcd_rgb = o3d.geometry.PointCloud()
        pcd_rgb.points = o3d.utility.Vector3dVector(pcd_pred)  # 设置点云坐标
        pcd_rgb.colors = o3d.utility.Vector3dVector(
            pcd_colors / 255.)  # 将颜色归一化到 [0, 1]
        out_path = str(self.output_path) + "/" + self.scene + "_rgb.ply"
        o3d.io.write_point_cloud(out_path, pcd_rgb)
        # load SCANNET_COLOR_MAP_200 from saved config.yaml if present

        # obj_colors = vis_utils.get_pcd_colors(pcd_obj_ids, vis_utils.get_cmap())
        print("get_obj_ids_and_masks started...")
        ids_input = list(self.obj_detect_track.objects.keys())
        ids_input = np.array(ids_input, dtype=np.int32)
        obj_masks, ids = vis_utils.get_obj_ids_and_masks(
            pcd_obj_ids, ids_input)
        print("get_obj_ids_and_masks completed.")

        # 从 obj_masks 和 ids 中获取每个对象的点云和 OBB
        # obj_colors_for_instances 是为每个唯一ID生成的语义颜色
        print("get_obj_and_obb started...")
        obj_colors_for_instances = vis_utils.get_pcd_colors(
            ids, vis_utils.get_cmap())
        print("obj_colors_for_instances completed.")

        print("Generating OBBs and object point clouds...")
        obb_obj_list = vis_utils.get_obj_and_obb(
            obj_masks, pcd_pred, obj_colors_for_instances)
        print("Generating OBBs and object point clouds completed.")
        # 保存到文件
        instance_dir = Path(str(self.output_path) + "/instance/")
        instance_dir.mkdir(exist_ok=True, parents=True)
        merged_cloud = o3d.geometry.PointCloud()
        for i, obj_pcd in enumerate(obb_obj_list):
            merged_cloud += obj_pcd.to_legacy()
            out_path = str(
                instance_dir / (self.scene + f"_obj_{ids[i]}_obb.ply"))
            o3d.io.write_point_cloud(out_path, obj_pcd.to_legacy())
            print(f"Object {ids[i]} point cloud saved to {out_path}")

        print(f"point cloud size: {len(pcd_pred)}")
        print(f"obj ids size: {len(ids)}")
        print(f"ovo instances: {len(self.obj_detect_track.objects)}")
        print(f"obj_masks size: {len(obj_masks)}")

        # pcd_o3d = o3d.geometry.PointCloud()
        # pcd_o3d.points = o3d.utility.Vector3dVector(pcd_pred)
        # pcd_o3d.colors = o3d.utility.Vector3dVector(obj_colors)  # 归一化到 [0, 1]，float32
        # 保存到文件
        out_path = str(self.output_path) + "/" + self.scene + "_obj.ply"
        o3d.io.write_point_cloud(out_path, merged_cloud)
        print(f"Point cloud saved to {out_path}")

        self.obj_detect_track.cpu()
        del self.slam_backbone, self.obj_detect_track
        torch.cuda.empty_cache()

    def _setup_output_path(self, output_path: str):
        self.output_path = Path(output_path)
        self.output_path.mkdir(parents=True, exist_ok=True)

    def save_representation(self):
        map_params = self.slam_backbone.get_map_dict()
        ovo_map_params = self.obj_detect_track.capture_dict(
            debug_info=self.config.get("debug", False))
        ckpt = {
            "map_params": map_params,
            "ovo_map_params": ovo_map_params
        }
        io_utils.save_dict_to_ckpt(
            ckpt, "ovo_map.ckpt", directory=self.output_path)

    @staticmethod
    def _quat_to_rot(qwxyz: np.ndarray) -> np.ndarray:
        w, x, y, z = qwxyz
        n = w * w + x * x + y * y + z * z
        if n < 1e-8:
            return np.eye(3, dtype=np.float32)
        s = 2.0 / n
        wx, wy, wz = s * w * x, s * w * y, s * w * z
        xx, xy, xz = s * x * x, s * x * y, s * x * z
        yy, yz, zz = s * y * y, s * y * z, s * z * z
        return np.array([
            [1 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1 - (xx + yy)]
        ], dtype=np.float32)
