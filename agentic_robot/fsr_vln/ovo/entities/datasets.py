"""Slightly modified code based on Gaussian-SLAM's datasets."""

import math
import os
from pathlib import Path

import cv2
import numpy as np
import torch
import json
import imageio
import re
from scipy.spatial.transform import Rotation as R


class BaseDataset(torch.utils.data.Dataset):

    def __init__(self, dataset_config: dict):
        self.dataset_path = Path(dataset_config["input_path"])
        print(f"Loading dataset from {self.dataset_path}")
        self.frame_limit = dataset_config.get("frame_limit", -1)
        self.dataset_config = dataset_config
        resize_ratio = dataset_config.get("resize_ratio", 1.0)
        self.height = int(dataset_config["H"] * resize_ratio)
        self.width = int(dataset_config["W"] * resize_ratio)
        self.time_stamps = []
        self.fx = dataset_config["fx"] * resize_ratio
        self.fy = dataset_config["fy"] * resize_ratio
        self.cx = dataset_config["cx"] * resize_ratio
        self.cy = dataset_config["cy"] * resize_ratio
        self.original_intrinsics = np.array(
            [[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])

        target_res = dataset_config.get("target_res", None)

        self.depth_scale = dataset_config["depth_scale"]
        self.distortion = np.array(
            dataset_config['distortion']) if 'distortion' in dataset_config else None

        self.crop_edge = dataset_config['crop_edge'] if 'crop_edge' in dataset_config else 0
        if self.crop_edge > 0:
            self.height -= 2 * self.crop_edge
            self.width -= 2 * self.crop_edge
            self.cx -= self.crop_edge
            self.cy -= self.crop_edge

        if target_res is not None:
            self.fx *= (target_res[1] / self.width)
            self.fy *= (target_res[0] / self.height)
            self.cx *= (target_res[1] / self.width)
            self.cy *= (target_res[0] / self.height)
            self.height = target_res[0]
            self.width = target_res[1]

        self.fovx = 2 * math.atan(self.width / (2 * self.fx))
        self.fovy = 2 * math.atan(self.height / (2 * self.fy))
        self.intrinsics = np.array(
            [[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])

        self.color_paths = []
        self.depth_paths = []
        self.depth_overrides = {}

    def __len__(self):
        return len(
            self.color_paths) if self.frame_limit < 0 else int(
            self.frame_limit)


class Replica(BaseDataset):

    def __init__(self, dataset_config: dict):
        super().__init__(dataset_config)
        self.color_paths = sorted(
            list((self.dataset_path / "results").glob("frame*.jpg")))
        self.depth_paths = sorted(
            list((self.dataset_path / "results").glob("depth*.png")))
        self.load_poses(self.dataset_path / "traj.txt")
        print(f"Loaded {len(self.color_paths)} frames")

    def load_poses(self, path):
        self.poses = []
        with open(path, "r") as f:
            lines = f.readlines()
        for line in lines:
            c2w = np.array(list(map(float, line.split()))).reshape(4, 4)
            T_switch_axis = np.array(
                [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=np.float32)
            c2w = T_switch_axis @ c2w
            self.poses.append(c2w.astype(np.float32))

    def __getitem__(self, index):
        color_data = cv2.imread(str(self.color_paths[index]))
        color_data = cv2.resize(
            color_data,
            (self.width,
             self.height),
            interpolation=cv2.INTER_LINEAR)  # added
        color_data = color_data.astype(np.uint8)  # added
        color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)
        if hasattr(self, 'depth_overrides') and index in self.depth_overrides:
            depth_data = self.depth_overrides[index]
        else:
            depth_data = cv2.imread(
                str(self.depth_paths[index]), cv2.IMREAD_UNCHANGED)
            depth_data = cv2.resize(
                depth_data.astype(float),
                (self.width,
                 self.height),
                interpolation=cv2.INTER_NEAREST)
            depth_data = depth_data.astype(np.float32) / self.depth_scale

        return index, color_data, depth_data, self.poses[index]


class ScanNet(BaseDataset):
    def __init__(self, dataset_config: dict):
        super().__init__(dataset_config)

        def _name_key(x):
            name = os.path.splitext(os.path.basename(str(x)))[0]
            try:
                return float(name)
            except ValueError:
                return name

        self.color_paths = sorted(
            list((self.dataset_path / "color").glob("*.jpg")) +
            list((self.dataset_path / "color").glob("*.png")),
            key=_name_key
        )
        self.depth_paths = sorted(
            list((self.dataset_path / "depth").glob("*.png")) +
            list((self.dataset_path / "depth").glob("*.jpg")),
            key=_name_key
        )
        self.time_stamps = []
        self.time_stamps = [
            float(
                os.path.splitext(
                    os.path.basename(
                        str(path)))[0]) for path in self.color_paths]

        print(f"Loaded {len(self.color_paths)} color frames")
        print(f"Loaded {len(self.depth_paths)} depth frames")
        self.load_poses(self.dataset_path / "pose")
        depth_th = dataset_config.get("depth_th", 0)
        if depth_th > 0:
            self.depth_th = depth_th
        else:
            self.depth_th = None

    def load_poses(self, path):
        self.poses = []

        def _name_key(x):
            name = os.path.splitext(os.path.basename(str(x)))[0]
            try:
                return int(name)
            except ValueError:
                try:
                    return float(name)
                except ValueError:
                    return name

        pose_paths = sorted(path.glob('*.txt'),
                            key=_name_key)
        for pose_path in pose_paths:
            with open(pose_path, "r") as f:
                lines = f.readlines()
            ls = []
            for line in lines:
                ls.append(list(map(float, line.split())))
            c2w = np.array(ls).reshape(4, 4).astype(np.float32)
            T_switch_axis = np.array(
                [[1, 0, 0, -2.5], [0, 0, 1, 0], [0, -1, 0, 2.5], [0, 0, 0, 1]], dtype=np.float32)
            c2w = T_switch_axis @ c2w
            self.poses.append(c2w)

    def __getitem__(self, index):
        color_data = cv2.imread(str(self.color_paths[index]))
        if self.distortion is not None:
            color_data = cv2.undistort(
                color_data, self.intrinsics, self.distortion)
        color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)

        using_override = hasattr(
            self, 'depth_overrides') and index in self.depth_overrides
        if using_override:
            depth_data = self.depth_overrides[index]
            if not isinstance(depth_data, np.ndarray):
                depth_data = np.array(depth_data)
            depth_data = depth_data.astype(np.float32)
            self.depth_th = 20.0
        else:
            depth_data = cv2.imread(
                str(self.depth_paths[index]), cv2.IMREAD_UNCHANGED)
            depth_data = depth_data.astype(np.float32) / self.depth_scale

        if self.depth_th is not None:
            depth_data[depth_data > self.depth_th] = 0

        edge = self.crop_edge
        if edge > 0:
            lr_color_data = color_data[edge:-edge, edge:-edge]
            if not using_override:
                depth_data = depth_data[edge:-edge, edge:-edge]
        else:
            lr_color_data = color_data

        lr_color_data = cv2.resize(
            lr_color_data,
            (self.width,
             self.height),
            interpolation=cv2.INTER_LINEAR)
        if not using_override:
            depth_data = cv2.resize(
                depth_data, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
        # Interpolate depth values for splatting
        return index, lr_color_data, depth_data, self.poses[index]


class G1(BaseDataset):
    def __init__(self, dataset_config: dict):
        super().__init__(dataset_config)

        def _name_key(x):
            name = os.path.splitext(os.path.basename(str(x)))[0]
            try:
                return float(name)
            except ValueError:
                return name
        # load time
        self.time_stamps = []

        self.color_paths = sorted(
            list((self.dataset_path / "color").glob("*.jpg")) +
            list((self.dataset_path / "color").glob("*.png")),
            key=_name_key
        )
        self.depth_paths = sorted(
            list((self.dataset_path / "depth").glob("*.png")) +
            list((self.dataset_path / "depth").glob("*.jpg")),
            key=_name_key
        )
        self.time_stamps = [
            float(
                os.path.splitext(
                    os.path.basename(
                        str(path)))[0]) for path in self.color_paths]
        print(f"Loaded {len(self.color_paths)} color frames")
        print(f"Loaded {len(self.depth_paths)} depth frames")
        self.load_poses(self.dataset_path / "pose")
        if len(self.poses) != len(self.color_paths) or len(self.poses) != len(self.depth_paths):
            valid_length = min(len(self.color_paths), len(
                self.depth_paths), len(self.poses))
            print(
                f"Aligning G1 dataset lengths to {valid_length} "
                f"(color={len(self.color_paths)}, depth={len(self.depth_paths)}, poses={len(self.poses)})"
            )
            self.color_paths = self.color_paths[:valid_length]
            self.depth_paths = self.depth_paths[:valid_length]
            self.poses = self.poses[:valid_length]
            self.time_stamps = self.time_stamps[:valid_length]
        depth_th = dataset_config.get("depth_th", 0)
        if depth_th > 0:
            self.depth_th = depth_th
        else:
            self.depth_th = None

    def load_poses(self, path):
        self.poses = []

        poses_txt_path = self.dataset_path / "poses.txt"
        if poses_txt_path.exists():
            print(f"Loading G1 poses from {poses_txt_path}")
            tum_pose_raw = np.loadtxt(poses_txt_path)
            if tum_pose_raw.ndim == 1:
                tum_pose_raw = tum_pose_raw.reshape(1, -1)
            tum_pose_raw = tum_pose_raw[tum_pose_raw[:, 0].argsort()]
            for pose in tum_pose_raw:
                ts, tx, ty, tz, qx, qy, qz, qw = pose
                quat = [qx, qy, qz, qw]
                rot_matrix = R.from_quat(quat).as_matrix()
                T = np.eye(4, dtype=np.float32)
                T[:3, :3] = rot_matrix.astype(np.float32)
                T[:3, 3] = np.array([tx, ty, tz], dtype=np.float32)
                c2w = np.linalg.inv(T).astype(np.float32)
                self.poses.append(c2w)
            return

        def _name_key(x):
            name = os.path.splitext(os.path.basename(str(x)))[0]
            try:
                return int(name)
            except ValueError:
                try:
                    return float(name)
                except ValueError:
                    return name

        pose_paths = sorted(path.glob('*.txt'),
                            key=_name_key)
        for pose_path in pose_paths:
            with open(pose_path, "r") as f:
                lines = f.readlines()
            ls = []
            for line in lines:
                ls.append(list(map(float, line.split())))
            c2w = np.array(ls).reshape(4, 4).astype(np.float32)
            # T_switch_axis = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [
            #                     0, -1, 0, 0], [0, 0, 0, 1]], dtype=np.float32)  #  fastlivo2
            # c2w = T_switch_axis @ c2w
            self.poses.append(c2w)

    def __getitem__(self, index):
        color_data = cv2.imread(str(self.color_paths[index]))
        if self.distortion is not None:
            color_data = cv2.undistort(
                color_data, self.intrinsics, self.distortion)
        color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)

        using_override = hasattr(
            self, 'depth_overrides') and index in self.depth_overrides
        if using_override:
            depth_data = self.depth_overrides[index]
            if not isinstance(depth_data, np.ndarray):
                depth_data = np.array(depth_data)
            depth_data = depth_data.astype(np.float32)
            self.depth_th = 20.0
        else:
            depth_data = cv2.imread(
                str(self.depth_paths[index]), cv2.IMREAD_UNCHANGED)
            depth_data = depth_data.astype(np.float32) / self.depth_scale

        if self.depth_th is not None:
            depth_data[depth_data > self.depth_th] = 0

        edge = self.crop_edge
        if edge > 0:
            lr_color_data = color_data[edge:-edge, edge:-edge]
            if not using_override:
                depth_data = depth_data[edge:-edge, edge:-edge]
        else:
            lr_color_data = color_data

        lr_color_data = cv2.resize(
            lr_color_data,
            (self.width,
             self.height),
            interpolation=cv2.INTER_LINEAR)
        if not using_override:
            depth_data = cv2.resize(
                depth_data, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
        # Interpolate depth values for splatting
        return index, lr_color_data, depth_data, self.poses[index]


class TUM(BaseDataset):
    def __init__(self, dataset_config: dict):
        super().__init__(dataset_config)

        def _name_key(x):
            name = os.path.splitext(os.path.basename(str(x)))[0]
            try:
                return float(name)
            except ValueError:
                return name

        # Load RGB and depth images sorted by timestamp
        self.color_paths = sorted(
            list((self.dataset_path / "rgb").glob("*.jpg")) +
            list((self.dataset_path / "rgb").glob("*.png")),
            key=_name_key
        )
        self.depth_paths = sorted(
            list((self.dataset_path / "depth").glob("*.png")) +
            list((self.dataset_path / "depth").glob("*.jpg")),
            key=_name_key
        )
        print(f"Loaded {len(self.color_paths)} color frames")
        print(f"Loaded {len(self.depth_paths)} depth frames")

        # Load poses from groundtruth.txt and match timestamps
        self.load_poses(self.dataset_path / "groundtruth.txt")

        # Match RGB, depth, and pose timestamps
        self.match_timestamps()

        depth_th = dataset_config.get("depth_th", 0)
        if depth_th > 0:
            self.depth_th = depth_th
        else:
            self.depth_th = None

        # 预先计算remap映射表（如果有畸变参数）
        self.use_remap = self.distortion is not None
        self.map1 = None
        self.map2 = None
        if self.use_remap:
            # 假设所有图片shape一致，取第一张图片shape
            if len(self.color_paths) > 0:
                sample_img = cv2.imread(str(self.color_paths[0]))
                h, w = sample_img.shape[:2]
                self.map1, self.map2 = cv2.initUndistortRectifyMap(
                    self.original_intrinsics, self.distortion, None, self.original_intrinsics, (w, h), cv2.CV_16SC2)

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """Convert quaternion (qx, qy, qz, qw) to rotation matrix."""
        # Normalize quaternion
        norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-8:
            return np.eye(3)
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        # Convert to rotation matrix
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
             2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 *
             (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
             1 - 2 * (qx * qx + qy * qy)]
        ])
        return R

    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion (qx, qy, qz, qw)."""
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] -
                            R[2, 2]) * 2  # s = 4 * qx
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] -
                            R[2, 2]) * 2  # s = 4 * qy
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] -
                            R[1, 1]) * 2  # s = 4 * qz
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s

        # Normalize
        norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm > 1e-8:
            qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        return qx, qy, qz, qw

    def quaternion_slerp(self, q1, q2, t):
        """
        Spherical linear interpolation (SLERP) between two quaternions.

        Args:
            q1: First quaternion (qx, qy, qz, qw)
            q2: Second quaternion (qx, qy, qz, qw)
            t: Interpolation parameter [0, 1]

        Returns:
            Interpolated quaternion (qx, qy, qz, qw)
        """
        q1 = np.array(q1)
        q2 = np.array(q2)

        # Normalize quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)

        # Compute dot product
        dot = np.dot(q1, q2)

        # If dot product is negative, negate one quaternion to take shorter
        # path
        if dot < 0.0:
            q2 = -q2
            dot = -dot

        # If quaternions are very close, use linear interpolation
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)

        # Calculate angle between quaternions
        theta_0 = np.arccos(np.abs(dot))
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)

        # SLERP formula
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return (s0 * q1 + s1 * q2) / np.linalg.norm(s0 * q1 + s1 * q2)

    def interpolate_pose(self, pose1, pose2, t):
        """
        Interpolate between two poses.

        Args:
            pose1: First pose (4x4 c2w matrix)
            pose2: Second pose (4x4 c2w matrix)
            t: Interpolation parameter [0, 1]

        Returns:
            Interpolated pose (4x4 c2w matrix)
        """
        # Extract rotation and translation
        R1 = pose1[:3, :3]
        t1 = pose1[:3, 3]
        R2 = pose2[:3, :3]
        t2 = pose2[:3, 3]

        # Interpolate rotation using SLERP
        q1 = self.rotation_matrix_to_quaternion(R1)
        q2 = self.rotation_matrix_to_quaternion(R2)
        q_interp = self.quaternion_slerp(q1, q2, t)
        R_interp = self.quaternion_to_rotation_matrix(
            q_interp[0], q_interp[1], q_interp[2], q_interp[3])

        # Interpolate translation linearly
        t_interp = (1 - t) * t1 + t * t2

        # Build interpolated pose
        pose_interp = np.eye(4, dtype=np.float32)
        pose_interp[:3, :3] = R_interp
        pose_interp[:3, 3] = t_interp

        return pose_interp

    def find_pose_bounds(self, target_ts, pose_timestamps, max_diff=0.1):
        """
        Find two poses that bound the target timestamp for interpolation.

        Args:
            target_ts: Target timestamp
            pose_timestamps: Sorted list of pose timestamps
            max_diff: Maximum allowed time difference (seconds)

        Returns:
            (pose1_ts, pose1, pose2_ts, pose2, t) or (None, None, None, None, None) if not found
            t is the interpolation parameter [0, 1]
        """
        if len(pose_timestamps) == 0:
            return None, None, None, None, None

        # Find the two timestamps that bound target_ts
        pose_ts_array = np.array(pose_timestamps)

        # Find index where target_ts would be inserted
        idx = np.searchsorted(pose_ts_array, target_ts)

        if idx == 0:
            # Target is before all poses, use first pose
            if abs(pose_ts_array[0] - target_ts) <= max_diff:
                return pose_timestamps[0], self.gt_poses[pose_timestamps[0]
                                                         ], None, None, 0.0
            return None, None, None, None, None
        elif idx >= len(pose_timestamps):
            # Target is after all poses, use last pose
            if abs(pose_ts_array[-1] - target_ts) <= max_diff:
                return pose_timestamps[-1], self.gt_poses[pose_timestamps[-1]
                                                          ], None, None, 0.0
            return None, None, None, None, None
        else:
            # Target is between two poses
            ts1 = pose_timestamps[idx - 1]
            ts2 = pose_timestamps[idx]

            # Check if both poses are within max_diff
            if abs(
                    ts1 -
                    target_ts) > max_diff and abs(
                    ts2 -
                    target_ts) > max_diff:
                return None, None, None, None, None

            pose1 = self.gt_poses[ts1]
            pose2 = self.gt_poses[ts2]

            # Calculate interpolation parameter
            if abs(ts2 - ts1) < 1e-8:
                t = 0.0
            else:
                t = (target_ts - ts1) / (ts2 - ts1)

            return ts1, pose1, ts2, pose2, t

    def load_poses(self, gt_path):
        """
        Load poses from groundtruth.txt file.

        Format: timestamp tx ty tz qx qy qz qw
        Returns: dict mapping timestamp to c2w pose matrix
        """
        self.gt_poses = {}  # timestamp -> c2w matrix

        if not gt_path.exists():
            print(f"Warning: groundtruth.txt not found at {gt_path}")
            return

        with open(gt_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                parts = line.split()
                if len(parts) < 8:
                    continue

                try:
                    timestamp = float(parts[0])
                    tx, ty, tz = float(
                        parts[1]), float(
                        parts[2]), float(
                        parts[3])
                    qx, qy, qz, qw = float(
                        parts[4]), float(
                        parts[5]), float(
                        parts[6]), float(
                        parts[7])

                    # Convert quaternion to rotation matrix
                    R = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)

                    # Build c2w matrix (camera-to-world)
                    c2w = np.eye(4, dtype=np.float32)
                    c2w[:3, :3] = R
                    c2w[:3, 3] = [tx, ty, tz]

                    self.gt_poses[timestamp] = c2w
                except (ValueError, IndexError) as e:
                    continue

        print(f"Loaded {len(self.gt_poses)} poses from groundtruth.txt")

    def find_closest_timestamp(self, target_ts, timestamp_list, max_diff=0.05):
        """
        Find closest timestamp in timestamp_list to target_ts.

        Args:
            target_ts: Target timestamp
            timestamp_list: List of timestamps to search
            max_diff: Maximum allowed time difference (seconds)

        Returns:
            (closest_ts, index) or (None, None) if no match found
        """
        if len(timestamp_list) == 0:
            return None, None

        # Convert to numpy array for efficient search
        ts_array = np.array(timestamp_list)
        diffs = np.abs(ts_array - target_ts)
        min_idx = np.argmin(diffs)
        min_diff = diffs[min_idx]

        if min_diff <= max_diff:
            return timestamp_list[min_idx], min_idx
        return None, None

    def match_timestamps(self):
        """
        Match RGB, depth, and pose timestamps.

        Creates aligned lists where each index corresponds to a matched frame.
        """

        # Extract timestamps from file names
        def extract_timestamp(path):
            name = os.path.splitext(os.path.basename(str(path)))[0]
            try:
                return float(name)
            except ValueError:
                return None

        rgb_timestamps = [extract_timestamp(p) for p in self.color_paths]
        depth_timestamps = [extract_timestamp(p) for p in self.depth_paths]
        pose_timestamps = sorted(self.gt_poses.keys())

        # Filter out None timestamps and create index mapping
        valid_depth_timestamps = [
            (i, ts) for i, ts in enumerate(depth_timestamps) if ts is not None]
        depth_ts_to_idx = {ts: idx for idx, ts in valid_depth_timestamps}
        valid_depth_ts_list = [ts for _, ts in valid_depth_timestamps]

        # Match RGB and depth timestamps (use associate.txt if available,
        # otherwise match by closest timestamp)
        associate_path = self.dataset_path / "associate.txt"
        matched_indices = []

        if associate_path.exists():
            # Use associate.txt for RGB-depth matching
            rgb_to_depth = {}
            with open(associate_path, "r") as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        try:
                            rgb_ts = float(parts[0])
                            rgb_file = parts[1]
                            depth_ts = float(parts[2])
                            depth_file = parts[3]
                            rgb_to_depth[rgb_ts] = depth_ts
                        except ValueError:
                            continue

            # Match RGB frames to depth and pose
            for i, rgb_ts in enumerate(rgb_timestamps):
                if rgb_ts is None:
                    continue

                # Find matching depth timestamp
                if rgb_ts in rgb_to_depth:
                    depth_ts = rgb_to_depth[rgb_ts]
                    depth_idx = depth_ts_to_idx.get(depth_ts, None)
                    if depth_idx is None:
                        # Fallback to closest timestamp search
                        depth_ts, depth_idx = self.find_closest_timestamp(
                            rgb_ts, valid_depth_ts_list)
                        if depth_idx is not None:
                            # Map back to original index
                            depth_idx = depth_ts_to_idx[depth_ts]
                else:
                    depth_ts, depth_idx = self.find_closest_timestamp(
                        rgb_ts, valid_depth_ts_list)
                    if depth_idx is not None:
                        # Map back to original index
                        depth_idx = depth_ts_to_idx[depth_ts]

                # Find pose bounds for interpolation
                ts1, pose1, ts2, pose2, t = self.find_pose_bounds(
                    rgb_ts, pose_timestamps)

                if depth_idx is not None and pose1 is not None:
                    if pose2 is not None and t is not None:
                        # Interpolate pose
                        interpolated_pose = self.interpolate_pose(
                            pose1, pose2, t)
                        matched_indices.append({
                            'rgb_idx': i,
                            'depth_idx': depth_idx,
                            'pose': interpolated_pose
                        })
                    else:
                        # Use nearest pose (within bounds)
                        matched_indices.append({
                            'rgb_idx': i,
                            'depth_idx': depth_idx,
                            'pose': pose1
                        })
        else:
            # Match by closest timestamp
            for i, rgb_ts in enumerate(rgb_timestamps):
                if rgb_ts is None:
                    continue

                # Find closest depth timestamp
                depth_ts, depth_idx = self.find_closest_timestamp(
                    rgb_ts, valid_depth_ts_list)
                if depth_idx is not None:
                    # Map back to original index
                    depth_idx = depth_ts_to_idx[depth_ts]

                # Find pose bounds for interpolation
                ts1, pose1, ts2, pose2, t = self.find_pose_bounds(
                    rgb_ts, pose_timestamps)

                if depth_idx is not None and pose1 is not None:
                    if pose2 is not None and t is not None:
                        # Interpolate pose
                        interpolated_pose = self.interpolate_pose(
                            pose1, pose2, t)
                        matched_indices.append({
                            'rgb_idx': i,
                            'depth_idx': depth_idx,
                            'pose': interpolated_pose
                        })
                    else:
                        # Use nearest pose (within bounds)
                        matched_indices.append({
                            'rgb_idx': i,
                            'depth_idx': depth_idx,
                            'pose': pose1
                        })

        # Create aligned lists
        self.matched_color_paths = []
        self.matched_depth_paths = []
        self.matched_poses = []

        for match in matched_indices:
            self.matched_color_paths.append(self.color_paths[match['rgb_idx']])
            self.matched_depth_paths.append(
                self.depth_paths[match['depth_idx']])
            self.matched_poses.append(match['pose'])

        print(
            f"Matched {len(self.matched_color_paths)} frames (RGB-Depth-Pose)")

        # Save frame_id to timestamp mapping for evaluation
        frame_timestamp_map_path = self.dataset_path / "frame_timestamp_map.txt"
        with open(frame_timestamp_map_path, "w") as f:
            f.write("# frame_id timestamp\n")
            for frame_id, match in enumerate(matched_indices):
                rgb_ts = rgb_timestamps[match['rgb_idx']]
                if rgb_ts is not None:
                    f.write(f"{frame_id} {rgb_ts}\n")
        print(
            f"Saved frame_id to timestamp mapping to {frame_timestamp_map_path}")

        # Update color_paths and depth_paths to matched versions
        self.color_paths = self.matched_color_paths
        self.depth_paths = self.matched_depth_paths
        self.poses = self.matched_poses

    def __getitem__(self, index):
        color_data = cv2.imread(str(self.color_paths[index]))
        if self.use_remap and self.map1 is not None and self.map2 is not None:
            color_data = cv2.remap(
                color_data,
                self.map1,
                self.map2,
                interpolation=cv2.INTER_LINEAR)
        color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)
        # # save undistored image
        # undistor_image_dir = self.dataset_path / "undistorted_color"
        # undistor_image_dir.mkdir(exist_ok=True)
        # undistorted_image_path = undistor_image_dir / os.path.basename(str(self.color_paths[index]))
        # cv2.imwrite(str(undistorted_image_path), cv2.cvtColor(color_data, cv2.COLOR_RGB2BGR))

        using_override = hasattr(
            self, 'depth_overrides') and index in self.depth_overrides
        if using_override:
            depth_data = self.depth_overrides[index]
            if not isinstance(depth_data, np.ndarray):
                depth_data = np.array(depth_data)
            depth_data = depth_data.astype(np.float32)
            self.depth_th = 10.0
        else:
            depth_data = cv2.imread(
                str(self.depth_paths[index]), cv2.IMREAD_UNCHANGED)
            depth_data = depth_data.astype(np.float32) / self.depth_scale

        if self.depth_th is not None:
            depth_data[depth_data > self.depth_th] = 0

        lr_color_data = cv2.resize(
            color_data, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
        if not using_override:
            depth_data = cv2.resize(
                depth_data, (self.width, self.height), interpolation=cv2.INTER_NEAREST)

        return index, lr_color_data, depth_data, self.poses[index]


class Pocket(BaseDataset):
    def __init__(self, dataset_config: dict):
        super().__init__(dataset_config)

        def _name_key(x):
            name = os.path.splitext(os.path.basename(str(x)))[0]
            try:
                return float(name)
            except ValueError:
                return name

        # Load RGB and depth images sorted by timestamp
        self.color_paths = sorted(
            list((self.dataset_path / "images").glob("*.jpg")) +
            list((self.dataset_path / "images").glob("*.png")),
            key=_name_key
        )
        self.depth_paths = sorted(
            list((self.dataset_path / "depth").glob("*.png")) +
            list((self.dataset_path / "depth").glob("*.jpg")),
            key=_name_key
        )
        print(f"Loaded {len(self.color_paths)} color frames")
        print(f"Loaded {len(self.depth_paths)} depth frames")
        self.time_stamps = []
        self.time_stamps = [
            float(
                os.path.splitext(
                    os.path.basename(
                        str(path)))[0]) for path in self.color_paths]

        # Load poses from groundtruth.txt and match timestamps
        self.load_poses(self.dataset_path / "cam_pos.txt")

        # Match RGB, depth, and pose timestamps
        self.match_timestamps()

        depth_th = dataset_config.get("depth_th", 0)
        if depth_th > 0:
            self.depth_th = depth_th
        else:
            self.depth_th = None

        # 预先计算remap映射表（如果有畸变参数）
        self.use_remap = self.distortion is not None
        self.map1 = None
        self.map2 = None
        if self.use_remap:
            # 假设所有图片shape一致，取第一张图片shape
            if len(self.color_paths) > 0:
                sample_img = cv2.imread(str(self.color_paths[0]))
                h, w = sample_img.shape[:2]
                self.map1, self.map2 = cv2.initUndistortRectifyMap(
                    self.original_intrinsics, self.distortion, None, self.original_intrinsics, (w, h), cv2.CV_16SC2)

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """Convert quaternion (qx, qy, qz, qw) to rotation matrix."""
        # Normalize quaternion
        norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-8:
            return np.eye(3)
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        # Convert to rotation matrix
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
             2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 *
             (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
             1 - 2 * (qx * qx + qy * qy)]
        ])
        return R

    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion (qx, qy, qz, qw)."""
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] -
                            R[2, 2]) * 2  # s = 4 * qx
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] -
                            R[2, 2]) * 2  # s = 4 * qy
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] -
                            R[1, 1]) * 2  # s = 4 * qz
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s

        # Normalize
        norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm > 1e-8:
            qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        return qx, qy, qz, qw

    def quaternion_slerp(self, q1, q2, t):
        """
        Spherical linear interpolation (SLERP) between two quaternions.

        Args:
            q1: First quaternion (qx, qy, qz, qw)
            q2: Second quaternion (qx, qy, qz, qw)
            t: Interpolation parameter [0, 1]

        Returns:
            Interpolated quaternion (qx, qy, qz, qw)
        """
        q1 = np.array(q1)
        q2 = np.array(q2)

        # Normalize quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)

        # Compute dot product
        dot = np.dot(q1, q2)

        # If dot product is negative, negate one quaternion to take shorter
        # path
        if dot < 0.0:
            q2 = -q2
            dot = -dot

        # If quaternions are very close, use linear interpolation
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)

        # Calculate angle between quaternions
        theta_0 = np.arccos(np.abs(dot))
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)

        # SLERP formula
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return (s0 * q1 + s1 * q2) / np.linalg.norm(s0 * q1 + s1 * q2)

    def interpolate_pose(self, pose1, pose2, t):
        """
        Interpolate between two poses.

        Args:
            pose1: First pose (4x4 c2w matrix)
            pose2: Second pose (4x4 c2w matrix)
            t: Interpolation parameter [0, 1]

        Returns:
            Interpolated pose (4x4 c2w matrix)
        """
        # Extract rotation and translation
        R1 = pose1[:3, :3]
        t1 = pose1[:3, 3]
        R2 = pose2[:3, :3]
        t2 = pose2[:3, 3]

        # Interpolate rotation using SLERP
        q1 = self.rotation_matrix_to_quaternion(R1)
        q2 = self.rotation_matrix_to_quaternion(R2)
        q_interp = self.quaternion_slerp(q1, q2, t)
        R_interp = self.quaternion_to_rotation_matrix(
            q_interp[0], q_interp[1], q_interp[2], q_interp[3])

        # Interpolate translation linearly
        t_interp = (1 - t) * t1 + t * t2

        # Build interpolated pose
        pose_interp = np.eye(4, dtype=np.float32)
        pose_interp[:3, :3] = R_interp
        pose_interp[:3, 3] = t_interp

        return pose_interp

    def find_pose_bounds(self, target_ts, pose_timestamps, max_diff=0.1):
        """
        Find two poses that bound the target timestamp for interpolation.

        Args:
            target_ts: Target timestamp
            pose_timestamps: Sorted list of pose timestamps
            max_diff: Maximum allowed time difference (seconds)

        Returns:
            (pose1_ts, pose1, pose2_ts, pose2, t) or (None, None, None, None, None) if not found
            t is the interpolation parameter [0, 1]
        """
        if len(pose_timestamps) == 0:
            return None, None, None, None, None

        # Find the two timestamps that bound target_ts
        pose_ts_array = np.array(pose_timestamps)

        # Find index where target_ts would be inserted
        idx = np.searchsorted(pose_ts_array, target_ts)

        if idx == 0:
            # Target is before all poses, use first pose
            if abs(pose_ts_array[0] - target_ts) <= max_diff:
                return pose_timestamps[0], self.gt_poses[pose_timestamps[0]
                                                         ], None, None, 0.0
            return None, None, None, None, None
        elif idx >= len(pose_timestamps):
            # Target is after all poses, use last pose
            if abs(pose_ts_array[-1] - target_ts) <= max_diff:
                return pose_timestamps[-1], self.gt_poses[pose_timestamps[-1]
                                                          ], None, None, 0.0
            return None, None, None, None, None
        else:
            # Target is between two poses
            ts1 = pose_timestamps[idx - 1]
            ts2 = pose_timestamps[idx]

            # Check if both poses are within max_diff
            if abs(
                    ts1 -
                    target_ts) > max_diff and abs(
                    ts2 -
                    target_ts) > max_diff:
                return None, None, None, None, None

            pose1 = self.gt_poses[ts1]
            pose2 = self.gt_poses[ts2]

            # Calculate interpolation parameter
            if abs(ts2 - ts1) < 1e-8:
                t = 0.0
            else:
                t = (target_ts - ts1) / (ts2 - ts1)

            return ts1, pose1, ts2, pose2, t

    def load_poses(self, gt_path):
        """
        Load poses from groundtruth.txt file.

        Format: timestamp tx ty tz qx qy qz qw
        Returns: dict mapping timestamp to c2w pose matrix
        """
        self.gt_poses = {}  # timestamp -> c2w matrix

        if not gt_path.exists():
            print(f"Warning: groundtruth.txt not found at {gt_path}")
            return

        with open(gt_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                parts = line.split()
                if len(parts) < 8:
                    continue

                try:
                    timestamp = float(parts[0])
                    tx, ty, tz = float(
                        parts[1]), float(
                        parts[2]), float(
                        parts[3])
                    qx, qy, qz, qw = float(
                        parts[4]), float(
                        parts[5]), float(
                        parts[6]), float(
                        parts[7])

                    # Convert quaternion to rotation matrix
                    R = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)

                    # Build c2w matrix (camera-to-world)
                    c2w = np.eye(4, dtype=np.float32)
                    c2w[:3, :3] = R
                    c2w[:3, 3] = [tx, ty, tz]
                    T_switch_axis = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [
                        0, -1, 0, 0], [0, 0, 0, 1]], dtype=np.float32)
                    c2w = T_switch_axis @ c2w
                    self.gt_poses[timestamp] = c2w
                except (ValueError, IndexError) as e:
                    continue

        print(f"Loaded {len(self.gt_poses)} poses from groundtruth.txt")

    def find_closest_timestamp(self, target_ts, timestamp_list, max_diff=0.05):
        """
        Find closest timestamp in timestamp_list to target_ts.

        Args:
            target_ts: Target timestamp
            timestamp_list: List of timestamps to search
            max_diff: Maximum allowed time difference (seconds)

        Returns:
            (closest_ts, index) or (None, None) if no match found
        """
        if len(timestamp_list) == 0:
            return None, None

        # Convert to numpy array for efficient search
        ts_array = np.array(timestamp_list)
        diffs = np.abs(ts_array - target_ts)
        min_idx = np.argmin(diffs)
        min_diff = diffs[min_idx]

        if min_diff <= max_diff:
            return timestamp_list[min_idx], min_idx
        return None, None

    def match_timestamps(self):
        """
        Match RGB, depth, and pose timestamps.

        Creates aligned lists where each index corresponds to a matched frame.
        """

        # Extract timestamps from file names
        def extract_timestamp(path):
            name = os.path.splitext(os.path.basename(str(path)))[0]
            try:
                return float(name)
            except ValueError:
                return None

        rgb_timestamps = [extract_timestamp(p) for p in self.color_paths]
        depth_timestamps = [extract_timestamp(p) for p in self.depth_paths]
        pose_timestamps = sorted(self.gt_poses.keys())

        # Filter out None timestamps and create index mapping
        valid_depth_timestamps = [
            (i, ts) for i, ts in enumerate(depth_timestamps) if ts is not None]
        depth_ts_to_idx = {ts: idx for idx, ts in valid_depth_timestamps}
        valid_depth_ts_list = [ts for _, ts in valid_depth_timestamps]

        # Match RGB and depth timestamps (use associate.txt if available,
        # otherwise match by closest timestamp)
        associate_path = self.dataset_path / "associate.txt"
        matched_indices = []

        if associate_path.exists():
            # Use associate.txt for RGB-depth matching
            rgb_to_depth = {}
            with open(associate_path, "r") as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        try:
                            rgb_ts = float(parts[0])
                            rgb_file = parts[1]
                            depth_ts = float(parts[2])
                            depth_file = parts[3]
                            rgb_to_depth[rgb_ts] = depth_ts
                        except ValueError:
                            continue

            # Match RGB frames to depth and pose
            for i, rgb_ts in enumerate(rgb_timestamps):
                if rgb_ts is None:
                    continue

                # Find matching depth timestamp
                if rgb_ts in rgb_to_depth:
                    depth_ts = rgb_to_depth[rgb_ts]
                    depth_idx = depth_ts_to_idx.get(depth_ts, None)
                    if depth_idx is None:
                        # Fallback to closest timestamp search
                        depth_ts, depth_idx = self.find_closest_timestamp(
                            rgb_ts, valid_depth_ts_list)
                        if depth_idx is not None:
                            # Map back to original index
                            depth_idx = depth_ts_to_idx[depth_ts]
                else:
                    depth_ts, depth_idx = self.find_closest_timestamp(
                        rgb_ts, valid_depth_ts_list)
                    if depth_idx is not None:
                        # Map back to original index
                        depth_idx = depth_ts_to_idx[depth_ts]

                # Find pose bounds for interpolation
                ts1, pose1, ts2, pose2, t = self.find_pose_bounds(
                    rgb_ts, pose_timestamps)

                if depth_idx is not None and pose1 is not None:
                    if pose2 is not None and t is not None:
                        # Interpolate pose
                        interpolated_pose = self.interpolate_pose(
                            pose1, pose2, t)
                        matched_indices.append({
                            'rgb_idx': i,
                            'depth_idx': depth_idx,
                            'pose': interpolated_pose
                        })
                    else:
                        # Use nearest pose (within bounds)
                        matched_indices.append({
                            'rgb_idx': i,
                            'depth_idx': depth_idx,
                            'pose': pose1
                        })
        else:
            # Match by closest timestamp
            for i, rgb_ts in enumerate(rgb_timestamps):
                if rgb_ts is None:
                    continue

                # Find closest depth timestamp
                depth_ts, depth_idx = self.find_closest_timestamp(
                    rgb_ts, valid_depth_ts_list)
                if depth_idx is not None:
                    # Map back to original index
                    depth_idx = depth_ts_to_idx[depth_ts]

                # Find pose bounds for interpolation
                ts1, pose1, ts2, pose2, t = self.find_pose_bounds(
                    rgb_ts, pose_timestamps)

                if depth_idx is not None and pose1 is not None:
                    if pose2 is not None and t is not None:
                        # Interpolate pose
                        interpolated_pose = self.interpolate_pose(
                            pose1, pose2, t)
                        matched_indices.append({
                            'rgb_idx': i,
                            'depth_idx': depth_idx,
                            'pose': interpolated_pose
                        })
                    else:
                        # Use nearest pose (within bounds)
                        matched_indices.append({
                            'rgb_idx': i,
                            'depth_idx': depth_idx,
                            'pose': pose1
                        })

        # Create aligned lists
        self.matched_color_paths = []
        self.matched_depth_paths = []
        self.matched_poses = []

        for match in matched_indices:
            self.matched_color_paths.append(self.color_paths[match['rgb_idx']])
            self.matched_depth_paths.append(
                self.depth_paths[match['depth_idx']])
            self.matched_poses.append(match['pose'])

        print(
            f"Matched {len(self.matched_color_paths)} frames (RGB-Depth-Pose)")

        # Save frame_id to timestamp mapping for evaluation
        frame_timestamp_map_path = self.dataset_path / "frame_timestamp_map.txt"
        with open(frame_timestamp_map_path, "w") as f:
            f.write("# frame_id timestamp\n")
            for frame_id, match in enumerate(matched_indices):
                rgb_ts = rgb_timestamps[match['rgb_idx']]
                if rgb_ts is not None:
                    f.write(f"{frame_id} {rgb_ts}\n")
        print(
            f"Saved frame_id to timestamp mapping to {frame_timestamp_map_path}")

        # Update color_paths and depth_paths to matched versions
        self.color_paths = self.matched_color_paths
        self.depth_paths = self.matched_depth_paths
        self.poses = self.matched_poses

    def __getitem__(self, index):
        color_data = cv2.imread(str(self.color_paths[index]))
        if self.use_remap and self.map1 is not None and self.map2 is not None:
            color_data = cv2.remap(
                color_data,
                self.map1,
                self.map2,
                interpolation=cv2.INTER_LINEAR)
        color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)
        # # save undistored image
        # undistor_image_dir = self.dataset_path / "undistorted_color"
        # undistor_image_dir.mkdir(exist_ok=True)
        # undistorted_image_path = undistor_image_dir / os.path.basename(str(self.color_paths[index]))
        # cv2.imwrite(str(undistorted_image_path), cv2.cvtColor(color_data, cv2.COLOR_RGB2BGR))

        using_override = hasattr(
            self, 'depth_overrides') and index in self.depth_overrides
        if using_override:
            depth_data = self.depth_overrides[index]
            if not isinstance(depth_data, np.ndarray):
                depth_data = np.array(depth_data)
            depth_data = depth_data.astype(np.float32)
            self.depth_th = 10.0
        else:
            depth_data = cv2.imread(
                str(self.depth_paths[index]), cv2.IMREAD_UNCHANGED)
            depth_data = depth_data.astype(np.float32) / self.depth_scale

        if self.depth_th is not None:
            depth_data[depth_data > self.depth_th] = 0

        lr_color_data = cv2.resize(
            color_data, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
        if not using_override:
            depth_data = cv2.resize(
                depth_data, (self.width, self.height), interpolation=cv2.INTER_NEAREST)

        return index, lr_color_data, depth_data, self.poses[index]


class ScanNetPP(BaseDataset):
    def __init__(self, dataset_config: dict):
        super().__init__(dataset_config)
        self.use_train_split = dataset_config["use_train_split"]
        self.train_test_split = json.load(
            open(f"{self.dataset_path}/dslr/train_test_lists.json", "r"))
        if self.use_train_split:
            self.image_names = self.train_test_split["train"]
        else:
            self.image_names = self.train_test_split["test"]
        self.load_data()

    def load_data(self):
        self.poses = []
        cams_path = self.dataset_path / "dslr" /\
            "nerfstudio" / "transforms_undistorted.json"
        cams_metadata = json.load(open(str(cams_path), "r"))
        frames_key = "frames" if self.use_train_split else "test_frames"
        frames_metadata = cams_metadata[frames_key]
        frame2idx = {
            frame["file_path"]: index for index,
            frame in enumerate(frames_metadata)}
        P = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0],
                     [0, 0, 0, 1]]).astype(np.float32)
        for image_name in self.image_names:
            frame_metadata = frames_metadata[frame2idx[image_name]]
            # if self.ignore_bad and frame_metadata['is_bad']:
            #     continue
            color_path = str(
                self.dataset_path /
                "dslr" /
                "undistorted_images" /
                image_name)
            depth_path = str(
                self.dataset_path /
                "dslr" /
                "undistorted_projected_depth" /
                image_name.replace(
                    '.JPG',
                    '.png'))
            self.color_paths.append(color_path)
            self.depth_paths.append(depth_path)
            c2w = np.array(
                frame_metadata["transform_matrix"]).astype(
                np.float32)
            c2w = P @ c2w @ P.T
            self.poses.append(c2w)

    def __len__(self):
        if self.use_train_split:
            return len(
                self.image_names) if self.frame_limit < 0 else int(
                self.frame_limit)
        else:
            return len(self.image_names)

    def __getitem__(self, index):
        color_data = np.asarray(
            imageio.imread(
                self.color_paths[index]),
            dtype=float)
        color_data = cv2.resize(
            color_data,
            (self.width,
             self.height),
            interpolation=cv2.INTER_LINEAR)
        color_data = color_data.astype(np.uint8)

        if hasattr(self, 'depth_overrides') and index in self.depth_overrides:
            depth_data = self.depth_overrides[index]
        else:
            depth_data = np.asarray(
                imageio.imread(
                    self.depth_paths[index]),
                dtype=np.int64)
            depth_data = cv2.resize(
                depth_data.astype(float),
                (self.width,
                 self.height),
                interpolation=cv2.INTER_NEAREST)
            depth_data = depth_data.astype(np.float32) / self.depth_scale
        return index, color_data, depth_data, self.poses[index]


class HM3D(BaseDataset):
    def __init__(self, dataset_config: dict):
        super().__init__(dataset_config)

        def _name_key(x):
            name = os.path.splitext(os.path.basename(str(x)))[0]
            # try:
            #     return float(name)
            # except ValueError:
            return name

        self.color_paths = sorted(
            list((self.dataset_path / "rgb").glob("*.jpg")) +
            list((self.dataset_path / "rgb").glob("*.png")),
            key=_name_key
        )
        self.depth_paths = sorted(
            list((self.dataset_path / "depth").glob("*.png")) +
            list((self.dataset_path / "depth").glob("*.jpg")),
            key=_name_key
        )
        self.time_stamps = []

        self.time_stamps = [
            float(
                re.search(
                    r'(\d{6})',
                    os.path.splitext(
                        os.path.basename(
                            str(path)))[0]).group()) for path in self.color_paths]

        print(f"Loaded {len(self.color_paths)} color frames")
        print(f"Loaded {len(self.depth_paths)} depth frames")
        print("color_paths")
        self.load_poses(self.dataset_path / "pose")
        depth_th = dataset_config.get("depth_th", 0)
        if depth_th > 0:
            self.depth_th = depth_th
        else:
            self.depth_th = None

    def load_poses(self, path):
        self.poses = []

        def _name_key(x):
            name = os.path.splitext(os.path.basename(str(x)))[0]
            try:
                return int(name)
            except ValueError:
                try:
                    return float(name)
                except ValueError:
                    return name

        pose_paths = sorted(path.glob('*.txt'),
                            key=_name_key)
        for pose_path in pose_paths:
            with open(pose_path, "r") as f:
                lines = f.readline().strip()
                values = lines.split()
                values = [float(val) for val in values]
                c2w = np.array(values).reshape(4, 4).astype(np.float32)
                T_switch_axis = np.array(
                    [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]], dtype=np.float32)
                c2w = c2w @ T_switch_axis
                self.poses.append(c2w)

    def __getitem__(self, index):
        color_data = cv2.imread(str(self.color_paths[index]))
        if self.distortion is not None:
            color_data = cv2.undistort(
                color_data, self.intrinsics, self.distortion)
        color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)

        using_override = hasattr(
            self, 'depth_overrides') and index in self.depth_overrides
        if using_override:
            depth_data = self.depth_overrides[index]
            if not isinstance(depth_data, np.ndarray):
                depth_data = np.array(depth_data)
            depth_data = depth_data.astype(np.float32)
            self.depth_th = 20.0
        else:
            depth_data = cv2.imread(
                str(self.depth_paths[index]), cv2.IMREAD_UNCHANGED)
            depth_data = depth_data.astype(np.float32) / self.depth_scale

        if self.depth_th is not None:
            depth_data[depth_data > self.depth_th] = 0

        edge = self.crop_edge
        if edge > 0:
            lr_color_data = color_data[edge:-edge, edge:-edge]
            if not using_override:
                depth_data = depth_data[edge:-edge, edge:-edge]
        else:
            lr_color_data = color_data

        lr_color_data = cv2.resize(
            lr_color_data,
            (self.width,
             self.height),
            interpolation=cv2.INTER_LINEAR)
        if not using_override:
            depth_data = cv2.resize(
                depth_data, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
        # Interpolate depth values for splatting
        return index, lr_color_data, depth_data, self.poses[index]


def get_dataset(dataset_name: str):
    if dataset_name == "replica":
        return Replica
    elif dataset_name == "scannet":
        return ScanNet
    elif dataset_name == "scannetpp":
        return ScanNetPP
    elif dataset_name == "tum":
        return TUM
    elif dataset_name == "g1" or dataset_name == "go2":
        return G1
    elif dataset_name == "pocket":
        return Pocket
    elif dataset_name == "hm3d":
        return HM3D
    raise NotImplementedError(f"Dataset {dataset_name} not implemented")
