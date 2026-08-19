#!/usr/bin/env python3
"""Build a real OVO/HMSG semantic map from the live Isaac RGB-D-pose frame."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation


WORKSPACE = Path(__file__).resolve().parents[4]
FSR_ROOT = Path(__file__).resolve().parents[1]
for path in (WORKSPACE, FSR_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from tools.shared_memory_utils import MultiImageReader
from ovo.entities.obj_detect_track import ObjDetectTrack
from ovo.integration.hmsg_bridge import build_hmsg_from_ovo_output
from ovo.slam.vanilla_mapper import VanillaMapper


def quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError("Invalid camera quaternion")
    w, x, y, z = quaternion / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


def read_synchronized_frame() -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    reader = MultiImageReader()
    try:
        for _ in range(20):
            bgr = reader.read_single_image("head")
            depth = reader.read_single_image("head_depth")
            poses = reader.read_single_image("head_pose")
            timestamps = [reader.last_timestamps.get(name, 0) for name in (
                "head", "head_depth", "head_pose")]
            if all(value is not None for value in (bgr, depth, poses)) and len(set(timestamps)) == 1:
                break
        else:
            raise RuntimeError("No synchronized Isaac RGB-D-pose frame available")
    finally:
        reader.close()

    if bgr.shape != (480, 640, 3) or depth.shape != (480, 640) or poses.shape != (2, 7):
        raise ValueError(
            f"Unexpected Isaac frame shapes: rgb={bgr.shape}, depth={depth.shape}, poses={poses.shape}")
    if not np.isfinite(depth).all() or not np.isfinite(poses).all():
        raise ValueError("Isaac RGB-D-pose contains non-finite values")
    return cv2.cvtColor(bgr.copy(), cv2.COLOR_BGR2RGB), depth.copy(), poses.copy(), timestamps[0]


def camera_to_world(camera_pose: np.ndarray) -> np.ndarray:
    ros_camera_pose = np.eye(4, dtype=np.float32)
    ros_camera_pose[:3, :3] = quaternion_matrix(camera_pose[3:])
    ros_camera_pose[:3, 3] = camera_pose[:3]
    optical_to_ros = np.array([
        [0, 0, 1, 0],
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1],
    ], dtype=np.float32)
    return ros_camera_pose @ optical_to_ros


def write_horizon_frame(
        output: Path, rgb: np.ndarray, depth: np.ndarray, c2w: np.ndarray) -> None:
    images_dir = output / "images"
    depth_dir = output / "depth"
    images_dir.mkdir(exist_ok=True)
    depth_dir.mkdir(exist_ok=True)
    if not cv2.imwrite(str(images_dir / "00000.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
        raise OSError("Failed to save the live RGB frame")
    depth_mm = np.rint(depth * 1000.0).clip(0, 65535).astype(np.uint16)
    if not cv2.imwrite(str(depth_dir / "00000.png"), depth_mm):
        raise OSError("Failed to save the live depth frame")
    camera_config = {
        "Camera1.fx": 243.2,
        "Camera1.fy": 243.2,
        "Camera1.cx": 319.5,
        "Camera1.cy": 239.5,
        "Camera.width": 640,
        "Camera.height": 480,
    }
    with (output / "d435i.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(camera_config, stream, sort_keys=False)
    map_to_hmsg = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1],
    ], dtype=np.float32)
    w2c = np.linalg.inv(map_to_hmsg @ c2w)
    qx, qy, qz, qw = Rotation.from_matrix(w2c[:3, :3]).as_quat()
    tx, ty, tz = w2c[:3, 3]
    with (output / "poses.txt").open("w", encoding="utf-8") as stream:
        stream.write(
            f"0 {tx:.9f} {ty:.9f} {tz:.9f} "
            f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")


def write_mask_overlay(
        output: Path,
        rgb: np.ndarray,
        instance_ids: list[int],
        binary_maps: torch.Tensor) -> list[int]:
    """Save the actual SAM3 masks over the captured Isaac RGB frame."""
    if binary_maps is None or len(instance_ids) != binary_maps.shape[0]:
        raise RuntimeError("SAM3 masks and tracked instance ids do not match")
    overlay = rgb.copy()
    counts = []
    for index, instance_id in enumerate(instance_ids):
        mask = binary_maps[index].detach().cpu().numpy().astype(bool)
        if mask.shape != rgb.shape[:2] or not mask.any():
            raise RuntimeError(f"invalid SAM3 mask for instance {instance_id}")
        counts.append(int(mask.sum()))
        color = np.array(cv2.cvtColor(
            np.uint8([[[(instance_id * 47) % 180, 220, 245]]]),
            cv2.COLOR_HSV2RGB)[0, 0])
        overlay[mask] = (0.55 * color + 0.45 * overlay[mask]).astype(np.uint8)
        ys, xs = np.nonzero(mask)
        cv2.putText(
            overlay,
            f"id={instance_id}",
            (int(np.median(xs)), int(np.median(ys))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    if not cv2.imwrite(
            str(output / "object_masks_overlay.png"),
            cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)):
        raise OSError("Failed to save the SAM3 object mask overlay")
    return counts


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prompt", action="append", required=True)
    parser.add_argument("--sam3-checkpoint", type=Path, required=True)
    parser.add_argument("--siglip-snapshot", type=Path, required=True)
    parser.add_argument("--hmsg-config", type=Path, required=True)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=False)
    os.environ["SIGLIP_MODEL_PATH"] = str(args.siglip_snapshot)
    rgb, depth, poses, timestamp_ms = read_synchronized_frame()
    c2w = camera_to_world(poses[0])
    device = "cuda" if torch.cuda.is_available() else "cpu"
    intrinsics = torch.tensor([
        [243.2, 0.0, 319.5],
        [0.0, 243.2, 239.5],
        [0.0, 0.0, 1.0],
    ], dtype=torch.float32, device=device)
    config = {
        "device": device,
        "mapping": {"max_frame_points": 100000, "k_pooling": 1, "downscale_res": 2},
        "semantic": {
            "match_distance_th": 0.05,
            "track_th": 20,
            "depth_filter": False,
            "log": False,
            "kf_queue_delay": 0,
            "min_points_th": 10,
            "min_track_frames_th": 1,
            "sam": {
                "precomputed": False,
                "multi_crop": True,
                "mask_res": 384,
                "sam_version": "3",
                "sam_ckpt_path": str(args.sam3_checkpoint),
                "sam_prompts": args.prompt,
                "confidence_threshold": 0.5,
                "min_mask_region_area": 100,
                "nms_iou_th": 0.8,
                "nms_score_th": 0.25,
                "nms_inner_th": 0.5,
            },
            "clip": {
                "use_half": True,
                "embed_type": "fixed_weights",
                "model_card": "SigLIP-384",
                "model_path": str(args.siglip_snapshot),
                "k_top_views": 10,
            },
        },
    }

    mapper = VanillaMapper(config, intrinsics)
    frame = (0, rgb, depth, c2w, rgb)
    c2w_tensor = torch.from_numpy(c2w).to(device)
    mapper.track_camera(frame)
    mapper.map(frame, c2w_tensor)
    detector = ObjDetectTrack(
        config["semantic"], None, "isaaclab", None, intrinsics, device=device)
    updated_ids, tracked_ids, binary_maps = detector.detect_and_track_objects(
        [0, rgb, depth, ()], mapper.get_map(), c2w_tensor)
    if updated_ids is None or not tracked_ids:
        raise RuntimeError("SAM3/OVO produced no tracked 3D instances")
    mapper.update_pcd_obj_ids(updated_ids)
    mask_pixel_counts = write_mask_overlay(
        args.output, rgb, [int(value) for value in tracked_ids], binary_maps)
    detector.complete_semantic_info()
    features = detector.get_objs_clips()
    if features.shape != (len(detector.objects), 1152):
        raise RuntimeError(f"Unexpected SigLIP instance feature shape: {features.shape}")
    feature_norms = features.float().norm(dim=1)
    if not torch.isfinite(features).all() or (feature_norms < 0.9).any():
        raise RuntimeError("SigLIP produced a non-finite or zero instance feature")

    torch.save({
        "map_params": mapper.get_map_dict(),
        "ovo_map_params": detector.capture_dict(False),
    }, args.output / "ovo_map.ckpt")
    write_horizon_frame(args.output, rgb, depth, c2w)
    with (args.output / "config.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)

    metadata = {
        "source": "live_isaac_shared_memory",
        "source_timestamp_ms": timestamp_ms,
        "rgb_shape": list(rgb.shape),
        "depth_shape": list(depth.shape),
        "depth_range_m": [float(depth.min()), float(depth.max())],
        "camera_sim_pose_wxyz": poses[0].tolist(),
        "robot_sim_pose_wxyz": poses[1].tolist(),
        "prompts": args.prompt,
        "map_points": int(mapper.get_map()[0].shape[0]),
        "instance_ids": [int(value) for value in detector.objects],
        "mask_pixel_counts": mask_pixel_counts,
        "feature_shape": list(features.shape),
        "feature_norms": feature_norms.cpu().tolist(),
    }
    with (args.output / "runtime_evidence.json").open("w", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2)

    build_hmsg_from_ovo_output(
        scene_output_path=args.output,
        scene_input_path=args.output,
        dataset_name="G1",
        scene_name=args.output.name,
        hmsg_config_path=str(args.hmsg_config),
        obj_labels="FINALLABEL",
        min_object_points=10)
    print(json.dumps(metadata, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
