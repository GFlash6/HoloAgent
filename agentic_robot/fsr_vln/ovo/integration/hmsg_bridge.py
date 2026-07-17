from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

import numpy as np
import open3d as o3d
import torch
from omegaconf import OmegaConf

from memory.hmsg.graph.graph import Graph
from memory.hmsg.utils.graph_utils import pcd_denoise_dbscan


_SUPPORTED_CLIP_TYPES_BY_DIM = {
    512: "ViT-B-32",
    768: "ViT-L/14",
    1024: "ViT-H-14",
    1152: "SigLIP-384",
}

_DEFAULT_CLIP_CHECKPOINT_BY_TYPE = {
    "ViT-B-32": "laion2b_s34b_b79k",
    "ViT-L/14": "laion2b_s32b_b82k",
    "ViT-H-14": "laion2b_s32b_b79k",
    "SigLIP-384": "hf-hub:timm/ViT-SO400M-14-SigLIP-384",
}


def _to_numpy(x):
    if x is None:
        return None
    if isinstance(x, np.ndarray):
        return x
    if torch.is_tensor(x):
        return x.detach().cpu().numpy()
    return np.asarray(x)


def _normalize_feat(feat: np.ndarray, target_dim: int) -> np.ndarray:
    feat = np.asarray(feat).reshape(-1)
    feat = np.nan_to_num(feat)
    if feat.shape[0] == target_dim:
        return feat.astype(np.float32)
    out = np.zeros((target_dim,), dtype=np.float32)
    n_copy = min(target_dim, feat.shape[0])
    if n_copy > 0:
        out[:n_copy] = feat[:n_copy].astype(np.float32)
    return out


def _extract_instance_feat_map(ovo_map_params: Dict) -> Dict[int, np.ndarray]:
    feat_map: Dict[int, np.ndarray] = {}
    if not ovo_map_params:
        return feat_map

    ins_ids = _to_numpy(ovo_map_params.get("ins_3d_ids", np.array([])))
    if ins_ids is None:
        return feat_map

    for ins_id in ins_ids.tolist():
        key = f"ins3d_{int(ins_id)}_clip_feature"
        feat = ovo_map_params.get(key)
        if feat is None:
            continue
        feat_np = _to_numpy(feat)
        if feat_np is None:
            continue
        feat_map[int(ins_id)] = np.asarray(feat_np).reshape(-1)

    return feat_map


def _build_default_hmsg_cfg(
    dataset_name: str,
    scene_name: str,
    scene_input_path: Path,
    scene_output_path: Path,
    clip_type: str,
    clip_checkpoint: str,
    obj_labels: str,
):
    dataset = dataset_name.lower()
    return {
        "main": {
            "dataset": dataset,
            "scene_id": scene_name,
            "dataset_path": str(scene_input_path),
            "save_path": str(scene_output_path),
            "depth_cut": 10,
            "use_gpt": False,
            "skip_sam_init": True,
        },
        "models": {
            "clip": {
                "type": clip_type,
                "checkpoint": clip_checkpoint,
            },
            "sam": {
                "type": "vit_h",
                "checkpoint": "",
                "points_per_side": 16,
                "pred_iou_thresh": 0.86,
                "points_per_batch": 64,
                "stability_score_thresh": 0.92,
                "crop_n_layers": 1,
                "min_mask_region_area": 100,
            },
        },
        "pipeline": {
            "skip_frames": 10,
            "grid_resolution": 0.05,
            "merge_objects_graph": False,
            "save_intermediate_results": False,
            "obj_labels": obj_labels,
        },
    }


def build_hmsg_from_ovo_output(
    scene_output_path: str | Path,
    scene_input_path: str | Path,
    dataset_name: str,
    scene_name: str,
    hmsg_config_path: Optional[str] = None,
    obj_labels: str = "FINALLABEL",
    min_object_points: int = 10,
) -> str:
    scene_output_path = Path(scene_output_path)
    scene_input_path = Path(scene_input_path)

    ckpt_path = scene_output_path / "ovo_map.ckpt"
    if not ckpt_path.exists():
        raise FileNotFoundError(f"OVO checkpoint not found: {ckpt_path}")

    ckpt = torch.load(ckpt_path, map_location="cpu")
    map_params = ckpt.get("map_params", ckpt.get("gaussian_params", None))
    if map_params is None:
        raise ValueError(
            "Cannot find `map_params` (or `gaussian_params`) in ovo_map.ckpt")

    ovo_map_params = ckpt.get("ovo_map_params", {})

    xyz = _to_numpy(map_params.get("xyz"))
    obj_ids = _to_numpy(map_params.get("obj_ids"))
    colors = _to_numpy(map_params.get("color"))
    if xyz is None or obj_ids is None:
        raise ValueError("OVO map data is missing `xyz` or `obj_ids`")

    xyz = np.asarray(xyz, dtype=np.float32)
    obj_ids = np.asarray(obj_ids).reshape(-1).astype(np.int64)
    if xyz.shape[0] != obj_ids.shape[0]:
        raise ValueError(
            f"Shape mismatch: xyz={xyz.shape}, obj_ids={obj_ids.shape}")

    feat_map = _extract_instance_feat_map(ovo_map_params)

    inferred_dim = None
    for feat in feat_map.values():
        if feat is not None and feat.size > 0:
            inferred_dim = int(feat.reshape(-1).shape[0])
            break

    print(f"[HMSG] Inferred CLIP feature dim from OVO map: {inferred_dim}")
    if inferred_dim is not None and inferred_dim not in _SUPPORTED_CLIP_TYPES_BY_DIM:
        print(
            f"[HMSG][WARN] Unsupported feature dim {inferred_dim}, fallback to ViT-B-32 (512) with feature trunc/pad."
        )
    clip_type = _SUPPORTED_CLIP_TYPES_BY_DIM.get(inferred_dim, "SigLIP-384")
    clip_checkpoint = _DEFAULT_CLIP_CHECKPOINT_BY_TYPE[clip_type]

    base_cfg_dict = _build_default_hmsg_cfg(
        dataset_name=dataset_name,
        scene_name=scene_name,
        scene_input_path=scene_input_path,
        scene_output_path=scene_output_path,
        clip_type=clip_type,
        clip_checkpoint=clip_checkpoint,
        obj_labels=obj_labels,
    )
    if hmsg_config_path:
        user_cfg = OmegaConf.load(hmsg_config_path)
        cfg = OmegaConf.merge(OmegaConf.create(base_cfg_dict), user_cfg)
    else:
        cfg = OmegaConf.create(base_cfg_dict)

    cfg.main.dataset = dataset_name.lower()
    cfg.main.scene_id = scene_name
    cfg.main.dataset_path = str(scene_input_path)
    cfg.main.save_path = str(scene_output_path)
    cfg.main.skip_sam_init = True
    if not hasattr(cfg.main, "depth_cut"):
        cfg.main.depth_cut = 10

    hmsg = Graph(cfg)

    full_pcd = o3d.geometry.PointCloud()
    full_pcd.points = o3d.utility.Vector3dVector(xyz)
    if colors is not None and len(colors) == len(xyz):
        colors = np.asarray(colors, dtype=np.float32)
        if colors.max() > 1.0:
            colors = colors / 255.0
        full_pcd.colors = o3d.utility.Vector3dVector(np.clip(colors, 0.0, 1.0))
        # filter point cloud
    full_pcd = full_pcd.voxel_down_sample(voxel_size=cfg.pipeline.voxel_size)
    print("Original full_pcd has {} points".format(len(full_pcd.points)))
    full_pcd = pcd_denoise_dbscan(full_pcd, eps=0.01, min_points=100)
    print("After denoising, full_pcd has {} points".format(len(full_pcd.points)))
    _, inlier_idx = full_pcd.remove_radius_outlier(nb_points=1000, radius=1.0)
    inlier_cloud = full_pcd.select_by_index(inlier_idx)
    print("After radius outlier removal, full_pcd has {} points".format(
        len(inlier_cloud.points)))
    hmsg.full_pcd = inlier_cloud

    unique_obj_ids = sorted({int(x) for x in obj_ids.tolist() if int(x) >= 0})
    hmsg.mask_pcds = []
    hmsg.mask_feats = []

    for obj_id in unique_obj_ids:
        mask = obj_ids == obj_id
        if int(mask.sum()) < min_object_points:
            continue

        obj_xyz = xyz[mask]
        obj_pcd = o3d.geometry.PointCloud()
        obj_pcd.points = o3d.utility.Vector3dVector(obj_xyz)
        if colors is not None and len(colors) == len(xyz):
            obj_colors = np.asarray(colors[mask], dtype=np.float32)
            if obj_colors.max() > 1.0:
                obj_colors = obj_colors / 255.0
            obj_pcd.colors = o3d.utility.Vector3dVector(
                np.clip(obj_colors, 0.0, 1.0))

        hmsg.mask_pcds.append(obj_pcd)
        feat = feat_map.get(obj_id)
        if feat is None:
            feat = np.zeros((hmsg.clip_feat_dim,), dtype=np.float32)
        else:
            feat = _normalize_feat(feat, hmsg.clip_feat_dim)
        hmsg.mask_feats.append(feat)

    if not hmsg.mask_pcds:
        raise ValueError(
            "No valid instances found from OVO map for HMSG construction")

    print(
        f"[HMSG] scene={scene_name}, points={len(xyz)}, instances={len(hmsg.mask_pcds)}")
    hmsg.build_hier_multimodal_scene_graph(save_path=str(scene_output_path))
    return str(scene_output_path)
