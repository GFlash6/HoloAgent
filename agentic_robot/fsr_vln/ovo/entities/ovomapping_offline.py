from __future__ import annotations
from typing import Any, Dict
import torch
import torch.multiprocessing as mp
from pathlib import Path
import numpy as np
import time
import os
import gc
import open3d as o3d  # 确保安装了 Open3D 库
from .logger import Logger
from ovo.entities.obj_detect_track import ObjDetectTrack
from .datasets import get_dataset
from .visualizer import stream_pcd
from ..slam.vanilla_mapper import VanillaMapper
from ..utils import io_utils
import matplotlib.pyplot as plt
import yaml
from ..utils import vis_utils
import time
import cv2
from PIL import Image


def get_slam_backbone(config: Dict[str, Any], dataset,
                      cam_intrinsics: torch.Tensor) -> VanillaMapper:
    backbone = config["slam"].get("slam_module", "vanilla")
    if backbone == "orbslam2":
        from ..slam.orbslam2 import WrapperORBSLAM2
        return WrapperORBSLAM2(
            config,
            cam_intrinsics,
            world_ref=torch.from_numpy(
                dataset[0][3]))
    else:
        return VanillaMapper(config, cam_intrinsics)


class OVOSemMap():
    """
    OVOSemMap is a class responsible for managing the semantic mapping process
    using the OVO framework.

    It initializes the necessary components, sets up the output path, and handles the main program flow including tracking, mapping,
    and semantic segmentation.
    Args:
        config (dict): Configuration dictionary containing experiment settings.
    """

    def __init__(self, config: Dict[str, Any],
                 output_path: str, scene: str) -> None:
        self._setup_output_path(output_path)
        io_utils.save_dict_to_yaml(
            config, "config.yaml", directory=self.output_path)
        config["output_path"] = str(self.output_path)
        self.scene = scene
        self.save_time = 0
        self.save_obj_clouds = True
        self.save_kf_cloud = True

        self.config = config
        self.device = config.get("device", "cuda")
        self.dataset_name = config["dataset_name"]
        self.stream = self.config["vis"]["stream"]
        self.show_stream = self.config["vis"]["show_stream"]
        self.map_every = config["mapping"].get("map_every", 10)
        print("Mapping every", self.map_every, "frames")
        self.segment_every = config["semantic"].get("segment_every", 10)
        print("Segmenting every", self.segment_every, "frames")
        if config.get("tracking", None) is None:
            self.track_every = 1
        else:
            self.track_every = config["tracking"].get("track_every", 1)

        self.logger = Logger(
            self.output_path,
            os.getpid(),
            config["use_wandb"])
        self.dataset = get_dataset(config["dataset_name"])(
            {**config["data"], **config["cam"]})

        cam_intrinsics = torch.tensor(
            self.dataset.intrinsics.astype(
                np.float32), device=self.device)
        print("Camera intrinsics:", cam_intrinsics)
        config["semantic"]["debug_info"] = self.config.get("debug_info", False)

        self.slam_backbone = get_slam_backbone(
            config, self.dataset, cam_intrinsics)
        self.ovo = ObjDetectTrack(
            config["semantic"],
            self.logger,
            self.dataset_name,
            config["data"]["scene_name"],
            cam_intrinsics,
            device=self.device)
        self.pcd_dir = Path(self.output_path) / "pcd"
        self.pcd_dir.mkdir(exist_ok=True, parents=True)
        if config["semantic"]["sam"].get(
                "precomputed",
                False) or config["semantic"]["sam"].get(
                "precompute",
                False):
            self.ovo.mask_generator.precompute(
                self.dataset, self.segment_every)

    def _setup_output_path(self, output_path: str) -> None:
        """
        Sets up the output path for saving results based on the provided
        configuration.

        Args:
            config: A dictionary containing the experiment configuration including data and output path information.
        """
        self.output_path = Path(output_path)
        self.output_path.mkdir(exist_ok=True, parents=True)

    def save_representation(self) -> None:
        """
        Saves the current map and scene objects parameters to a checkpoint
        file.

        This method retrieves the map parameters from the SLAM backbone and the
        scene objects parameters from the OVO module. It then creates a
        dictionary containing these parameters and saves it to a checkpoint
        file named after the submap ID. The checkpoint file is saved in the
        'submaps' directory within the specified output path.
        """
        print("Saving representation ...")
        map_params = self.slam_backbone.get_map_dict()
        ovo_map_params = self.ovo.capture_dict(
            debug_info=self.config.get("debug", False))
        submap_ckpt = {
            "map_params": map_params,
            "ovo_map_params": ovo_map_params,
        }
        io_utils.save_dict_to_ckpt(
            submap_ckpt, "ovo_map.ckpt", directory=self.output_path)
        print("Representation is saved to", self.output_path / "ovo_map.ckpt")

    def save_mask_overlay(
            self,
            frame_id: int,
            image: np.ndarray,
            matched_ins_ids: list,
            binary_maps: torch.Tensor) -> None:
        """将每张图像的 mask 叠加到原图，相同 id 的 mask 使用相同颜色，绘制 id 文本，并保存到 mask_overlays
        目录。"""
        if not matched_ins_ids or binary_maps is None or binary_maps.shape[0] == 0:
            return
        out_dir = Path(self.output_path) / "mask_overlays"
        out_dir.mkdir(exist_ok=True, parents=True)

        # 唯一 id -> 颜色 (RGB 0-255)，相同 id 始终相同颜色
        # 使用 HSV 色彩空间，根据 ID 均匀分配色相，确保高区分度
        unique_ids = sorted(set(matched_ins_ids))
        id_to_color = {}

        # 使用 HSV 色彩空间生成高区分度颜色
        # H (色相): 0-179 (OpenCV 范围)，均匀分布
        # S (饱和度): 固定高值 220-255，确保颜色鲜艳
        # V (亮度): 固定高值 220-255，确保颜色明亮

        for idx, ins_id in enumerate(unique_ids):
            if ins_id not in id_to_color:
                # 色相均匀分布：将 ID 映射到 0-179 范围
                # 使用黄金角度分割（约 137.5度），确保相邻颜色差异最大
                golden_angle = 137.508  # 黄金角度（度）
                h_float = (ins_id * golden_angle) % 180.0
                h = int(h_float * 179.0 / 180.0)  # 转换为 OpenCV 的 0-179 范围

                # 饱和度和亮度：使用高值确保颜色鲜艳明亮
                # 稍微变化 S 和 V 以增加区分度（避免所有颜色都完全一样）
                s_variation = (ins_id * 7) % 36  # 0-35 的变化
                s = 220 + s_variation  # 220-255，高饱和度
                v = 240 - (s_variation // 2)  # 225-240，高亮度

                # HSV -> RGB
                hsv = np.array([[[h, s, v]]], dtype=np.uint8)
                rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
                id_to_color[ins_id] = rgb[0, 0]

        overlay = np.asarray(image, dtype=np.uint8).copy()
        if overlay.max() <= 1.0:
            overlay = (overlay * 255).astype(np.uint8)

        H_img, W_img = overlay.shape[:2]
        alpha = 0.70  # mask 颜色更突出，增大 alpha 使着色更浓

        for i, ins_id in enumerate(matched_ins_ids):
            mask = binary_maps[i]
            if isinstance(mask, torch.Tensor):
                mask = mask.detach().cpu().numpy()
            if mask.shape[0] != H_img or mask.shape[1] != W_img:
                mask = np.asarray(
                    torch.nn.functional.interpolate(
                        torch.from_numpy(mask).float(
                        ).unsqueeze(0).unsqueeze(0),
                        size=(
                            H_img,
                            W_img),
                        mode="nearest").squeeze().numpy() > 0.5,
                    dtype=bool)
            else:
                mask = mask.astype(bool)

            color = id_to_color[ins_id]  # (3,) RGB
            overlay[mask] = (alpha * color + (1 - alpha) *
                             overlay[mask]).astype(np.uint8)

        # 在 mask 中心绘制 id 文本
        overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
        for i, ins_id in enumerate(matched_ins_ids):
            mask = binary_maps[i]
            if isinstance(mask, torch.Tensor):
                mask = mask.detach().cpu().numpy()
            if mask.shape[0] != H_img or mask.shape[1] != W_img:
                mask = np.asarray(
                    torch.nn.functional.interpolate(
                        torch.from_numpy(
                            mask.astype(
                                np.float32)).unsqueeze(0).unsqueeze(0), size=(
                            H_img, W_img), mode="nearest").squeeze().numpy() > 0.5, dtype=bool)
            else:
                mask = mask.astype(bool)
            ys, xs = np.where(mask)
            if len(ys) > 0 and len(xs) > 0:
                cy, cx = int(ys.mean()), int(xs.mean())
                font_scale = max(0.4, min(0.6, 800 / max(H_img, W_img)))
                thickness = max(1.0, int(round(1.5 * 800 / max(H_img, W_img))))
                cv2.putText(
                    overlay_bgr,
                    str(ins_id),
                    (cx,
                     cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    (255,
                     255,
                     255),
                    thickness,
                    cv2.LINE_AA)
                cv2.putText(
                    overlay_bgr, str(ins_id), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), max(
                        1, thickness - 1), cv2.LINE_AA)
        overlay = cv2.cvtColor(overlay_bgr, cv2.COLOR_BGR2RGB)
        time_stamp = self.dataset.time_stamps[frame_id] if hasattr(
            self.dataset, "time_stamps") else None

        save_path = out_dir / f"{time_stamp}.png"
        if overlay.max() <= 1.0:
            overlay = (overlay * 255).astype(np.uint8)
        Image.fromarray(overlay).save(save_path)
        print(f"Saved mask overlay to {save_path}")

    def dbscan_filter_point_cloud(
        self,
        xyz: np.ndarray,
        rgb: np.ndarray,
        obj_id: np.ndarray,
        eps: float = 0.02,
        min_points: int = 10,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Filter point cloud outliers with DBSCAN before writing semantic
        PLY."""
        if xyz.shape[0] < min_points:
            return xyz, rgb, obj_id

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
        labels = np.asarray(
            pcd.cluster_dbscan(
                eps=eps,
                min_points=min_points,
                print_progress=False))
        keep_mask = labels >= 0
        if not np.any(keep_mask):
            return xyz, rgb, obj_id
        return xyz[keep_mask], rgb[keep_mask], obj_id[keep_mask]

    def write_semantic_ply(
            self,
            path: Path,
            xyz: np.ndarray,
            rgb: np.ndarray,
            obj_id: np.ndarray) -> None:
        """Write PLY with x, y, z, red, green, blue, obj_id per vertex
        (semantic point cloud)."""
        n = xyz.shape[0]
        obj_id = np.asarray(obj_id).reshape(n).astype(np.int32)
        rgb = np.asarray(rgb).astype(np.uint8)
        if rgb.ndim == 1:
            rgb = rgb.reshape(-1, 3)
        dtype = np.dtype([
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
            ("red", "u1"), ("green", "u1"), ("blue", "u1"),
            ("obj_id", "<i4"),
        ])
        arr = np.empty(n, dtype=dtype)
        arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        arr["red"], arr["green"], arr["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        arr["obj_id"] = obj_id
        with open(path, "wb") as f:
            f.write(b"ply\n")
            f.write(b"format binary_little_endian 1.0\n")
            f.write(f"element vertex {n}\n".encode())
            f.write(b"property float x\nproperty float y\nproperty float z\n")
            f.write(
                b"property uchar red\nproperty uchar green\nproperty uchar blue\n")
            f.write(b"property int obj_id\n")
            f.write(b"end_header\n")
            arr.tofile(f)

    def save_point_clouds(self):
        pcd_pred, points_obj_ids, pcd_obj_ids = self.slam_backbone.get_map()
        pcd_pred = pcd_pred.cpu().numpy().astype(np.float32)
        points_obj_ids = points_obj_ids.cpu().numpy().astype(np.int32)
        pcd_obj_ids = pcd_obj_ids.cpu().numpy().astype(np.int32)
        pcd_colors = self.slam_backbone.get_pcd_colors()
        pcd_rgb = o3d.geometry.PointCloud()
        pcd_rgb.points = o3d.utility.Vector3dVector(pcd_pred)
        pcd_rgb.colors = o3d.utility.Vector3dVector(pcd_colors / 255.)
        out_path = str(self.output_path) + "/" + self.scene + "_rgb.ply"
        o3d.io.write_point_cloud(out_path, pcd_rgb)
        if self.save_obj_clouds:
            ids_input = list(self.ovo.objects.keys())
            ids_input = np.array(ids_input, dtype=np.int32)
            obj_masks, ids = vis_utils.get_obj_ids_and_masks(
                pcd_obj_ids, ids_input)
            obj_colors_for_instances = vis_utils.get_pcd_colors(
                ids, vis_utils.get_cmap())
            obb_obj_list = vis_utils.get_obj_and_obb(
                obj_masks, pcd_pred, obj_colors_for_instances)
            instance_dir = Path(str(self.output_path) + "/instance/")
            instance_dir.mkdir(exist_ok=True, parents=True)
            merged_cloud = o3d.geometry.PointCloud()
            for i, obj_pcd in enumerate(obb_obj_list):
                merged_cloud += obj_pcd.to_legacy()
                out_path = str(
                    instance_dir / (self.scene + f"_obj_{ids[i]}_obb.ply"))
                o3d.io.write_point_cloud(out_path, obj_pcd.to_legacy())
                print(f"Object {ids[i]} point cloud saved to {out_path}")
            out_path = str(self.output_path) + "/" + self.scene + "_obj.ply"
            o3d.io.write_point_cloud(out_path, merged_cloud)
            print(f"Point cloud saved to {out_path}")

        # Save per-keyframe semantic point clouds as PLY (xyz + rgb by obj_id +
        # obj_id; ref: iggt_slam update_map kfs/pcd_idxs)
        if self.save_kf_cloud:
            kfs = self.slam_backbone.get_kfs()
            if kfs:
                keyframe_pcd_dir = self.pcd_dir / "keyframe_pcd"
                keyframe_pcd_dir.mkdir(exist_ok=True, parents=True)
                cmap = vis_utils.get_cmap()
                dbscan_config = self.config.get(
                    "semantic", {}).get(
                    "dbscan_filter", {})
                dbscan_eps = dbscan_config.get("eps", 0.02)
                dbscan_min_points = dbscan_config.get("min_points", 10)
                for frame_id in sorted(kfs.keys()):
                    pcd_start, pcd_end = kfs[frame_id]["pcd_idxs"]
                    kf_xyz = pcd_pred[pcd_start:pcd_end].astype(np.float32)
                    kf_obj_id = pcd_obj_ids[pcd_start:pcd_end].flatten().astype(
                        np.int32)
                    # Semantic PLY: xyz + rgb by obj_id + obj_id
                    kf_semantic_rgb = vis_utils.get_pcd_colors(
                        kf_obj_id, cmap)  # (N,3) float [0,1]
                    kf_rgb = (
                        kf_semantic_rgb *
                        255.0).clip(
                        0,
                        255).astype(
                        np.uint8)
                    kf_xyz, kf_rgb, kf_obj_id = self.dbscan_filter_point_cloud(
                        kf_xyz, kf_rgb, kf_obj_id, eps=dbscan_eps, min_points=dbscan_min_points)
                    time_stamp = self.dataset.time_stamps[frame_id] if hasattr(
                        self.dataset, "time_stamps") else None
                    self.write_semantic_ply(
                        keyframe_pcd_dir /
                        f"{time_stamp}.ply",
                        kf_xyz,
                        kf_rgb,
                        kf_obj_id)
                    # # RGB point cloud: xyz + original image colors
                    # kf_rgb_original = pcd_colors[pcd_start:pcd_end]  # (N,3) uint8
                    # pcd_rgb_kf = o3d.geometry.PointCloud()
                    # pcd_rgb_kf.points = o3d.utility.Vector3dVector(kf_xyz)
                    # pcd_rgb_kf.colors = o3d.utility.Vector3dVector(kf_rgb_original.astype(np.float64) / 255.0)
                    # o3d.io.write_point_cloud(str(keyframe_pcd_dir / f"kf_{frame_id:06d}_rgb.ply"), pcd_rgb_kf)
                print(
                    f"Saved {len(kfs)} keyframe semantic + RGB point clouds to {keyframe_pcd_dir}")

    def run(self) -> None:
        """
        Starts the main program flow, including tracking and mapping.

        If self.config["vis"]["stream"] is True, a Open3D visualizer is
        launched in a parallel process.
        """
        stream = self.config["vis"]["stream"]
        show_stream = self.config["vis"]["show_stream"]

        with mp.Manager() as manager:
            if stream:
                cam_data = {
                    "height": self.dataset.height,
                    "width": self.dataset.width,
                    "intrinsic": self.dataset.intrinsics}
                mpqueue = mp.Queue()
                # 0 idle, 1 requested, 2 completed
                query_flag = mp.Value('i', 0)
                query_pipe, vis_pipe = mp.Pipe()
                p = mp.Process(target=stream_pcd,
                               args=(self.ovo,
                                     mpqueue,
                                     [query_flag,
                                      vis_pipe],
                                     cam_data,
                                     self.config["data"]["scene_name"],
                                     self.logger.output_path,
                                     show_stream),
                               name="O3DVisualizer")
                p.start()

            torch.cuda.synchronize()
            t_start = time.time()
            print("Dataset length:", len(self.dataset))
            total_frames = len(self.dataset)
            valid_frames = total_frames // self.map_every
            last_map_frame_id = (
                total_frames - 1) // self.map_every * self.map_every
            update_fre = self.map_every * 100
            for frame_id in range(len(self.dataset)):
                # print("Processing frame", frame_id)
                if self.track_every == 1 or frame_id % self.track_every == 0 or frame_id % self.map_every == 0 or frame_id % self.segment_every == 0:
                    frame_data = self.dataset[frame_id]
                    self.slam_backbone.track_camera(frame_data)

                    estimated_c2w = self.slam_backbone.get_c2w(frame_id)
                    missing_depth = not (frame_data[2] > 0).any()
                    if estimated_c2w is None or missing_depth:
                        continue

                    t_lc = 0
                    if frame_id % self.map_every == 0 or self.config["slam"]["slam_module"] == "orbslam2":
                        self.slam_backbone.map(frame_data, estimated_c2w)
                        is_last_map_tick = (frame_id == last_map_frame_id)
                        if self.slam_backbone.map_updated:  # We want to update more frequently for visualization purposes, but this can be turned off for faster mapping without visualization
                            torch.cuda.synchronize()
                            t_lc_i = time.time()
                            map_data = self.slam_backbone.get_map()
                            t_get_map = time.time() - t_lc_i
                            print(f"Get map took {t_get_map};")
                            kfs = self.slam_backbone.get_kfs()
                            t_update_map_begin = time.time()
                            updated_points_ins_ids = self.ovo.update_map(
                                map_data, kfs)
                            t_update_map_end = time.time()
                            print(
                                f"Update map took {t_update_map_end - t_update_map_begin};")
                            if updated_points_ins_ids is not None:
                                t_update_pcd_obj_ids_begin = time.time()
                                self.slam_backbone.update_pcd_obj_ids(
                                    updated_points_ins_ids)
                                t_update_pcd_obj_ids_end = time.time()
                                print(
                                    f"Update PCD obj IDs took {t_update_pcd_obj_ids_end - t_update_pcd_obj_ids_begin};")
                            self.slam_backbone.map_updated = False
                            torch.cuda.synchronize()
                            t_lc = time.time() - t_lc_i
                            print(f"Sem LC update took {t_lc};")

                    t_sem = 0
                    if frame_id % self.segment_every == 0:
                        t_sem_i = time.time()
                        autocast_device_type = "cuda" if str(
                            self.device).startswith("cuda") else str(self.device)
                        with torch.no_grad(), torch.autocast(device_type=autocast_device_type, dtype=torch.bfloat16):
                            if len(frame_data) == 5:
                                image = frame_data[-1]
                            else:
                                image = frame_data[1]

                            if self.dataset.height != image.shape[0] or self.dataset.width != image.shape[1]:
                                rgb_depth_ratio = (
                                    image.shape[0] /
                                    self.dataset.dataset_config["H"],
                                    image.shape[1] /
                                    self.dataset.dataset_config["W"],
                                    self.dataset.crop_edge)
                            else:
                                rgb_depth_ratio = ()

                            scene_data = [
                                frame_id, image, frame_data[2], rgb_depth_ratio]

                            map_data = self.slam_backbone.get_map()
                            updated_points_ins_ids, matched_ins_ids, binary_maps = self.ovo.detect_and_track_objects(
                                scene_data, map_data, estimated_c2w)
                            if updated_points_ins_ids is not None:
                                self.slam_backbone.update_pcd_obj_ids(
                                    updated_points_ins_ids)
                            # # save overlay for each frame with matched instances
                            # if matched_ins_ids is not None and binary_maps is not None:
                            #     self.save_mask_overlay(frame_id, image, matched_ins_ids, binary_maps)

                            self.ovo.compute_semantic_info()
                            self.logger.log_memory_usage(frame_id)
                        t_sem = time.time() - t_sem_i
                        print(f"Semantic update took {t_sem};")

                        if stream:
                            pcd, _, pcd_obj_ids = self.slam_backbone.get_map()
                            c2w = self.slam_backbone.get_c2w(frame_id)
                            if c2w is None:
                                continue
                            c2w = c2w.cpu().numpy().astype(np.float16)
                            colors = self.slam_backbone.get_pcd_colors()

                            mpqueue.put([pcd.cpu().numpy().astype(
                                np.float16), pcd_obj_ids.cpu().numpy().astype(np.int16), colors, c2w])

                            if query_flag.value == 1:
                                query = query_pipe.recv()
                                query_map = self.ovo.query(query).cpu().numpy()
                                with query_flag.get_lock():
                                    query_pipe.send(query_map)
                                    query_flag.value = 2
                    if frame_id % 50 == 0:
                        gc.collect()

            self.ovo.complete_semantic_info()

            torch.cuda.synchronize()
            t_end = time.time()
            fps = len(self.dataset) / self.segment_every / (t_end - t_start)
            if stream and p.is_alive():
                while mpqueue.qsize() > 0:
                    if query_flag.value == 1:
                        query = query_pipe.recv()
                        query_map = self.ovo.query(query).cpu().numpy()
                        with query_flag.get_lock():
                            query_pipe.send(query_map)
                            query_flag.value = 2
                    time.sleep(2)
                time.sleep(5)

        self.logger.log_fps(fps)
        # self.logger.log_max_memory_usage()
        self.logger.write_stats()
        self.logger.print_final_stats()

        self.save_representation()

        save_t0 = time.time()
        # load SCANNET_COLOR_MAP_200 from saved config.yaml if present
        self.save_point_clouds()
        save_t1 = time.time()
        print(f"Saving point cloud took {save_t1 - save_t0} seconds.")
        self.save_time = save_t1 - save_t0

        self.ovo.cpu()
        del self.slam_backbone, self.ovo
        torch.cuda.empty_cache()

        if self.config["vis"].get("stream", False):
            p.terminate()
