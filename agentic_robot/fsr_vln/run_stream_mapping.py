from typing import Any, Dict, Deque, Optional, Tuple
from datetime import datetime
from pathlib import Path
import argparse
import wandb
import torch
import numpy as np
import time
import yaml
import uuid
import gc
import os
import shutil
import rclpy
from rclpy.node import Node
from ovo.utils import io_utils, gen_utils, eval_utils
from ovo.entities.semantic_mapping_online import SemanticMapping
from ovo.entities.obj_detect_track import ObjDetectTrack


def load_representation(
        scene_path: Path,
        eval: bool = False) -> ObjDetectTrack:
    config = io_utils.load_config(scene_path / "config.yaml", inherit=False)
    submap_ckpt = torch.load(scene_path / "ovo_map.ckpt")
    map_params = submap_ckpt.get("map_params", None)
    if map_params is None:
        map_params = submap_ckpt["gaussian_params"]

    obj_detect_track = ObjDetectTrack(
        config["semantic"],
        None,
        config["data"]["scene_name"],
        eval=eval,
        device=config.get(
            "device",
            "cuda"))
    obj_detect_track.restore_dict(submap_ckpt["ovo_map_params"])
    return obj_detect_track, map_params


def compute_scene_labels(
        scene_path: Path,
        dataset_name: str,
        scene_name: str,
        data_path: str,
        dataset_info: Dict) -> None:

    obj_detect_track, map_params = load_representation(scene_path, eval=True)
    pcd_pred = map_params["xyz"]
    points_obj_ids = map_params["obj_ids"]

    _, pcd_gt = io_utils.load_scene_data(
        dataset_name, scene_name, data_path, dataset_info, False)
    classes = dataset_info["class_names"] if dataset_info.get(
        "map_to_reduced", None) is None else dataset_info["class_names_reduced"]
    pred_path = scene_path.parent / dataset_info["dataset"]
    os.makedirs(pred_path, exist_ok=True)
    pred_path = pred_path / (scene_name + ".txt")

    # It may happen that all the points associated to an object where prunned,
    # such that the number of unique labels in points_obj_ids, is different
    # from the number of semantic module instances
    print("Computing predicted instances labels ...")

    instances_info = obj_detect_track.classify_instances(classes)

    mesh_semantic_labels = dict()
    print("Matching instances to ground truth mesh ...")
    mesh_instance_labels, mesh_instances_masks, matched_instances_ids = eval_utils.match_labels_to_vtx(
        points_obj_ids[:, 0], pcd_pred, pcd_gt)

    map_id_to_idx = {
        id: i for i, id in enumerate(
            obj_detect_track.objects.keys())}
    mesh_semantic_labels = instances_info["classes"][np.vectorize(
        map_id_to_idx.get)(mesh_instance_labels)]
    instances_info["masks"] = mesh_instances_masks.int().numpy()

    """
    import open3d as o3d
    import numpy as np
    gt_pcd = o3d.geometry.PointCloud()
    gt_pcd.points = o3d.utility.Vector3dVector(pcd_gt)
    pred_pcd = o3d.geometry.PointCloud()
    pred_pcd.points = o3d.utility.Vector3dVector(pcd_pred)
    o3d.visualization.draw_geometries_with_key_callbacks([gt_pcd, pred_pcd], {})
    """

    print(f"Writing prediction to {pred_path}!")
    io_utils.write_labels(pred_path, mesh_semantic_labels)
    io_utils.write_instances(scene_path.parent, scene_name, instances_info)

    obj_detect_track.cpu()
    del obj_detect_track


def run_semantic_mapping(data_dir: str, scene: str, dataset: str, experiment_name: str, config_path: str, tmp_run: bool = False, depth_filter: bool = False) -> None:  # 修复拼写错误
    config = io_utils.load_config(config_path)
    map_module = config["slam"]["slam_module"]

    config_slam = io_utils.load_config(
        os.path.join(
            config["slam"]["config_path"],
            map_module,
            dataset.lower() +
            ".yaml"))
    io_utils.update_recursive(config, config_slam)

    config_dataset = io_utils.load_config(
        f"configs/{dataset}/{dataset.lower()}.yaml")
    io_utils.update_recursive(config, config_dataset)

    if os.path.exists(f"configs/{dataset}/{dataset.lower()}.yaml"):
        config_scene = io_utils.load_config(
            f"configs/{dataset}/{dataset.lower()}.yaml")
        io_utils.update_recursive(config, config_scene)

    if "data" not in config:
        config["data"] = {}
    # config["data"]["scene_name"] = scene
    # output_path = Path(f"data/output/{dataset}/")
    config["data"]["scene_name"] = scene
    config["data"]["input_path"] = os.path.join(
        data_dir, "input", "datasets", dataset, "decoded_scans", scene)

    # output_path = Path(f"data/output/{dataset}/")
    output_path = Path(os.path.join(data_dir, "output", dataset))
    if tmp_run:
        output_path = output_path / "tmp"

    output_path = output_path / experiment_name / scene

    if depth_filter is not None:
        config["semantic"]["depth_filter"] = depth_filter

    if os.getenv('DISABLE_WANDB') == 'true':
        config["use_wandb"] = False
    elif config.get("use_wandb", False):
        wandb.init(
            project=config.get(
                "project_name",
                "default_project"),
            config=config,
            dir="data/working/output/wandb",
            group=experiment_name if experiment_name else config["data"]["scene_name"],
            name=f'{config["data"]["scene_name"]}_{time.strftime("%Y%m%d_%H%M%S", time.localtime())}_{str(uuid.uuid4())[:5]}',
        )
    # 打印配置信息
    print("Configuration:")
    for key, value in config.items():
        if isinstance(value, dict):
            print(f"{key}:")
            for sub_key, sub_value in value.items():
                print(f"  {sub_key}: {sub_value}")
        else:
            print(f"{key}: {value}")
    gen_utils.setup_seed(config.get("seed", 42))
    try:
        run_ros_semantic_mapping(config, output_path)  # 修复拼写错误
    except Exception as e:
        print(f"运行时发生错误: {e}")
    finally:
        if config.get("use_wandb", False):
            wandb.finish()

    print("Finished run.✨")


# 修复拼写错误
def run_ros_semantic_mapping(config: Dict[str, Any], output_path: str):
    rclpy.init()
    node = SemanticMapping(
        config,
        output_path,
        scene=config["data"]["scene_name"])
    node.spin()


def main(args):
    if args.experiment_name is None:
        experiment_name = datetime.now().strftime("%Y%m%d_%H%M")
        tmp_run = True
    else:
        assert len(args.experiment_name) > 0, "Experiment name cannot be '' "
        experiment_name = args.experiment_name
        tmp_run = False

    run_semantic_mapping(
        args.data_dir,
        args.scene_name,
        args.dataset_name,
        args.experiment_name,
        args.config_path,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Arguments to run and evaluate over a dataset')
    parser.add_argument(
        '--data_dir',
        type=str,
        required=True,
        help='Output data directory')
    parser.add_argument(
        '--scene_name',
        type=str,
        required=True,
        help='Scene name to process')
    parser.add_argument(
        '--dataset_name',
        type=str,
        required=True,
        help='Dataset name')
    parser.add_argument(
        '--experiment_name',
        type=str,
        default=None,
        help='Experiment name')
    parser.add_argument(
        '--depth_filter',
        type=bool,
        default=False,
        help='Enable depth filter')
    parser.add_argument(
        '--config_path',
        type=str,
        default="configs/ovo_zed.yaml",
        help='Path to the config file')

    args = parser.parse_args()
    main(args)
