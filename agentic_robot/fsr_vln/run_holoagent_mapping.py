from typing import Dict
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

from ovo.utils import io_utils, gen_utils, eval_utils
from ovo.entities.ovomapping_offline import OVOSemMap
from ovo.entities.obj_detect_track import ObjDetectTrack

# Import sim3 alignment utilities
import sys


def load_representation(
        scene_path: Path,
        dataset_name: str,
        eval: bool = False) -> ObjDetectTrack:
    config = io_utils.load_config(scene_path / "config.yaml", inherit=False)
    submap_ckpt = torch.load(scene_path / "ovo_map.ckpt")
    map_params = submap_ckpt.get("map_params", None)
    if map_params is None:
        map_params = submap_ckpt["gaussian_params"]

    ovo = ObjDetectTrack(
        config["semantic"],
        None,
        dataset_name,
        config["data"]["scene_name"],
        eval=eval,
        device=config.get(
            "device",
            "cuda"))
    ovo.restore_dict(submap_ckpt["ovo_map_params"])
    return ovo, map_params


def compute_scene_labels(
        scene_path: Path,
        dataset_name: str,
        scene_name: str,
        data_path: str,
        dataset_info: Dict) -> None:

    ovo, map_params = load_representation(scene_path, dataset_name, eval=True)
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

    instances_info = ovo.classify_instances(classes)

    mesh_semantic_labels = dict()
    print("Matching instances to ground truth mesh ...")
    mesh_instance_labels, mesh_instances_masks, matched_instances_ids = eval_utils.match_labels_to_vtx(
        points_obj_ids[:, 0], pcd_pred, pcd_gt)

    map_id_to_idx = {id: i for i, id in enumerate(ovo.objects.keys())}
    mesh_semantic_labels = instances_info["classes"][np.vectorize(
        map_id_to_idx.get)(mesh_instance_labels)]
    # # 先将 None 替换为 -1
    # idx_array = np.vectorize(lambda x: map_id_to_idx.get(x, -1))(mesh_instance_labels)
    # # 只对有效索引赋值，否则赋默认背景/无效标签
    # valid_mask = idx_array != -1
    # mesh_semantic_labels = np.full_like(idx_array, fill_value=-1)  # -1 表示无效/背景
    # mesh_semantic_labels[valid_mask] = instances_info["classes"][idx_array[valid_mask]]
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

    ovo.cpu()
    del ovo


def run_scene(
        data_dir: str,
        scene: str,
        dataset: str,
        experiment_name: str,
        config: dict,
        tmp_run: bool = False,
        depth_filter: bool = None) -> Path:

    # config = io_utils.load_config("configs/ovo.yaml")
    map_module = config["slam"]["slam_module"]
    if map_module == "orbslam2":
        map_module = "vanilla"

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
    config["data"]["scene_name"] = scene
    config["data"]["input_path"] = os.path.join(
        data_dir, "input", "datasets", dataset, "decoded_scans", scene)
    output_path = Path(os.path.join(data_dir, "output", dataset))

    if tmp_run:
        output_path = output_path / "tmp"

    output_path = output_path / experiment_name / scene

    if depth_filter is not None:
        config["semantic"]["depth_filter"] = depth_filter

    if os.getenv('DISABLE_WANDB') == 'true':
        config["use_wandb"] = False
    elif config["use_wandb"]:
        wandb.init(
            project=config["project_name"],
            config=config,
            dir="data/working/output/wandb",
            group=config["data"]["scene_name"]
            if experiment_name != ""
            else experiment_name,
            name=f'{config["data"]["scene_name"]}_{time.strftime("%Y%m%d_%H%M%S", time.localtime())}_{str(uuid.uuid4())[:5]}',
        )

    gen_utils.setup_seed(config["seed"])
    gslam = OVOSemMap(config, output_path=output_path, scene=scene)
    gslam.run()

    if tmp_run:
        final_path = Path(f"data/output/{dataset}/") / experiment_name / scene
        shutil.move(output_path, final_path)
        output_path = final_path

    if config["use_wandb"]:
        wandb.finish()
    print("Finished run.✨")
    return output_path, gslam.save_time


def main(args):
    if args.experiment_name is None:
        experiment_name = datetime.now().strftime("%Y%m%d_%H%M")
        tmp_run = True
    else:
        assert len(args.experiment_name) > 0, "Experiment name cannot be '' "
        experiment_name = args.experiment_name
        tmp_run = False

    data_dir = args.data_dir
    experiment_path = os.path.join(
        data_dir,
        "output",
        args.dataset_name,
        experiment_name)

    if args.scenes_list is not None:
        # Accept either a filepath to a file containing scene names or an inline
        # comma/newline-separated list passed as the argument (e.g. "a,b,c" or
        # "a\nb\nc").
        s = args.scenes_list
        if os.path.isfile(s):
            with open(s, "r") as f:
                scenes = f.read().splitlines()
        else:
            # treat the provided string as an inline list of scenes
            scenes = [item.strip() for item in s.replace(
                ',', '\n').splitlines() if item.strip()]
    else:
        scenes = args.scenes

    if len(scenes) == 0 or args.segment or args.eval:
        path = Path("configs/") / args.dataset_name / args.dataset_info_file
        with open(path, 'r') as f:
            dataset_info = yaml.full_load(f)

        if len(scenes) == 0:
            scenes = dataset_info["scenes"]

    cfg_path = args.cfg_path
    config = io_utils.load_config(cfg_path)
    gslam = None
    for scene in scenes:
        input_path = os.path.join(
            data_dir,
            "input",
            "datasets",
            args.dataset_name,
            "decoded_scans",
            scene)
        if not os.path.exists(input_path):
            print(
                f"Scene {scene} does not exist under {input_path}, skipping ...")
            continue

        if args.run:
            t0 = time.time()
            scene_output_path, save_time = run_scene(
                data_dir, scene, args.dataset_name, experiment_name, config, tmp_run=tmp_run)
            t1 = time.time()
            print(f"Scene {scene} took: {t1-t0:.2f}")
            # 写入run-time日志
            log_path = os.path.join(experiment_path, "run_time.log")
            with open(log_path, "a") as log_file:
                log_file.write(
                    f"{scene} instance mapping time: {t1-t0:.2f} seconds\n")
                log_file.write(f"{scene} save time: {save_time:.2f} seconds\n")

        if args.build_hmsg:
            from ovo.integration import build_hmsg_from_ovo_output

            print(f"[HMSG] Building scene graph for scene {scene} ...")
            hmsg_t0 = time.time()
            scene_input_path = os.path.join(
                data_dir,
                "input",
                "datasets",
                args.dataset_name,
                "decoded_scans",
                scene,
            )
            scene_output_path = os.path.join(
                data_dir,
                "output",
                args.dataset_name,
                experiment_name,
                scene,
            )
            build_hmsg_from_ovo_output(
                scene_output_path=scene_output_path,
                scene_input_path=scene_input_path,
                dataset_name=args.dataset_name,
                scene_name=scene,
                hmsg_config_path=args.hmsg_config_path,
                obj_labels=args.hmsg_obj_labels,
                min_object_points=args.hmsg_min_object_points,
            )
            hmsg_t1 = time.time()
            print(f"[HMSG] Scene {scene} graph done in {hmsg_t1-hmsg_t0:.2f}s")
            log_path = os.path.join(experiment_path, "run_time.log")
            with open(log_path, "a") as log_file:
                log_file.write(
                    f"{scene} HMSG build time: {hmsg_t1-hmsg_t0:.2f} seconds\n")
        gc.collect()

    if args.segment:
        # data_path ="data/input/datasets/"
        data_path = os.path.join(data_dir, "input", "datasets")
        align_method = config["evaluation"].get("align_method", "none")
        print(f"Using alignment: method={align_method}")
        for scene in scenes:
            scene_path = Path(experiment_path) / scene
            compute_scene_labels(
                scene_path,
                args.dataset_name,
                scene,
                data_path,
                dataset_info,
                align_method,
            )

    if args.eval:
        if dataset_info["dataset"] == "scannet200":
            gt_path = Path(input_path).parent.parent / "scannet200_semantic_gt"
        elif dataset_info["dataset"] == "scannet20":
            gt_path = Path(input_path).parent.parent / "scannet20_semantic_gt"
        elif dataset_info["dataset"] == "replica":
            gt_path = Path(input_path).parent.parent / "replica_semantic_gt"
        print(f"Evaluating scene {scene} ...")
        print("Ground truth path:", gt_path)
        print(
            "Experiment path:",
            Path(experiment_path) /
            dataset_info["dataset"])
        eval_utils.eval_semantics(
            Path(experiment_path) /
            dataset_info["dataset"],
            gt_path,
            scenes,
            dataset_info,
            ignore_background=args.ignore_background)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Arguments to run and evaluate over a dataset')
    parser.add_argument('--data_dir', default=None, type=str)
    parser.add_argument('--cfg_path', default="configs/ovo_new.yaml", type=str)
    parser.add_argument(
        '--dataset_name',
        help="Dataset used. Choose either `Replica`, `ScanNet`")
    parser.add_argument(
        '--scenes_list',
        type=str,
        default=None,
        help="Path to a txt containing a scene name on each line. If set, `--scenes` is ignored. If neither `--scenes` nor `--scenes_list` are set, the scene list will be loaded from `data/working/config/<dataset_name>/<dataset_info_file>`")
    parser.add_argument(
        '--dataset_info_file',
        type=str,
        default="eval_info.yaml")
    parser.add_argument('--experiment_name', default=None, type=str)
    parser.add_argument(
        '--run',
        action='store_true',
        help="If set, compute the final metrics, after running OVO and segmenting.")
    parser.add_argument(
        '--build_hmsg',
        action='store_true',
        help="If set, build HMSG scene graph from OVO instance map after each --run scene.")
    parser.add_argument(
        '--hmsg_config_path',
        default=None,
        type=str,
        help="Optional HMSG config yaml path. If unset, an internal bridge config is used.")
    parser.add_argument(
        '--hmsg_obj_labels',
        default="SCANNET20",
        type=str,
        help="Object label set used by HMSG object naming.")
    parser.add_argument(
        '--hmsg_min_object_points',
        default=10,
        type=int,
        help="Minimum number of points for an OVO instance to be kept in HMSG.")
    parser.add_argument(
        '--segment',
        action='store_true',
        help="If set, use the reconstructed scene to segment the gt point-cloud, after running OVO.")
    parser.add_argument('--eval', action='store_true')
    args = parser.parse_args()
    main(args)
