import argparse
from pathlib import Path
import torch
import numpy as np
import open3d as o3d
import json  # 导入json库
from typing import List
# --- 解决方案开始 ---
# import sys
# # 将项目根目录添加到Python的搜索路径中
# sys.path.append("${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/mapvln/ovo_slam")
# # 假设 run_eval.py 在同一目录下或在 Python 路径中
from run_ovo_mapping import load_representation


def retrieve_and_save_top_k_instances(
    semantic_module,
    params: dict,
    run_path: Path,
    query_texts: List[str],
    k: int = 5
):
    """
    根据一个或多个文本查询检索最相关的k个实例，并将其点云保存到文件。

    Args:
        semantic_module: 已加载的包含 .query 和 .objects 的语义模块。
        params (dict): 包含 "xyz", "obj_ids", "color" 的已加载参数字典。
        run_path (Path): 原始运行的路径，用于确定输出目录。
        query_texts (List[str]): 用于查询的文本字符串列表。
        k (int, optional): 每个查询要检索和保存的顶部实例数量。默认为 5。
    """
    # 5. 准备点云数据 (从加载的 params 中获取) - 只需执行一次
    pcd_pred = params["xyz"]
    pcd_obj_ids = params["obj_ids"].squeeze().numpy().astype(np.int32)
    sh_c0 = 0.28209479177387814
    if params.get("features_dc", None) is not None:
        pcd_colors = (params["features_dc"] * sh_c0 + 0.5).clip(0, 1).numpy()
    elif params.get("color") is not None:
        pcd_colors = params["color"].numpy() / 255.0
    else:
        pcd_colors = np.random.rand(*pcd_pred.shape)

    # 获取与分数对应的对象ID列表 - 只需执行一次
    object_ids_in_order = list(semantic_module.objects.keys())
    if not object_ids_in_order:
        print("No objects found to query.")
        return

    # 为每个查询文本执行检索和保存
    for query_text in query_texts:
        print(
            f"\n--- Retrieving top {k} instances for query: '{query_text}' ---")

        # 1. 获取所有实例的查询分数
        relevance_map = semantic_module.query([query_text]).squeeze()

        # 3. 找到Top-k的分数和索引
        num_objects = len(object_ids_in_order)
        actual_k = min(k, num_objects)
        if actual_k == 0:
            print("  No objects to rank for this query.")
            continue
        top_k_scores, top_k_indices = torch.topk(relevance_map, k=actual_k)

        # 4. 获取Top-k的对象ID
        top_k_object_ids = [object_ids_in_order[i] for i in top_k_indices]

        # 6. 为每个Top-k实例创建并保存点云
        instance_dir = run_path / "top_k_instances_queried"
        instance_dir.mkdir(exist_ok=True, parents=True)

        for i, obj_id in enumerate(top_k_object_ids):
            score = top_k_scores[i].item()
            print(
                f"  - Saving Top-{i+1}: Object ID {obj_id} (Score: {score:.4f})")

            # 创建掩码以选择属于此对象的点
            mask = (pcd_obj_ids == obj_id)

            if not np.any(mask):
                print(f"    Warning: No points found for Object ID {obj_id}.")
                continue

            # 提取该对象的点和颜色
            instance_points = pcd_pred[mask]
            instance_colors = pcd_colors[mask]

            # 对单个实例点云进行空间聚类以过滤离群点
            if len(instance_points) > 50:
                pcd_instance = o3d.geometry.PointCloud()
                pcd_instance.points = o3d.utility.Vector3dVector(
                    instance_points)
                labels = np.array(
                    pcd_instance.cluster_dbscan(
                        eps=0.05,
                        min_points=10,
                        print_progress=False))
                if labels.max() != -1:
                    counts = np.bincount(labels[labels != -1])
                    largest_cluster_label = counts.argmax()
                    cluster_mask = (labels == largest_cluster_label)
                    original_point_count = len(instance_points)
                    instance_points = instance_points[cluster_mask]
                    instance_colors = instance_colors[cluster_mask]
                    print(
                        f"    Filtered instance points: {original_point_count} -> {len(instance_points)}")

            # 创建Open3D点云对象，颜色统一为深绿色
            instance_pcd = o3d.geometry.PointCloud()
            instance_pcd.points = o3d.utility.Vector3dVector(instance_points)
            deep_green = np.array(
                [0.0, 0.5, 0.0], dtype=float)  # 深绿色 RGB (0-1)
            instance_pcd.colors = o3d.utility.Vector3dVector(
                np.tile(deep_green, (len(instance_points), 1)))

            # 保存文件
            query_sanitized = "".join(
                c if c.isalnum() else "_" for c in query_text)
            out_path = instance_dir / \
                f"top_{i+1}_{query_sanitized}_obj_{obj_id}.ply"
            o3d.io.write_point_cloud(str(out_path), instance_pcd)
            print(f"    Saved to {out_path}")
# ...existing code...


def retrieve_and_save_top_k_instances_new(
    semantic_module,
    params: dict,
    run_path: Path,
    query_texts: List[str],
    k: int = 5,
    score_threshold: float = 0.85  # 用于判断是否重排序的阈值
):
    """
    根据一个或多个文本查询检索最相关的k个实例，并将其点云保存到文件。
    逻辑：
    1. 先按相似度分数取 Top-K。
    2. 对于这 Top-K 个结果：
       - 如果分数 > score_threshold，则认为匹配可信，按点云数量从大到小重排序（优先选大的完整物体）。
       - 如果分数 <= score_threshold，保持按分数排序（优先选匹配度高的）。

    Args:
        semantic_module: 已加载的包含 .query 和 .objects 的语义模块。
        params (dict): 包含 "xyz", "obj_ids", "color" 的已加载参数字典。
        run_path (Path): 原始运行的路径，用于确定输出目录。
        query_texts (List[str]): 用于查询的文本字符串列表。
        k (int, optional): 每个查询要检索和保存的顶部实例数量。默认为 5。
        score_threshold (float, optional): 重排序阈值。
    """
    # 5. 准备点云数据 (从加载的 params 中获取) - 只需执行一次
    pcd_pred = params["xyz"]
    pcd_obj_ids = params["obj_ids"].squeeze().numpy().astype(np.int32)
    sh_c0 = 0.28209479177387814
    if params.get("features_dc", None) is not None:
        pcd_colors = (params["features_dc"] * sh_c0 + 0.5).clip(0, 1).numpy()
    elif params.get("color") is not None:
        pcd_colors = params["color"].numpy() / 255.0
    else:
        pcd_colors = np.random.rand(*pcd_pred.shape)

    # 获取与分数对应的对象ID列表 - 只需执行一次
    object_ids_in_order = list(semantic_module.objects.keys())
    if not object_ids_in_order:
        print("No objects found to query.")
        return

    # 为每个查询文本执行检索和保存
    for query_text in query_texts:
        print(
            f"\n--- Retrieving top {k} instances for query: '{query_text}' (Re-sort Threshold: {score_threshold}) ---")

        # 1. 获取所有实例的查询分数
        relevance_map = semantic_module.query([query_text]).squeeze()

        # 2. 找到Top-k的分数和索引 (原始按分数排序)
        num_objects = len(object_ids_in_order)
        actual_k = min(k, num_objects)
        if actual_k == 0:
            print("  No objects to rank for this query.")
            continue

        top_k_scores, top_k_indices = torch.topk(relevance_map, k=actual_k)

        # 3. 收集 Top-K 实例的详细信息
        candidates = []
        for i in range(actual_k):
            idx = top_k_indices[i].item()
            obj_id = object_ids_in_order[idx]
            score = top_k_scores[i].item()

            # 统计点数
            mask = (pcd_obj_ids == obj_id)
            num_points = np.sum(mask)

            if num_points > 0:
                candidates.append({
                    'id': obj_id,
                    'score': score,
                    'num_points': num_points,
                    'mask': mask
                })

        if not candidates:
            print("  No valid candidates found in Top-K.")
            continue

        # 4. 混合排序逻辑
        # 分离出高分候选者和低分候选者
        high_score_candidates = [
            c for c in candidates if c['score'] > score_threshold]
        low_score_candidates = [
            c for c in candidates if c['score'] <= score_threshold]

        # 高分候选者：按点云数量降序重排序
        high_score_candidates.sort(key=lambda x: x['num_points'], reverse=True)

        # 低分候选者：保持按分数降序排序 (原本就是按分数排的，这里为了保险再排一次)
        low_score_candidates.sort(key=lambda x: x['score'], reverse=True)

        # 合并列表：高分且大的排最前，低分的排后面
        final_candidates = high_score_candidates + low_score_candidates

        # 5. 为每个排序后的实例创建并保存点云
        instance_dir = run_path / "top_k_instances_queried"
        instance_dir.mkdir(exist_ok=True, parents=True)

        for i, candidate in enumerate(final_candidates):
            obj_id = candidate['id']
            score = candidate['score']
            num_points = candidate['num_points']
            mask = candidate['mask']

            is_resorted = score > score_threshold
            sort_method = "Size" if is_resorted else "Score"

            print(
                f"  - Saving Rank-{i+1} ({sort_method}): Object ID {obj_id} (Score: {score:.4f}, Points: {num_points})")

            # 提取该对象的点和颜色
            instance_points = pcd_pred[mask]
            instance_colors = pcd_colors[mask]

            # 对单个实例点云进行空间聚类以过滤离群点
            if len(instance_points) > 50:
                pcd_instance = o3d.geometry.PointCloud()
                pcd_instance.points = o3d.utility.Vector3dVector(
                    instance_points)
                labels = np.array(
                    pcd_instance.cluster_dbscan(
                        eps=0.05,
                        min_points=10,
                        print_progress=False))
                if labels.max() != -1:
                    counts = np.bincount(labels[labels != -1])
                    largest_cluster_label = counts.argmax()
                    cluster_mask = (labels == largest_cluster_label)
                    original_point_count = len(instance_points)
                    instance_points = instance_points[cluster_mask]
                    instance_colors = instance_colors[cluster_mask]
                    print(
                        f"    Filtered instance points: {original_point_count} -> {len(instance_points)}")

            # 创建Open3D点云对象，颜色统一为深绿色
            instance_pcd = o3d.geometry.PointCloud()
            instance_pcd.points = o3d.utility.Vector3dVector(instance_points)
            deep_green = np.array(
                [0.0, 0.5, 0.0], dtype=float)  # 深绿色 RGB (0-1)
            instance_pcd.colors = o3d.utility.Vector3dVector(
                np.tile(deep_green, (len(instance_points), 1)))

            # 保存文件
            query_sanitized = "".join(
                c if c.isalnum() else "_" for c in query_text)
            # 文件名格式: rank_query_objID_score_points.ply
            out_path = instance_dir / \
                f"rank_{i+1}_{query_sanitized}_obj_{obj_id}_s{score:.2f}_p{num_points}.ply"
            o3d.io.write_point_cloud(str(out_path), instance_pcd)
            print(f"    Saved to {out_path}")
# ...existing code...


def main():
    parser = argparse.ArgumentParser(
        description="Query a pre-built OVO map using a list of objects from a JSON file.")
    parser.add_argument(
        '--run_path',
        type=str,
        required=True,
        help="Path to the completed experiment run directory (e.g., 'data/output/G1/ovo_mapping/sh_3f').")
    parser.add_argument(
        '--object_file',
        type=str,
        default='object.json',
        help="Path to the JSON file containing a list of object names to query.")
    parser.add_argument(
        '--k',
        type=int,
        default=5,
        help="Number of top instances to retrieve for each query.")
    # parser.add_argument('--working_dir', default="${HOLOAGENT_DATA_ROOT}/VLN/instance_tracking/mapvln/ovo_slam", type=str, help="The root working directory of the project.")
    args = parser.parse_args()

    # --- 从JSON文件加载查询列表 ---
    # object_file_path = Path(args.working_dir) / args.object_file
    object_file_path = Path(args.object_file)
    try:
        with open(object_file_path, 'r') as f:
            query_texts = json.load(f)
        if not isinstance(query_texts, list):
            raise ValueError("JSON file should contain a list of strings.")
        print(f"Loaded {len(query_texts)} queries from {object_file_path}")
    except FileNotFoundError:
        print(f"Error: Object file not found at {object_file_path}")
        return
    except json.JSONDecodeError:
        print(f"Error: Could not decode JSON from {object_file_path}")
        return
    except ValueError as e:
        print(f"Error: {e}")
        return
    # --- 加载完成 ---

    # run_path = Path(args.working_dir) / args.run_path
    run_path = Path(args.run_path)

    print(f"Loading representation from: {run_path}")
    semantic_module, params = load_representation(run_path, eval=True)

    retrieve_and_save_top_k_instances_new(
        semantic_module=semantic_module,
        params=params,
        run_path=run_path,
        query_texts=query_texts,  # 使用从JSON文件加载的列表
        k=args.k
    )
    print("\nQuery and save process finished.")


if __name__ == "__main__":
    main()
