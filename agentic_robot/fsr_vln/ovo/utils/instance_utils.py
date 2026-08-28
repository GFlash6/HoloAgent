import torch


def _aabb_gap(points1: torch.Tensor, points2: torch.Tensor) -> torch.Tensor:
    """Shortest distance between two axis-aligned 3D bounding boxes."""
    min1, max1 = points1.amin(dim=0), points1.amax(dim=0)
    min2, max2 = points2.amin(dim=0), points2.amax(dim=0)
    axis_gap = torch.maximum(min1 - max2, min2 - max1).clamp_min(0)
    return torch.linalg.vector_norm(axis_gap)


def same_instance(
        instance1,
        instance2,
        map_data,
        th_centroid=0.35,
        th_cossim=0.9,
        th_bbox_gap=0.15):
    """Match cross-view instances using appearance and depth-derived 3D pose.

    Separate views often observe different surfaces of the same object, so
    requiring dense point-to-point overlap is brittle.  Centroid distance and
    3D bounding-box gap retain a conservative spatial gate while CLIP features
    provide the visual gate.
    """
    points_3d, points_ids, points_ins_ids = map_data
    points1, points2 = points_3d[points_ins_ids ==
                                 instance1.id], points_3d[points_ins_ids == instance2.id]
    if points1.numel() == 0 or points2.numel() == 0:
        return False

    centroid_distance = torch.linalg.vector_norm(
        points1.mean(dim=0) - points2.mean(dim=0))
    if centroid_distance > th_centroid or _aabb_gap(points1, points2) > th_bbox_gap:
        return False

    if instance1.clip_feature is None or instance2.clip_feature is None:
        return False
    similarity = torch.nn.functional.cosine_similarity(
            instance1.clip_feature[0],
            instance2.clip_feature[0],
            dim=0)
    return bool(similarity >= th_cossim)


def fuse_instances(instance1, instance2, map_data):
    points_3d, points_ids, points_ins_ids = map_data

    instance1.add_points_ids(instance2.points_ids)
    for kf in instance2.kfs_ids:
        instance1.add_keyframes(kf)
    for (area, kf_id) in instance2.top_kf:
        instance1.add_top_kf(kf_id, area)
    points_ins_ids[points_ins_ids == instance2.id] = instance1.id
    return instance1, points_ins_ids
