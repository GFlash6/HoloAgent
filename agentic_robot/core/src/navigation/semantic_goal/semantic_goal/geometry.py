"""Geometry helpers shared by semantic navigation and unit tests."""

from __future__ import annotations

import math
from collections import deque

import numpy as np


def frontier_candidates(
    grid: np.ndarray,
    origin_xy: np.ndarray,
    resolution: float,
    robot_xy: np.ndarray,
    min_cluster_cells: int = 5,
    max_candidates: int = 8,
    visited_xy: list[np.ndarray] | None = None,
    revisit_radius_m: float = 0.75,
) -> list[dict]:
    """Return known-free frontier goals ranked by area gain and distance."""
    cells = np.asarray(grid)
    if cells.ndim != 2 or resolution <= 0 or min_cluster_cells < 1:
        raise ValueError("invalid occupancy grid or frontier parameters")
    free = (cells >= 0) & (cells <= 20)
    unknown = cells < 0
    adjacent_unknown = np.zeros_like(unknown)
    adjacent_unknown[1:, :] |= unknown[:-1, :]
    adjacent_unknown[:-1, :] |= unknown[1:, :]
    adjacent_unknown[:, 1:] |= unknown[:, :-1]
    adjacent_unknown[:, :-1] |= unknown[:, 1:]
    frontier = free & adjacent_unknown

    seen = np.zeros_like(frontier)
    candidates = []
    height, width = cells.shape
    for start_row, start_col in np.argwhere(frontier):
        if seen[start_row, start_col]:
            continue
        queue = deque([(int(start_row), int(start_col))])
        seen[start_row, start_col] = True
        cluster = []
        while queue:
            row, col = queue.popleft()
            cluster.append((row, col))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = row + dr, col + dc
                    if (
                        (dr or dc)
                        and 0 <= nr < height
                        and 0 <= nc < width
                        and frontier[nr, nc]
                        and not seen[nr, nc]
                    ):
                        seen[nr, nc] = True
                        queue.append((nr, nc))
        if len(cluster) < min_cluster_cells:
            continue
        cluster_array = np.asarray(cluster)
        center = cluster_array.mean(axis=0)
        row, col = cluster_array[
            np.linalg.norm(cluster_array - center, axis=1).argmin()
        ]
        xy = np.asarray(origin_xy, dtype=np.float64) + resolution * np.array(
            [col + 0.5, row + 0.5]
        )
        if any(
            np.linalg.norm(xy - old) < revisit_radius_m
            for old in visited_xy or []
        ):
            continue
        distance = float(np.linalg.norm(xy - np.asarray(robot_xy)[:2]))
        unknown_neighbors = {
            (nr, nc)
            for cluster_row, cluster_col in cluster
            for nr, nc in (
                (cluster_row - 1, cluster_col),
                (cluster_row + 1, cluster_col),
                (cluster_row, cluster_col - 1),
                (cluster_row, cluster_col + 1),
            )
            if 0 <= nr < height and 0 <= nc < width and unknown[nr, nc]
        }
        unknown_center = np.asarray(list(unknown_neighbors)).mean(axis=0)
        yaw = math.atan2(unknown_center[0] - row, unknown_center[1] - col)
        candidates.append({
            "xy": xy,
            "yaw": yaw,
            "gain_cells": len(cluster),
            "distance_m": distance,
            "utility": len(cluster) / (1.0 + distance),
        })
    return sorted(candidates, key=lambda item: item["utility"], reverse=True)[
        :max_candidates
    ]


def standoff_goal(
    robot_xy: np.ndarray,
    target_xy: np.ndarray,
    standoff_distance: float,
) -> tuple[np.ndarray, float]:
    delta = np.asarray(target_xy, dtype=np.float64) - np.asarray(
        robot_xy, dtype=np.float64
    )
    distance = float(np.linalg.norm(delta))
    if not math.isfinite(distance) or distance < 1e-6:
        raise ValueError("semantic target is at the current robot position")
    travel = max(0.0, distance - standoff_distance)
    return (
        np.asarray(robot_xy, dtype=np.float64) + delta / distance * travel,
        math.atan2(delta[1], delta[0]),
    )


def polyline_length(points) -> float:
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return 0.0
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points must have shape (N, 2)")
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def goal_error(robot_xy: np.ndarray, goal_xy: np.ndarray) -> float:
    """Return finite planar distance; reject malformed postcondition inputs."""
    robot = np.asarray(robot_xy, dtype=np.float64).reshape(-1)
    goal = np.asarray(goal_xy, dtype=np.float64).reshape(-1)
    if min(robot.size, goal.size) < 2:
        raise ValueError("robot/goal position must contain x and y")
    error = float(np.linalg.norm(robot[:2] - goal[:2]))
    if not math.isfinite(error):
        raise ValueError("robot/goal position is not finite")
    return error


def viewpoint_separation(
    observer_xy: np.ndarray,
    candidate_xy: np.ndarray,
    target_xy: np.ndarray,
) -> float:
    """Return the unsigned angular change in target viewpoint, in radians."""
    observer = np.asarray(observer_xy, dtype=np.float64) - target_xy
    candidate = np.asarray(candidate_xy, dtype=np.float64) - target_xy
    if min(np.linalg.norm(observer), np.linalg.norm(candidate)) < 1e-6:
        raise ValueError("viewpoint coincides with semantic target")
    first = math.atan2(observer[1], observer[0])
    second = math.atan2(candidate[1], candidate[0])
    return abs(math.atan2(math.sin(second - first), math.cos(second - first)))


def select_observation_candidate(
    results: list[dict], min_angle_rad: float, max_path_m: float
) -> int | None:
    """Select a useful viewpoint without exceeding the intervention path budget."""
    viable = [
        index for index, result in enumerate(results)
        if result.get("available")
        and result.get("path_exists")
        and result.get("path_length_m", math.inf) <= max_path_m
    ]
    if not viable:
        return None
    diverse = [
        index for index in viable
        if results[index]["viewpoint_separation_rad"] >= min_angle_rad
    ]
    if diverse:
        return min(diverse, key=lambda index: results[index]["path_length_m"])
    return max(
        viable,
        key=lambda index: (
            results[index]["viewpoint_separation_rad"],
            -results[index]["path_length_m"],
        ),
    )


def standoff_candidates(
    robot_xy: np.ndarray,
    target_xy: np.ndarray,
    standoff_distance: float,
    count: int,
) -> list[tuple[np.ndarray, float]]:
    if count < 1:
        raise ValueError("candidate count must be positive")
    first = standoff_goal(robot_xy, target_xy, standoff_distance)
    if count == 1:
        return [first]
    target = np.asarray(target_xy, dtype=np.float64)
    base = math.atan2(robot_xy[1] - target[1], robot_xy[0] - target[0])
    candidates = [first]
    for index in range(1, count):
        angle = base + 2.0 * math.pi * index / count
        point = target + standoff_distance * np.array(
            [math.cos(angle), math.sin(angle)]
        )
        candidates.append((point, math.atan2(target[1] - point[1], target[0] - point[0])))
    return candidates
