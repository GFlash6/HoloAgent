"""Geometry helpers shared by semantic navigation and unit tests."""

from __future__ import annotations

import math

import numpy as np


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
