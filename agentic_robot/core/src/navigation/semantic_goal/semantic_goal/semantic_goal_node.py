#!/usr/bin/env python3
"""Resolve semantic text through the live HMSG service and publish a Nav2 goal."""

from __future__ import annotations

import json
import math
import os
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def standoff_goal(
        robot_xy: np.ndarray,
        target_xy: np.ndarray,
        standoff_distance: float) -> tuple[np.ndarray, float]:
    delta = np.asarray(target_xy, dtype=np.float64) - np.asarray(
        robot_xy, dtype=np.float64)
    distance = float(np.linalg.norm(delta))
    if not math.isfinite(distance) or distance < 1e-6:
        raise ValueError("semantic target is at the current robot position")
    travel = max(0.0, distance - standoff_distance)
    return np.asarray(robot_xy, dtype=np.float64) + delta / distance * travel, math.atan2(
        delta[1], delta[0])


class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__("semantic_goal_node")
        self.query_url = os.getenv("HMSG_QUERY_URL", "http://127.0.0.1:8120/query")
        self.anchor_update_url = os.getenv(
            "HMSG_ANCHOR_UPDATE_URL", "http://127.0.0.1:8120/anchors/update")
        self.refine_url = os.getenv(
            "SEMANTIC_REFINE_URL", "http://127.0.0.1:8121/query")
        self.standoff_distance = float(os.getenv("SEMANTIC_STANDOFF_M", "0.7"))
        self.approach_standoff_distance = float(
            os.getenv("SEMANTIC_APPROACH_STANDOFF_M", "1.5"))
        self.max_pose_age = float(os.getenv("SEMANTIC_MAX_POSE_AGE", "1.0"))
        if (self.standoff_distance < 0 or self.approach_standoff_distance < 0
                or self.max_pose_age <= 0):
            raise ValueError("semantic standoff must be non-negative and pose age positive")

        self.latest_pose: np.ndarray | None = None
        self.latest_pose_received = 0.0
        self.stage = "IDLE"
        self.object_query = ""
        self.global_object_id = ""
        self.goal_publisher = self.create_publisher(PoseStamped, "/object_pose", 10)
        self.approach_publisher = self.create_publisher(
            PoseStamped, "/semantic_approach_pose", 10)
        self.status_publisher = self.create_publisher(String, "/waypoint_reached", 10)
        self.create_subscription(String, "/chat_loc_pub", self._semantic_callback, 10)
        self.create_subscription(String, "/waypoint_reached", self._status_callback, 10)
        self.create_subscription(
            String, "/semantic_approach_status", self._approach_status_callback, 10)
        self.create_subscription(String, "/chat_signal_pub", self._signal_callback, 10)
        self.create_subscription(Odometry, "/pose", self._pose_callback, 10)
        self.get_logger().info(f"Semantic HMSG query endpoint: {self.query_url}")

    def _pose_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.latest_pose = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ], dtype=np.float64)
        self.latest_pose_received = time.monotonic()

    def _publish_failure(self, detail: str) -> None:
        self.get_logger().error(detail)
        self.stage = "IDLE"
        message = String()
        message.data = "nav_failed"
        self.status_publisher.publish(message)

    @staticmethod
    def _post_json(url: str, payload: dict, timeout: float = 30.0) -> dict:
        http_request = Request(
            url=url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST")
        with urlopen(http_request, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))

    def _query_hmsg(self, object_query: str) -> dict:
        return self._post_json(self.query_url, {
            "object_query": object_query,
            "robot_map_pose_wxyz": self.latest_pose.tolist(),
        })

    @staticmethod
    def _validated_target(result: dict, label: str) -> np.ndarray:
        center = np.asarray(result.get("center_map"), dtype=np.float64)
        score = float(result.get("score", float("nan")))
        if center.shape != (3,) or not np.isfinite(center).all() or not math.isfinite(score):
            raise ValueError(f"{label} returned an invalid target")
        return center

    def _goal_for_target(self, target: np.ndarray, standoff: float) -> PoseStamped:
        goal_xy, yaw = standoff_goal(self.latest_pose[:2], target[:2], standoff)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(goal_xy[0])
        goal.pose.position.y = float(goal_xy[1])
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def _semantic_callback(self, message: String) -> None:
        if self.stage != "IDLE":
            self.get_logger().warning("Semantic navigation is already active")
            return
        parts = [part.strip() for part in message.data.split(",", maxsplit=2)]
        object_query = parts[-1] if parts else ""
        if not object_query:
            self._publish_failure("Semantic target has no object query")
            return
        if self.latest_pose is None or (
                time.monotonic() - self.latest_pose_received > self.max_pose_age):
            self._publish_failure("Cannot resolve semantic target without a fresh /pose")
            return

        try:
            result = self._query_hmsg(object_query)
            target = self._validated_target(result, "HMSG")
            goal = self._goal_for_target(target, self.approach_standoff_distance)
        except (HTTPError, URLError, TimeoutError, ValueError, KeyError, json.JSONDecodeError) as exc:
            self._publish_failure(f"Semantic HMSG query failed: {exc}")
            return

        self.object_query = object_query
        self.global_object_id = str(result["object_id"])
        self.stage = "COARSE_NAVIGATING"
        self.approach_publisher.publish(goal)
        self.get_logger().info(
            f"HMSG object={self.global_object_id} score={result['score']:.4f}; "
            f"approaching ({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f})")

    def _approach_status_callback(self, message: String) -> None:
        if self.stage != "COARSE_NAVIGATING" or message.data == "struck":
            return
        if message.data != "nav_finish":
            self._publish_failure(f"Semantic approach ended with {message.data}")
            return
        try:
            self.stage = "REFINING"
            refined = self._post_json(
                self.refine_url, {"object_query": self.object_query}, timeout=30.0)
            target = self._validated_target(refined, "online OVO")
            if refined.get("status") != "FOUND" or refined.get("frame_id") != "map":
                raise ValueError("online OVO target is not a FOUND map-frame result")
            self._post_json(self.anchor_update_url, {
                "object_id": self.global_object_id,
                "center_map": target.tolist(),
                "score": float(refined["score"]),
                "observation_count": int(refined["observation_count"]),
                "source_timestamp_ms": int(refined["source_timestamp_ms"]),
            })
            if self.latest_pose is None or (
                    time.monotonic() - self.latest_pose_received > self.max_pose_age):
                raise ValueError("cannot generate refined goal without a fresh /pose")
            goal = self._goal_for_target(target, self.standoff_distance)
            self.stage = "FINE_NAVIGATING"
            self.goal_publisher.publish(goal)
            self.get_logger().info(
                f"Persisted refined object={self.global_object_id}; "
                f"navigating to ({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f})")
        except (HTTPError, URLError, TimeoutError, ValueError, KeyError,
                json.JSONDecodeError) as exc:
            self._publish_failure(f"Semantic local refinement failed: {exc}")
        except Exception as exc:
            self._publish_failure(f"Unexpected semantic local refinement failure: {exc}")

    def _status_callback(self, message: String) -> None:
        if self.stage != "FINE_NAVIGATING":
            return
        if message.data in {"nav_finish", "nav_failed", "nav_canceled"}:
            self.stage = "IDLE"

    def _signal_callback(self, message: String) -> None:
        if message.data != "stop" or self.stage == "IDLE":
            return
        self.stage = "IDLE"


def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
