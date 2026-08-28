"""Expose stable ROS services while retaining HMSG/OVO HTTP internally."""

from __future__ import annotations

from urllib.error import HTTPError, URLError

import rclpy
from holoagent_interfaces.srv import QueryObject, UpdateObjectAnchor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .http_backend import SemanticBackendError, parse_query_result, post_json


class SemanticMapBridge(Node):
    def __init__(self) -> None:
        super().__init__("semantic_map_bridge")
        self.declare_parameter("hmsg_query_url", "http://127.0.0.1:8120/evidence")
        self.declare_parameter("hmsg_anchor_url", "http://127.0.0.1:8120/anchors/update")
        self.declare_parameter("ovo_query_url", "http://127.0.0.1:8121/evidence")
        self.declare_parameter("backend_mode", "online")
        self.declare_parameter("timeout_sec", 30.0)
        self.hmsg_query_url = self.get_parameter("hmsg_query_url").value
        self.hmsg_anchor_url = self.get_parameter("hmsg_anchor_url").value
        self.ovo_query_url = self.get_parameter("ovo_query_url").value
        self.backend_mode = str(self.get_parameter("backend_mode").value)
        self.timeout = float(self.get_parameter("timeout_sec").value)
        if self.backend_mode not in {"online", "prior"}:
            raise ValueError("backend_mode must be online or prior")
        if self.timeout <= 0.0:
            raise ValueError("timeout_sec must be positive")

        callbacks = ReentrantCallbackGroup()
        self.create_service(
            QueryObject,
            "semantic_map/query_object",
            self._query,
            callback_group=callbacks,
        )
        self.create_service(
            UpdateObjectAnchor,
            "semantic_map/update_anchor",
            self._update_anchor,
            callback_group=callbacks,
        )
        self.get_logger().info("Semantic ROS facade is ready")

    @staticmethod
    def _observer_payload(pose) -> list[float]:
        value = pose.pose
        return [
            value.position.x,
            value.position.y,
            value.position.z,
            value.orientation.w,
            value.orientation.x,
            value.orientation.y,
            value.orientation.z,
        ]

    def _query(self, request, response):
        try:
            payload = {"object_query": request.query}
            url = (
                self.hmsg_query_url
                if self.backend_mode == "prior" and not request.refine
                else self.ovo_query_url
            )
            if url == self.hmsg_query_url:
                payload["robot_map_pose_wxyz"] = self._observer_payload(
                    request.observer_pose)
            result = parse_query_result(post_json(url, payload, self.timeout))
            response.found = result["found"]
            response.object_id = result["object_id"]
            response.center.header.frame_id = "map"
            response.center.header.stamp = self.get_clock().now().to_msg()
            response.center.point.x, response.center.point.y, response.center.point.z = (
                result["center"]
            )
            response.score = result["score"]
            response.has_second_score = result["has_second_score"]
            response.second_score = result["second_score"]
            response.passes_backend_threshold = result["passes_backend_threshold"]
            response.backend_min_score = result["backend_min_score"]
            response.has_backend_min_margin = result["has_backend_min_margin"]
            response.backend_min_margin = result["backend_min_margin"]
            response.observation_count = result["observation_count"]
            timestamp_ms = result["source_timestamp_ms"]
            response.source_stamp.sec = timestamp_ms // 1000
            response.source_stamp.nanosec = (timestamp_ms % 1000) * 1_000_000
            response.detail = result["detail"]
        except (
            HTTPError,
            URLError,
            TimeoutError,
            ValueError,
            KeyError,
            SemanticBackendError,
        ) as exc:
            response.found = False
            response.detail = str(exc)
            self.get_logger().error(f"Semantic query failed: {exc}")
        return response

    def _update_anchor(self, request, response):
        if self.backend_mode == "online":
            response.accepted = True
            response.detail = "online OVO map owns incremental object tracks"
            return response
        try:
            payload = {
                "object_id": request.object_id,
                "center_map": [
                    request.center.point.x,
                    request.center.point.y,
                    request.center.point.z,
                ],
                "score": request.score,
                "observation_count": request.observation_count,
                "source_timestamp_ms": (
                    request.source_stamp.sec * 1000
                    + request.source_stamp.nanosec // 1_000_000
                ),
            }
            result = post_json(self.hmsg_anchor_url, payload, self.timeout)
            response.accepted = bool(result.get("success", result.get("accepted", True)))
            response.detail = str(result.get("detail", "anchor updated"))
        except (
            HTTPError,
            URLError,
            TimeoutError,
            ValueError,
            KeyError,
            SemanticBackendError,
        ) as exc:
            response.accepted = False
            response.detail = str(exc)
            self.get_logger().error(f"Semantic anchor update failed: {exc}")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMapBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
