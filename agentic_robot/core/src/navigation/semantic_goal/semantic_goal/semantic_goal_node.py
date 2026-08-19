#!/usr/bin/env python3
"""Two-stage semantic navigation with a stable ROS action interface."""

from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from holoagent_interfaces.action import NavigateToObject
from holoagent_interfaces.srv import QueryObject, UpdateObjectAnchor
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String

from .geometry import standoff_goal


TERMINAL_NAV_STATES = {"nav_finish", "nav_failed", "nav_canceled"}


class SemanticNavigator(Node):
    """Coordinate HMSG lookup, local refinement, and Nav2 goal execution."""

    def __init__(self) -> None:
        super().__init__("semantic_navigator")
        self.declare_parameter("coarse_standoff_m", 1.5)
        self.declare_parameter("fine_standoff_m", 0.7)
        self.declare_parameter("max_pose_age_sec", 1.0)
        self.declare_parameter("service_timeout_sec", 30.0)
        self.declare_parameter("navigation_timeout_sec", 300.0)

        self.coarse_standoff = float(self.get_parameter("coarse_standoff_m").value)
        self.fine_standoff = float(self.get_parameter("fine_standoff_m").value)
        self.max_pose_age = float(self.get_parameter("max_pose_age_sec").value)
        self.service_timeout = float(self.get_parameter("service_timeout_sec").value)
        self.navigation_timeout = float(
            self.get_parameter("navigation_timeout_sec").value
        )
        if (
            min(self.coarse_standoff, self.fine_standoff) < 0.0
            or min(self.max_pose_age, self.service_timeout, self.navigation_timeout)
            <= 0.0
        ):
            raise ValueError("semantic navigation distances and timeouts are invalid")

        callbacks = ReentrantCallbackGroup()
        self.query_client = self.create_client(
            QueryObject, "semantic_map/query_object", callback_group=callbacks
        )
        self.anchor_client = self.create_client(
            UpdateObjectAnchor,
            "semantic_map/update_anchor",
            callback_group=callbacks,
        )
        self.coarse_goal_publisher = self.create_publisher(
            PoseStamped, "navigation/semantic/coarse_goal", 10
        )
        self.fine_goal_publisher = self.create_publisher(
            PoseStamped, "navigation/goal_pose", 10
        )
        self.cancel_publisher = self.create_publisher(Empty, "navigation/cancel", 10)
        self.legacy_status_publisher = self.create_publisher(
            String, "waypoint_reached", 10
        )
        self.create_subscription(
            Odometry,
            "localization/odom",
            self._pose_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            String,
            "navigation/semantic/coarse_status",
            self._coarse_status_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            String,
            "navigation/goal_status",
            self._fine_status_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            String,
            "chat_loc_pub",
            self._legacy_query_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            String,
            "chat_signal_pub",
            self._legacy_signal_callback,
            10,
            callback_group=callbacks,
        )

        self.latest_pose: np.ndarray | None = None
        self.latest_pose_received = 0.0
        self._status_condition = threading.Condition()
        self._coarse_status = ""
        self._fine_status = ""
        self._active_lock = threading.Lock()
        self._goal_active = False
        self._legacy_goal_handle = None

        self.action_server = ActionServer(
            self,
            NavigateToObject,
            "navigation/navigate_to_object",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callbacks,
        )
        self.legacy_action_client = ActionClient(
            self,
            NavigateToObject,
            "navigation/navigate_to_object",
            callback_group=callbacks,
        )
        self.get_logger().info("Semantic navigation action is ready")

    def _pose_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.latest_pose = np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ],
            dtype=np.float64,
        )
        self.latest_pose_received = time.monotonic()

    def _coarse_status_callback(self, message: String) -> None:
        with self._status_condition:
            self._coarse_status = message.data
            self._status_condition.notify_all()

    def _fine_status_callback(self, message: String) -> None:
        with self._status_condition:
            self._fine_status = message.data
            self._status_condition.notify_all()

    def _goal_callback(self, goal_request) -> GoalResponse:
        if not goal_request.query.strip():
            return GoalResponse.REJECT
        with self._active_lock:
            if self._goal_active:
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _fresh_pose(self) -> bool:
        return self.latest_pose is not None and (
            time.monotonic() - self.latest_pose_received <= self.max_pose_age
        )

    def _observer_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.latest_pose[0])
        pose.pose.position.y = float(self.latest_pose[1])
        pose.pose.position.z = float(self.latest_pose[2])
        pose.pose.orientation.w = float(self.latest_pose[3])
        pose.pose.orientation.x = float(self.latest_pose[4])
        pose.pose.orientation.y = float(self.latest_pose[5])
        pose.pose.orientation.z = float(self.latest_pose[6])
        return pose

    def _wait_future(self, future, goal_handle, timeout: float):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if goal_handle.is_cancel_requested:
                self.cancel_publisher.publish(Empty())
                raise InterruptedError("semantic navigation canceled")
            if time.monotonic() >= deadline:
                raise TimeoutError("ROS service timed out")
            time.sleep(0.02)
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def _query(self, query: str, refine: bool, goal_handle):
        if not self.query_client.wait_for_service(timeout_sec=self.service_timeout):
            raise TimeoutError("semantic_map/query_object is unavailable")
        request = QueryObject.Request()
        request.query = query
        request.observer_pose = self._observer_pose()
        request.refine = refine
        response = self._wait_future(
            self.query_client.call_async(request), goal_handle, self.service_timeout
        )
        if not response.found or response.center.header.frame_id != "map":
            raise ValueError(response.detail or "semantic object was not found in map")
        return response

    def _update_anchor(self, object_id: str, query_result, goal_handle) -> None:
        if not self.anchor_client.wait_for_service(timeout_sec=self.service_timeout):
            raise TimeoutError("semantic_map/update_anchor is unavailable")
        request = UpdateObjectAnchor.Request()
        request.object_id = object_id
        request.center = query_result.center
        request.score = query_result.score
        request.observation_count = query_result.observation_count
        request.source_stamp = query_result.source_stamp
        response = self._wait_future(
            self.anchor_client.call_async(request), goal_handle, self.service_timeout
        )
        if not response.accepted:
            raise RuntimeError(response.detail or "semantic anchor update was rejected")

    def _goal_for_target(self, target, standoff: float) -> PoseStamped:
        goal_xy, yaw = standoff_goal(
            self.latest_pose[:2],
            np.array([target.point.x, target.point.y]),
            standoff,
        )
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(goal_xy[0])
        goal.pose.position.y = float(goal_xy[1])
        goal.pose.orientation.z = float(np.sin(yaw / 2.0))
        goal.pose.orientation.w = float(np.cos(yaw / 2.0))
        return goal

    @staticmethod
    def _publish_feedback(goal_handle, phase: str, score: float, goal) -> None:
        feedback = NavigateToObject.Feedback()
        feedback.phase = phase
        feedback.score = score
        feedback.current_goal = goal
        goal_handle.publish_feedback(feedback)

    def _wait_navigation(self, goal_handle, coarse: bool) -> str:
        attr = "_coarse_status" if coarse else "_fine_status"
        deadline = time.monotonic() + self.navigation_timeout
        with self._status_condition:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.cancel_publisher.publish(Empty())
                    return "nav_canceled"
                status = getattr(self, attr)
                if status in TERMINAL_NAV_STATES:
                    return status
                if time.monotonic() >= deadline:
                    self.cancel_publisher.publish(Empty())
                    return "nav_failed"
                self._status_condition.wait(timeout=0.1)
        return "nav_failed"

    @staticmethod
    def _result(success: bool, object_id: str, pose, detail: str):
        result = NavigateToObject.Result()
        result.success = success
        result.object_id = object_id
        if pose is not None:
            result.final_pose = pose
        result.detail = detail
        return result

    def _execute(self, goal_handle):
        object_id = ""
        final_goal = None
        try:
            if not self._fresh_pose():
                raise RuntimeError("localization/odom is missing or stale")
            request = goal_handle.request
            object_query = request.query.split(",", maxsplit=2)[-1].strip()
            coarse_standoff = (
                request.coarse_standoff
                if request.coarse_standoff > 0.0
                else self.coarse_standoff
            )
            fine_standoff = (
                request.fine_standoff
                if request.fine_standoff > 0.0
                else self.fine_standoff
            )

            coarse = self._query(object_query, False, goal_handle)
            object_id = coarse.object_id
            coarse_goal = self._goal_for_target(coarse.center, coarse_standoff)
            self._publish_feedback(
                goal_handle, "COARSE_NAVIGATING", coarse.score, coarse_goal
            )
            with self._status_condition:
                self._coarse_status = ""
            self.coarse_goal_publisher.publish(coarse_goal)
            status = self._wait_navigation(goal_handle, coarse=True)
            if status != "nav_finish":
                if status == "nav_canceled":
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(False, object_id, coarse_goal, status)

            self._publish_feedback(goal_handle, "REFINING", coarse.score, coarse_goal)
            refined = self._query(object_query, True, goal_handle)
            self._update_anchor(object_id, refined, goal_handle)
            if not self._fresh_pose():
                raise RuntimeError("localization/odom became stale before fine navigation")

            final_goal = self._goal_for_target(refined.center, fine_standoff)
            self._publish_feedback(
                goal_handle, "FINE_NAVIGATING", refined.score, final_goal
            )
            with self._status_condition:
                self._fine_status = ""
            self.fine_goal_publisher.publish(final_goal)
            status = self._wait_navigation(goal_handle, coarse=False)
            if status == "nav_finish":
                goal_handle.succeed()
                return self._result(True, object_id, final_goal, "completed")
            if status == "nav_canceled":
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return self._result(False, object_id, final_goal, status)
        except InterruptedError as exc:
            goal_handle.canceled()
            return self._result(False, object_id, final_goal, str(exc))
        except Exception as exc:
            self.get_logger().error(f"Semantic navigation failed: {exc}")
            goal_handle.abort()
            return self._result(False, object_id, final_goal, str(exc))
        finally:
            with self._active_lock:
                self._goal_active = False

    def _legacy_query_callback(self, message: String) -> None:
        query = message.data.split(",", maxsplit=2)[-1].strip()
        if not query:
            self._publish_legacy_status("nav_failed")
            return
        goal = NavigateToObject.Goal()
        goal.query = query
        future = self.legacy_action_client.send_goal_async(goal)
        future.add_done_callback(self._legacy_goal_response)

    def _legacy_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._publish_legacy_status("nav_failed")
            return
        self._legacy_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._legacy_result)

    def _legacy_result(self, future) -> None:
        result = future.result().result
        self._legacy_goal_handle = None
        self._publish_legacy_status("nav_finish" if result.success else "nav_failed")

    def _legacy_signal_callback(self, message: String) -> None:
        if message.data == "stop" and self._legacy_goal_handle is not None:
            self._legacy_goal_handle.cancel_goal_async()

    def _publish_legacy_status(self, status: str) -> None:
        message = String()
        message.data = status
        self.legacy_status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.action_server.destroy()
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
