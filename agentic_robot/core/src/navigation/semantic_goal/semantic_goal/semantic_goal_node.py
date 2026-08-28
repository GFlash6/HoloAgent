#!/usr/bin/env python3
"""Two-stage semantic navigation with a stable ROS action interface."""

from __future__ import annotations

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from holoagent_interfaces.action import NavigateToObject
from holoagent_interfaces.msg import LocalizationStatus, NavigationEvidence
from holoagent_interfaces.srv import QueryObject, UpdateObjectAnchor
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

from .contracts import ContractGate, ProgressStallMonitor
from .geometry import (
    frontier_candidates,
    goal_error,
    polyline_length,
    select_observation_candidate,
    standoff_candidates,
    viewpoint_separation,
)
from .trace import JsonlTrace


TERMINAL_NAV_STATES = {"nav_finish", "nav_failed", "nav_canceled"}


class SemanticEvidenceError(RuntimeError):
    def __init__(self, phase: str, detail: str) -> None:
        super().__init__(detail)
        self.phase = phase


class ContractRejectedError(RuntimeError):
    def __init__(self, decision: dict) -> None:
        self.decision = decision
        super().__init__(
            f"contract rejected {decision['stage']}: "
            + ", ".join(result["name"] for result in decision["failed"])
        )


class LocalizationEvidenceError(RuntimeError):
    pass


class GeometryEvidenceError(RuntimeError):
    def __init__(self, phase: str, detail: str) -> None:
        self.phase = phase
        super().__init__(detail)


class ExecutionEvidenceError(RuntimeError):
    pass


class SemanticNavigator(Node):
    """Coordinate HMSG lookup, local refinement, and Nav2 goal execution."""

    def __init__(self) -> None:
        super().__init__("semantic_navigator")
        self.declare_parameter("coarse_standoff_m", 1.5)
        self.declare_parameter("fine_standoff_m", 0.7)
        self.declare_parameter("max_pose_age_sec", 1.0)
        self.declare_parameter("service_timeout_sec", 30.0)
        self.declare_parameter("navigation_timeout_sec", 300.0)
        self.declare_parameter("planner_probe_timeout_sec", 10.0)
        self.declare_parameter("trace_path", "")
        self.declare_parameter("contract_policy_path", "")
        self.declare_parameter("require_localization_tracking", True)
        self.declare_parameter("require_plannable_goal", True)
        self.declare_parameter("require_backend_threshold", True)
        self.declare_parameter("require_completion_confirmation", False)
        self.declare_parameter("completion_goal_tolerance_m", 0.5)
        self.declare_parameter("goal_candidate_count", 1)
        self.declare_parameter("max_observation_retries", 0)
        self.declare_parameter("observation_standoff_m", 1.2)
        self.declare_parameter("observation_min_angle_deg", 60.0)
        self.declare_parameter("observation_max_path_m", 2.0)
        self.declare_parameter("execution_stall_timeout_sec", 0.0)
        self.declare_parameter("execution_min_progress_m", 0.05)
        self.declare_parameter("max_execution_recoveries", 3)
        self.declare_parameter("exploration_max_steps", 8)
        self.declare_parameter("exploration_timeout_sec", 300.0)
        self.declare_parameter("exploration_max_path_m", 8.0)
        self.declare_parameter("frontier_candidate_count", 8)
        self.declare_parameter("frontier_min_cluster_cells", 5)
        self.declare_parameter("frontier_revisit_radius_m", 0.75)
        self.declare_parameter("exploration_observation_wait_sec", 1.0)

        self.coarse_standoff = float(self.get_parameter("coarse_standoff_m").value)
        self.fine_standoff = float(self.get_parameter("fine_standoff_m").value)
        self.max_pose_age = float(self.get_parameter("max_pose_age_sec").value)
        self.service_timeout = float(self.get_parameter("service_timeout_sec").value)
        self.navigation_timeout = float(
            self.get_parameter("navigation_timeout_sec").value
        )
        self.execution_stall_timeout = float(
            self.get_parameter("execution_stall_timeout_sec").value
        )
        self.execution_min_progress = float(
            self.get_parameter("execution_min_progress_m").value
        )
        self.max_execution_recoveries = int(
            self.get_parameter("max_execution_recoveries").value
        )
        self.exploration_max_steps = int(
            self.get_parameter("exploration_max_steps").value
        )
        self.exploration_timeout = float(
            self.get_parameter("exploration_timeout_sec").value
        )
        self.exploration_max_path = float(
            self.get_parameter("exploration_max_path_m").value
        )
        self.frontier_candidate_count = int(
            self.get_parameter("frontier_candidate_count").value
        )
        self.frontier_min_cluster_cells = int(
            self.get_parameter("frontier_min_cluster_cells").value
        )
        self.frontier_revisit_radius = float(
            self.get_parameter("frontier_revisit_radius_m").value
        )
        self.exploration_observation_wait = float(
            self.get_parameter("exploration_observation_wait_sec").value
        )
        self.planner_probe_timeout = float(
            self.get_parameter("planner_probe_timeout_sec").value
        )
        if (
            min(self.coarse_standoff, self.fine_standoff) < 0.0
            or min(
                self.max_pose_age,
                self.service_timeout,
                self.navigation_timeout,
                self.planner_probe_timeout,
            )
            <= 0.0
        ):
            raise ValueError("semantic navigation distances and timeouts are invalid")
        if self.execution_stall_timeout < 0 or self.execution_min_progress <= 0:
            raise ValueError("execution stall parameters are invalid")
        if (
            self.max_execution_recoveries < 0
            or self.exploration_max_steps < 0
            or self.exploration_timeout <= 0
            or self.exploration_max_path <= 0
            or not 1 <= self.frontier_candidate_count <= 32
            or self.frontier_min_cluster_cells < 1
            or self.frontier_revisit_radius <= 0
            or self.exploration_observation_wait < 0
        ):
            raise ValueError("exploration and recovery limits are invalid")
        self.trace = JsonlTrace(
            str(self.get_parameter("trace_path").value),
            ros_time_ns=lambda: self.get_clock().now().nanoseconds,
        )
        self.contract_gate = ContractGate(
            str(self.get_parameter("contract_policy_path").value)
        )
        self.require_localization_tracking = bool(
            self.get_parameter("require_localization_tracking").value
        )
        self.require_plannable_goal = bool(
            self.get_parameter("require_plannable_goal").value
        )
        self.require_backend_threshold = bool(
            self.get_parameter("require_backend_threshold").value
        )
        self.require_completion_confirmation = bool(
            self.get_parameter("require_completion_confirmation").value
        )
        self.completion_goal_tolerance = float(
            self.get_parameter("completion_goal_tolerance_m").value
        )
        self.goal_candidate_count = int(
            self.get_parameter("goal_candidate_count").value
        )
        self.max_observation_retries = int(
            self.get_parameter("max_observation_retries").value
        )
        self.observation_standoff = float(
            self.get_parameter("observation_standoff_m").value
        )
        self.observation_min_angle = math.radians(float(
            self.get_parameter("observation_min_angle_deg").value
        ))
        self.observation_max_path = float(
            self.get_parameter("observation_max_path_m").value
        )
        if not 1 <= self.goal_candidate_count <= 16:
            raise ValueError("goal_candidate_count must be between 1 and 16")
        if not 0 <= self.max_observation_retries <= 3:
            raise ValueError("max_observation_retries must be between 0 and 3")
        if (
            self.observation_standoff <= 0.0
            or self.completion_goal_tolerance <= 0.0
            or self.observation_max_path <= 0.0
            or not 0.0 <= self.observation_min_angle <= math.pi
        ):
            raise ValueError("observation viewpoint parameters are invalid")

        callbacks = ReentrantCallbackGroup()
        self.query_client = self.create_client(
            QueryObject, "semantic_map/query_object", callback_group=callbacks
        )
        self.anchor_client = self.create_client(
            UpdateObjectAnchor,
            "semantic_map/update_anchor",
            callback_group=callbacks,
        )
        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            "compute_path_to_pose",
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
        self.readiness_publisher = self.create_publisher(
            String, "navigation/semantic/readiness", 10
        )
        self.create_subscription(
            Odometry,
            "localization/odom",
            self._pose_callback,
            10,
            callback_group=callbacks,
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            "map",
            self._map_callback,
            map_qos,
            callback_group=callbacks,
        )
        self.create_subscription(
            LocalizationStatus,
            "localization/status",
            self._localization_status_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            NavigationEvidence,
            "navigation/evidence",
            self._navigation_evidence_callback,
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
        self.latest_localization_status = None
        self.latest_map = None
        self._map_lock = threading.Lock()
        self._status_condition = threading.Condition()
        self._coarse_status = ""
        self._fine_status = ""
        self._active_lock = threading.Lock()
        self._goal_active = False
        self._legacy_goal_handle = None
        self._trace_task_id = ""
        self._latest_navigation_evidence = {}
        self.create_timer(0.2, self._publish_readiness, callback_group=callbacks)

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

    def _publish_readiness(self) -> None:
        status = self.latest_localization_status
        ready = (
            self._fresh_pose()
            and status is not None
            and status["localized"]
            and time.monotonic() - status["received_monotonic"] <= self.max_pose_age
        )
        message = String()
        message.data = "READY" if ready else "WAITING_FOR_FRESH_TRACKING_POSE"
        self.readiness_publisher.publish(message)

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

    def _localization_status_callback(self, message: LocalizationStatus) -> None:
        self.latest_localization_status = {
            "localized": bool(message.localized),
            "fitness_score": float(message.fitness_score),
            "consecutive_failures": int(message.consecutive_failures),
            "state": message.state,
            "detail": message.detail,
            "received_monotonic": time.monotonic(),
        }

    def _map_callback(self, message: OccupancyGrid) -> None:
        expected = int(message.info.width) * int(message.info.height)
        if expected <= 0 or len(message.data) != expected:
            return
        snapshot = {
            "grid": np.asarray(message.data, dtype=np.int16).reshape(
                int(message.info.height), int(message.info.width)
            ),
            "origin": np.array(
                [message.info.origin.position.x, message.info.origin.position.y],
                dtype=np.float64,
            ),
            "resolution": float(message.info.resolution),
            "received_monotonic": time.monotonic(),
        }
        with self._map_lock:
            self.latest_map = snapshot

    def _navigation_evidence_callback(self, message: NavigationEvidence) -> None:
        if not self._trace_task_id:
            return
        evidence = {
            "state": message.state,
            "distance_remaining": float(message.distance_remaining),
            "navigation_time_sec": float(message.navigation_time_sec),
            "estimated_time_remaining_sec": float(
                message.estimated_time_remaining_sec),
            "number_of_recoveries": int(message.number_of_recoveries),
        }
        self._latest_navigation_evidence[message.phase] = evidence
        self.trace.write(
            "execution_evidence",
            task_id=self._trace_task_id,
            phase=message.phase,
            **evidence,
        )

    def _trace_navigation_result(self, task_id: str, phase: str, status: str) -> None:
        evidence = self._latest_navigation_evidence.get(phase, {})
        self.trace.write(
            "navigation_result",
            task_id=task_id,
            phase=phase,
            status=status,
            last_distance_remaining=evidence.get("distance_remaining"),
            navigation_time_sec=evidence.get("navigation_time_sec"),
            number_of_recoveries=evidence.get("number_of_recoveries"),
        )

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
        if not response.found:
            raise SemanticEvidenceError(
                "refined" if refine else "coarse",
                response.detail or "semantic object was not found in map",
            )
        if response.center.header.frame_id != "map":
            raise SemanticEvidenceError(
                "query_contract", "semantic result is not expressed in map frame"
            )
        return response

    def _select_frontier_goal(
        self, task_id: str, attempt: int, visited: list[np.ndarray], goal_handle
    ) -> tuple[PoseStamped, dict]:
        with self._map_lock:
            map_snapshot = self.latest_map
        if map_snapshot is None:
            raise GeometryEvidenceError(
                "exploration", "live occupancy map is unavailable"
            )
        candidates = frontier_candidates(
            map_snapshot["grid"],
            map_snapshot["origin"],
            map_snapshot["resolution"],
            self.latest_pose[:2],
            self.frontier_min_cluster_cells,
            self.frontier_candidate_count,
            visited,
            self.frontier_revisit_radius,
        )
        results = []
        goals = []
        for index, candidate in enumerate(candidates):
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = float(candidate["xy"][0])
            goal.pose.position.y = float(candidate["xy"][1])
            goal.pose.orientation.z = float(math.sin(candidate["yaw"] / 2.0))
            goal.pose.orientation.w = float(math.cos(candidate["yaw"] / 2.0))
            goals.append(goal)
            result = self._probe_path(
                task_id, f"exploration_{attempt}", index, goal, goal_handle
            )
            results.append({**candidate, **result})
        viable = [
            index for index, result in enumerate(results)
            if result.get("available")
            and result.get("path_exists")
            and result.get("path_length_m", math.inf) <= self.exploration_max_path
        ]
        if not viable:
            raise GeometryEvidenceError(
                "exploration",
                "no unvisited frontier has a bounded known-space path",
            )
        selected = max(
            viable,
            key=lambda index: results[index]["gain_cells"]
            / (1.0 + results[index]["path_length_m"]),
        )
        evidence = {
            **results[selected],
            "attempt": attempt,
            "candidate_count": len(candidates),
            "plannable_candidate_count": len(viable),
            "max_path_m": self.exploration_max_path,
        }
        evidence["xy"] = evidence["xy"].tolist()
        self.trace.write("frontier_selection", task_id=task_id, **evidence)
        return goals[selected], evidence

    def _explore_for_target(
        self, task_id: str, query: str, goal_handle, initial_detail: str
    ):
        if self.exploration_max_steps == 0:
            raise SemanticEvidenceError("coarse", initial_detail)
        deadline = time.monotonic() + self.exploration_timeout
        visited = [self.latest_pose[:2].copy()]
        last_detail = initial_detail
        for attempt in range(1, self.exploration_max_steps + 1):
            if time.monotonic() >= deadline:
                break
            goal, evidence = self._select_frontier_goal(
                task_id, attempt, visited, goal_handle
            )
            visited.append(np.array([goal.pose.position.x, goal.pose.position.y]))
            self.trace.write(
                "intervention_started",
                task_id=task_id,
                intervention="frontier_exploration",
                **evidence,
            )
            self._publish_feedback(goal_handle, "EXPLORING", 0.0, goal)
            with self._status_condition:
                self._coarse_status = ""
            self._latest_navigation_evidence.pop("coarse", None)
            self.coarse_goal_publisher.publish(goal)
            status = self._wait_navigation(
                goal_handle,
                coarse=True,
                timeout=min(
                    self.navigation_timeout,
                    max(0.1, deadline - time.monotonic()),
                ),
            )
            self._trace_navigation_result(task_id, "coarse", status)
            self.trace.write(
                "intervention_finished",
                task_id=task_id,
                intervention="frontier_exploration",
                attempt=attempt,
                status=status,
            )
            if status == "nav_canceled" and goal_handle.is_cancel_requested:
                raise InterruptedError("semantic navigation canceled")
            if status != "nav_finish":
                last_detail = status
                continue
            wait_deadline = min(
                deadline, time.monotonic() + self.exploration_observation_wait
            )
            while time.monotonic() < wait_deadline:
                if goal_handle.is_cancel_requested:
                    self.cancel_publisher.publish(Empty())
                    raise InterruptedError("semantic navigation canceled")
                time.sleep(0.05)
            try:
                result = self._query(query, False, goal_handle)
                self.trace.write(
                    "exploration_target_found",
                    task_id=task_id,
                    attempt=attempt,
                    object_id=result.object_id,
                    score=float(result.score),
                )
                return result
            except SemanticEvidenceError as exc:
                if exc.phase != "coarse":
                    raise
                last_detail = str(exc)
        raise SemanticEvidenceError(
            "exploration",
            f"target remained unobserved after bounded exploration: {last_detail}",
        )

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

    def _goals_for_target(self, target, standoff: float) -> list[PoseStamped]:
        candidates = standoff_candidates(
            self.latest_pose[:2],
            np.array([target.point.x, target.point.y]),
            standoff,
            self.goal_candidate_count,
        )
        goals = []
        for goal_xy, yaw in candidates:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = float(goal_xy[0])
            goal.pose.position.y = float(goal_xy[1])
            goal.pose.orientation.z = float(np.sin(yaw / 2.0))
            goal.pose.orientation.w = float(np.cos(yaw / 2.0))
            goals.append(goal)
        return goals

    def _enforce_contract(self, task_id: str, stage: str, evidence: dict) -> None:
        if not self.contract_gate.enabled:
            return
        decision = self.contract_gate.evaluate(stage, evidence)
        self.trace.write("contract_decision", task_id=task_id, **decision)
        if not decision["passed"]:
            raise ContractRejectedError(decision)

    def _probe_path(
        self, task_id: str, phase: str, candidate_index: int, goal, goal_handle
    ) -> dict:
        try:
            if not self.path_client.wait_for_server(
                timeout_sec=self.planner_probe_timeout
            ):
                raise TimeoutError("compute_path_to_pose unavailable")
            request = ComputePathToPose.Goal()
            request.goal = goal
            request.start = self._observer_pose()
            request.use_start = True
            planner_goal = self._wait_future(
                self.path_client.send_goal_async(request),
                goal_handle,
                self.planner_probe_timeout,
            )
            if not planner_goal.accepted:
                evidence = {
                    "available": True,
                    "path_exists": False,
                    "detail": "planner rejected probe",
                    "geometry_no_path": 1.0,
                    "planner_unavailable": 0.0,
                }
                self.trace.write(
                    "geometry_evidence",
                    task_id=task_id,
                    phase=phase,
                    candidate_index=candidate_index,
                    **evidence,
                )
                return evidence
            wrapped = self._wait_future(
                planner_goal.get_result_async(),
                goal_handle,
                self.planner_probe_timeout,
            )
            poses = wrapped.result.path.poses
            points = [(pose.pose.position.x, pose.pose.position.y) for pose in poses]
            planning_time = wrapped.result.planning_time
            evidence = {
                "available": True,
                "path_exists": bool(poses),
                "path_length_m": polyline_length(points),
                "path_pose_count": len(poses),
                "planning_time_sec": (
                    float(planning_time.sec)
                    + float(planning_time.nanosec) / 1e9
                ),
                "action_status": int(wrapped.status),
                "geometry_no_path": 0.0 if poses else 1.0,
                "planner_unavailable": 0.0,
            }
            self.trace.write(
                "geometry_evidence",
                task_id=task_id,
                phase=phase,
                candidate_index=candidate_index,
                **evidence,
            )
            return evidence
        except InterruptedError:
            raise
        except Exception as exc:
            evidence = {
                "available": False,
                "detail": str(exc),
                "geometry_no_path": None,
                "planner_unavailable": 1.0,
            }
            self.trace.write(
                "geometry_evidence",
                task_id=task_id,
                phase=phase,
                candidate_index=candidate_index,
                **evidence,
            )
            return evidence

    def _select_goal(
        self, task_id: str, phase: str, target, standoff: float, goal_handle
    ) -> tuple[PoseStamped, dict]:
        goals = self._goals_for_target(target, standoff)
        results = []
        generator = "single_ray" if len(goals) == 1 else "ring"
        for index, goal in enumerate(goals):
            self.trace.write(
                "goal_candidate",
                task_id=task_id,
                phase=phase,
                candidate_index=index,
                x=goal.pose.position.x,
                y=goal.pose.position.y,
                standoff_m=float(standoff),
                generator=generator,
            )
            results.append(
                self._probe_path(task_id, phase, index, goal, goal_handle)
            )
        viable = [
            index
            for index, result in enumerate(results)
            if result.get("available") and result.get("path_exists")
        ]
        selected = min(
            viable,
            key=lambda index: results[index].get("path_length_m", math.inf),
            default=0,
        )
        evidence = {
            **results[selected],
            "candidate_count": len(goals),
            "plannable_candidate_count": len(viable),
            "selected_candidate_index": selected,
        }
        self.trace.write(
            "goal_selection",
            task_id=task_id,
            phase=phase,
            generator=generator,
            **evidence,
        )
        return goals[selected], evidence

    def _select_observation_goal(
        self, task_id: str, target, attempt: int, goal_handle
    ) -> tuple[PoseStamped, dict]:
        """Choose a plannable view that changes target bearing, not just distance."""
        phase = f"observation_{attempt}"
        goals = self._goals_for_target(target, self.observation_standoff)
        target_xy = np.array([target.point.x, target.point.y], dtype=np.float64)
        results = []
        for index, goal in enumerate(goals):
            separation = viewpoint_separation(
                self.latest_pose[:2],
                np.array([goal.pose.position.x, goal.pose.position.y]),
                target_xy,
            )
            self.trace.write(
                "observation_candidate",
                task_id=task_id,
                phase=phase,
                candidate_index=index,
                x=goal.pose.position.x,
                y=goal.pose.position.y,
                viewpoint_separation_deg=math.degrees(separation),
            )
            result = self._probe_path(task_id, phase, index, goal, goal_handle)
            results.append({**result, "viewpoint_separation_rad": separation})
        viable = [
            index for index, result in enumerate(results)
            if result.get("available") and result.get("path_exists")
        ]
        diverse = [
            index for index in viable
            if results[index]["viewpoint_separation_rad"] >= self.observation_min_angle
        ]
        selected = select_observation_candidate(
            results, self.observation_min_angle, self.observation_max_path
        )
        if selected is None:
            raise GeometryEvidenceError(
                "observation",
                f"no observation viewpoint within {self.observation_max_path:.2f} m",
            )
        budget_viable = [
            index for index in viable
            if results[index].get("path_length_m", math.inf)
            <= self.observation_max_path
        ]
        evidence = {
            **results[selected],
            "candidate_count": len(goals),
            "plannable_candidate_count": len(viable),
            "diverse_candidate_count": len(diverse),
            "budget_candidate_count": len(budget_viable),
            "max_path_m": self.observation_max_path,
            "selected_candidate_index": selected,
            "viewpoint_separation_deg": math.degrees(
                results[selected]["viewpoint_separation_rad"]
            ),
        }
        evidence.pop("viewpoint_separation_rad")
        self.trace.write(
            "observation_selection", task_id=task_id, phase=phase, **evidence
        )
        return goals[selected], evidence

    def _acquire_observation(
        self, task_id: str, target, attempt: int, score: float, goal_handle
    ) -> None:
        goal, evidence = self._select_observation_goal(
            task_id, target, attempt, goal_handle
        )
        self.trace.write(
            "intervention_started",
            task_id=task_id,
            intervention="acquire_observation",
            attempt=attempt,
            **evidence,
        )
        self._publish_feedback(goal_handle, "ACQUIRING_OBSERVATION", score, goal)
        with self._status_condition:
            self._coarse_status = ""
        self._latest_navigation_evidence.pop("coarse", None)
        self.coarse_goal_publisher.publish(goal)
        status = self._wait_navigation(goal_handle, coarse=True)
        self.trace.write(
            "intervention_finished",
            task_id=task_id,
            intervention="acquire_observation",
            attempt=attempt,
            status=status,
        )
        if status != "nav_finish":
            raise GeometryEvidenceError(
                "observation", f"observation navigation ended with {status}"
            )

    @staticmethod
    def _publish_feedback(goal_handle, phase: str, score: float, goal) -> None:
        feedback = NavigateToObject.Feedback()
        feedback.phase = phase
        feedback.score = score
        feedback.current_goal = goal
        goal_handle.publish_feedback(feedback)

    def _wait_navigation(
        self, goal_handle, coarse: bool, timeout: float | None = None
    ) -> str:
        attr = "_coarse_status" if coarse else "_fine_status"
        phase = "coarse" if coarse else "fine"
        deadline = time.monotonic() + (
            self.navigation_timeout if timeout is None else timeout
        )
        stall_monitor = (
            ProgressStallMonitor(
                self.execution_stall_timeout, self.execution_min_progress
            )
            if self.execution_stall_timeout > 0 else None
        )
        with self._status_condition:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.cancel_publisher.publish(Empty())
                    return "nav_canceled"
                status = getattr(self, attr)
                if status in TERMINAL_NAV_STATES:
                    return status
                evidence = self._latest_navigation_evidence.get(phase, {})
                recovery_count = int(evidence.get("number_of_recoveries", 0))
                if (
                    "number_of_recoveries" in evidence
                    and recovery_count > 0
                    and recovery_count >= self.max_execution_recoveries
                ):
                    self.cancel_publisher.publish(Empty())
                    return "recovery_limit"
                if (
                    stall_monitor is not None
                    and evidence.get("state") == "progress"
                    and stall_monitor.observe(
                        float(evidence.get("distance_remaining", math.inf)),
                        float(evidence.get("navigation_time_sec", math.nan)),
                    )
                ):
                    self.cancel_publisher.publish(Empty())
                    return "contract_stalled"
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
        task_id = bytes(goal_handle.goal_id.uuid).hex()
        self._trace_task_id = task_id
        object_query = goal_handle.request.query.split(",", maxsplit=2)[-1].strip()
        self.trace.write("task_started", task_id=task_id, query=object_query)
        try:
            status = self.latest_localization_status
            pose_fresh = self._fresh_pose()
            localization_evidence = (
                {
                    "available": False,
                    "pose_fresh": pose_fresh,
                    "pose_age_sec": None,
                    "localization_fitness": None,
                    "localization_failures": None,
                    "localization_not_tracking": 1.0,
                }
                if status is None
                else {
                    "available": True,
                    "pose_fresh": pose_fresh,
                    "localized": status["localized"],
                    "fitness_score": (
                        status["fitness_score"]
                        if math.isfinite(status["fitness_score"])
                        else None
                    ),
                    "consecutive_failures": status["consecutive_failures"],
                    "state": status["state"],
                    "detail": status["detail"],
                    "age_sec": time.monotonic() - status["received_monotonic"],
                    "pose_age_sec": (
                        time.monotonic() - self.latest_pose_received
                        if self.latest_pose is not None else None
                    ),
                    "localization_fitness": (
                        status["fitness_score"]
                        if math.isfinite(status["fitness_score"])
                        else None
                    ),
                    "localization_failures": status["consecutive_failures"],
                    "localization_not_tracking": (
                        0.0 if status["localized"] else 1.0
                    ),
                }
            )
            self.trace.write(
                "localization_evidence",
                task_id=task_id,
                **localization_evidence,
            )
            self._enforce_contract(
                task_id, "pre_dispatch", localization_evidence
            )
            if self.require_localization_tracking and (
                status is None or not status["localized"]
            ):
                state = "MISSING" if status is None else status["state"]
                raise LocalizationEvidenceError(
                    f"localization is not TRACKING: {state}"
                )
            if not pose_fresh:
                raise RuntimeError("localization/odom is missing or stale")
            request = goal_handle.request
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

            try:
                coarse = self._query(object_query, False, goal_handle)
            except SemanticEvidenceError as exc:
                if exc.phase != "coarse":
                    raise
                self.trace.write(
                    "semantic_evidence",
                    task_id=task_id,
                    phase="coarse",
                    available=False,
                    detail=str(exc),
                    recommended_intervention="frontier_exploration",
                )
                coarse = self._explore_for_target(
                    task_id, object_query, goal_handle, str(exc)
                )
            object_id = coarse.object_id
            coarse_margin = (
                float(coarse.score - coarse.second_score)
                if coarse.has_second_score else None
            )
            coarse_semantic_evidence = {
                "semantic_neg_score": -float(coarse.score),
                "semantic_neg_margin": (
                    -coarse_margin if coarse_margin is not None else None
                ),
                "semantic_neg_observations": -float(coarse.observation_count),
            }
            self.trace.write(
                "semantic_evidence",
                task_id=task_id,
                phase="coarse",
                object_id=object_id,
                score=float(coarse.score),
                second_score=(
                    float(coarse.second_score) if coarse.has_second_score else None
                ),
                score_margin=coarse_margin,
                observation_count=int(coarse.observation_count),
                source_stamp_ns=(
                    int(coarse.source_stamp.sec) * 1_000_000_000
                    + int(coarse.source_stamp.nanosec)
                ),
                detail=coarse.detail,
                center_x=float(coarse.center.point.x),
                center_y=float(coarse.center.point.y),
                passes_backend_threshold=bool(coarse.passes_backend_threshold),
                backend_min_score=float(coarse.backend_min_score),
                backend_min_margin=(
                    float(coarse.backend_min_margin)
                    if coarse.has_backend_min_margin else None
                ),
                **coarse_semantic_evidence,
            )
            self._enforce_contract(
                task_id, "coarse_semantic", coarse_semantic_evidence
            )
            if self.require_backend_threshold and not coarse.passes_backend_threshold:
                raise SemanticEvidenceError(
                    "coarse", "coarse evidence is below the backend threshold"
                )
            coarse_goal, coarse_geometry = self._select_goal(
                task_id, "coarse", coarse.center, coarse_standoff, goal_handle
            )
            self._enforce_contract(
                task_id, "coarse_geometry", coarse_geometry
            )
            if self.require_plannable_goal and not (
                coarse_geometry.get("available")
                and coarse_geometry.get("path_exists")
            ):
                raise GeometryEvidenceError(
                    "coarse", coarse_geometry.get("detail", "no coarse path")
                )
            self._publish_feedback(
                goal_handle, "COARSE_NAVIGATING", coarse.score, coarse_goal
            )
            with self._status_condition:
                self._coarse_status = ""
            self._latest_navigation_evidence.pop("coarse", None)
            self.coarse_goal_publisher.publish(coarse_goal)
            status = self._wait_navigation(goal_handle, coarse=True)
            self._trace_navigation_result(task_id, "coarse", status)
            if status != "nav_finish":
                self.trace.write(
                    "task_finished", task_id=task_id, success=False,
                    failure_type="execution", phase="coarse",
                    recommended_intervention="recover_or_abort", detail=status
                )
                if status == "nav_canceled" and goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(False, object_id, coarse_goal, status)

            observation_attempt = 0
            while True:
                self._publish_feedback(
                    goal_handle, "REFINING", coarse.score, coarse_goal
                )
                refined = self._query(object_query, True, goal_handle)
                refined_margin = (
                    float(refined.score - refined.second_score)
                    if refined.has_second_score else None
                )
                refined_semantic_evidence = {
                    "semantic_neg_score": -float(refined.score),
                    "semantic_neg_margin": (
                        -refined_margin if refined_margin is not None else None
                    ),
                    "semantic_neg_observations": -float(refined.observation_count),
                    "semantic_center_shift_m": math.hypot(
                        refined.center.point.x - coarse.center.point.x,
                        refined.center.point.y - coarse.center.point.y,
                    ),
                }
                self.trace.write(
                    "semantic_evidence",
                    task_id=task_id,
                    phase="refined",
                    observation_attempt=observation_attempt,
                    object_id=refined.object_id,
                    score=float(refined.score),
                    second_score=(
                        float(refined.second_score)
                        if refined.has_second_score else None
                    ),
                    score_margin=refined_margin,
                    observation_count=int(refined.observation_count),
                    source_stamp_ns=(
                        int(refined.source_stamp.sec) * 1_000_000_000
                        + int(refined.source_stamp.nanosec)
                    ),
                    detail=refined.detail,
                    center_x=float(refined.center.point.x),
                    center_y=float(refined.center.point.y),
                    passes_backend_threshold=bool(refined.passes_backend_threshold),
                    backend_min_score=float(refined.backend_min_score),
                    backend_min_margin=(
                        float(refined.backend_min_margin)
                        if refined.has_backend_min_margin else None
                    ),
                    **refined_semantic_evidence,
                )
                try:
                    self._enforce_contract(
                        task_id, "refined_semantic", refined_semantic_evidence
                    )
                except ContractRejectedError as exc:
                    can_observe = (
                        observation_attempt < self.max_observation_retries
                        and "acquire_observation" in exc.decision["interventions"]
                    )
                    if not can_observe:
                        raise
                    observation_attempt += 1
                    self._acquire_observation(
                        task_id,
                        coarse.center,
                        observation_attempt,
                        float(refined.score),
                        goal_handle,
                    )
                    continue
                if self.require_backend_threshold and not refined.passes_backend_threshold:
                    if observation_attempt < self.max_observation_retries:
                        observation_attempt += 1
                        self._acquire_observation(
                            task_id,
                            coarse.center,
                            observation_attempt,
                            float(refined.score),
                            goal_handle,
                        )
                        continue
                    raise SemanticEvidenceError(
                        "refined", "refined evidence is below the backend threshold"
                    )
                break
            self._update_anchor(object_id, refined, goal_handle)
            if not self._fresh_pose():
                raise RuntimeError("localization/odom became stale before fine navigation")

            final_goal, fine_geometry = self._select_goal(
                task_id, "fine", refined.center, fine_standoff, goal_handle
            )
            self._enforce_contract(task_id, "fine_geometry", fine_geometry)
            if self.require_plannable_goal and not (
                fine_geometry.get("available") and fine_geometry.get("path_exists")
            ):
                raise GeometryEvidenceError(
                    "fine", fine_geometry.get("detail", "no fine path")
                )
            self._publish_feedback(
                goal_handle, "FINE_NAVIGATING", refined.score, final_goal
            )
            with self._status_condition:
                self._fine_status = ""
            self._latest_navigation_evidence.pop("fine", None)
            self.fine_goal_publisher.publish(final_goal)
            status = self._wait_navigation(goal_handle, coarse=False)
            self._trace_navigation_result(task_id, "fine", status)
            if status == "nav_finish":
                last_execution = self._latest_navigation_evidence.get("fine", {})
                completion_evidence = {
                    "execution_recoveries": last_execution.get(
                        "number_of_recoveries"
                    ),
                    "final_distance_remaining": last_execution.get(
                        "distance_remaining"
                    ),
                }
                try:
                    confirmation = self._query(object_query, True, goal_handle)
                    confirmation_shift = math.hypot(
                        confirmation.center.point.x - refined.center.point.x,
                        confirmation.center.point.y - refined.center.point.y,
                    )
                    final_target_distance = math.hypot(
                        confirmation.center.point.x - self.latest_pose[0],
                        confirmation.center.point.y - self.latest_pose[1],
                    )
                    final_goal_error = goal_error(
                        self.latest_pose,
                        [final_goal.pose.position.x, final_goal.pose.position.y],
                    )
                    completion_evidence.update({
                        "confirmation_unavailable": 0.0,
                        "confirmation_neg_score": -float(confirmation.score),
                        "confirmation_neg_observations": -float(
                            confirmation.observation_count
                        ),
                        "confirmation_center_shift_m": confirmation_shift,
                        "confirmation_backend_rejected": (
                            0.0 if confirmation.passes_backend_threshold else 1.0
                        ),
                        "final_target_distance_m": final_target_distance,
                        "final_goal_error_m": final_goal_error,
                        "standoff_error_m": abs(
                            final_target_distance - fine_standoff
                        ),
                    })
                    self.trace.write(
                        "semantic_evidence",
                        task_id=task_id,
                        phase="confirmation",
                        object_id=confirmation.object_id,
                        score=float(confirmation.score),
                        observation_count=int(confirmation.observation_count),
                        center_x=float(confirmation.center.point.x),
                        center_y=float(confirmation.center.point.y),
                        center_shift_m=confirmation_shift,
                        final_target_distance_m=final_target_distance,
                        final_goal_error_m=final_goal_error,
                        standoff_error_m=abs(final_target_distance - fine_standoff),
                        robot_x=float(self.latest_pose[0]),
                        robot_y=float(self.latest_pose[1]),
                        passes_backend_threshold=bool(
                            confirmation.passes_backend_threshold
                        ),
                        detail=confirmation.detail,
                    )
                    if (
                        self.require_completion_confirmation
                        and self.require_backend_threshold
                        and not confirmation.passes_backend_threshold
                    ):
                        raise SemanticEvidenceError(
                            "confirmation",
                            "completion evidence is below the backend threshold",
                        )
                except Exception as exc:
                    completion_evidence["confirmation_unavailable"] = 1.0
                    self.trace.write(
                        "semantic_evidence",
                        task_id=task_id,
                        phase="confirmation",
                        available=False,
                        detail=str(exc),
                    )
                    if self.require_completion_confirmation:
                        raise SemanticEvidenceError("confirmation", str(exc))
                if (
                    self.require_completion_confirmation
                    and completion_evidence.get("final_goal_error_m", math.inf)
                    > self.completion_goal_tolerance
                ):
                    raise ExecutionEvidenceError(
                        "Nav2 reported success but localization-to-goal error "
                        f"is {completion_evidence['final_goal_error_m']:.3f} m "
                        f"> {self.completion_goal_tolerance:.3f} m"
                    )
                self._enforce_contract(
                    task_id,
                    "completion",
                    completion_evidence,
                )
                goal_handle.succeed()
                self.trace.write("task_finished", task_id=task_id, success=True)
                return self._result(True, object_id, final_goal, "completed")
            if status == "nav_canceled" and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            self.trace.write(
                "task_finished", task_id=task_id, success=False,
                failure_type="execution", phase="fine",
                recommended_intervention="recover_or_abort", detail=status
            )
            return self._result(False, object_id, final_goal, status)
        except InterruptedError as exc:
            self.trace.write(
                "task_finished", task_id=task_id, success=False,
                failure_type="canceled", detail=str(exc)
            )
            goal_handle.canceled()
            return self._result(False, object_id, final_goal, str(exc))
        except SemanticEvidenceError as exc:
            intervention = (
                "acquire_observation"
                if exc.phase in {"refined", "confirmation"}
                else "clarify_or_observe"
            )
            self.trace.write(
                "task_finished",
                task_id=task_id,
                success=False,
                failure_type="semantic",
                phase=exc.phase,
                recommended_intervention=intervention,
                detail=str(exc),
            )
            self.get_logger().error(f"Semantic evidence unavailable: {exc}")
            goal_handle.abort()
            return self._result(False, object_id, final_goal, str(exc))
        except LocalizationEvidenceError as exc:
            self.trace.write(
                "task_finished",
                task_id=task_id,
                success=False,
                failure_type="localization",
                phase="pre_dispatch",
                recommended_intervention="relocalize",
                detail=str(exc),
            )
            self.get_logger().warning(str(exc))
            goal_handle.abort()
            return self._result(False, object_id, final_goal, str(exc))
        except GeometryEvidenceError as exc:
            self.trace.write(
                "task_finished",
                task_id=task_id,
                success=False,
                failure_type="geometry",
                phase=exc.phase,
                recommended_intervention="select_alternate_goal",
                detail=str(exc),
            )
            self.get_logger().warning(f"Goal is not plannable: {exc}")
            goal_handle.abort()
            return self._result(False, object_id, final_goal, str(exc))
        except ExecutionEvidenceError as exc:
            self.trace.write(
                "task_finished",
                task_id=task_id,
                success=False,
                failure_type="execution",
                phase="completion",
                recommended_intervention="verify_pose_or_replan",
                detail=str(exc),
            )
            self.get_logger().warning(str(exc))
            goal_handle.abort()
            return self._result(False, object_id, final_goal, str(exc))
        except ContractRejectedError as exc:
            decision = exc.decision
            self.trace.write(
                "task_finished",
                task_id=task_id,
                success=False,
                failure_type="contract_rejected",
                phase=decision["stage"],
                recommended_intervention=(
                    decision["interventions"][0]
                    if len(decision["interventions"]) == 1 else "abstain"
                ),
                detail=str(exc),
            )
            self.get_logger().warning(str(exc))
            goal_handle.abort()
            return self._result(False, object_id, final_goal, str(exc))
        except Exception as exc:
            self.trace.write(
                "task_finished", task_id=task_id, success=False,
                failure_type="runtime_error", detail=str(exc)
            )
            self.get_logger().error(f"Semantic navigation failed: {exc}")
            goal_handle.abort()
            return self._result(False, object_id, final_goal, str(exc))
        finally:
            self._trace_task_id = ""
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
