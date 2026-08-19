"""Turn the robot-specific arm topic protocol into a task-scoped ROS action."""

from __future__ import annotations

import threading
import time

import rclpy
from holoagent_interfaces.action import ExecuteArmSkill
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from .protocol import parse_arm_result


class ArmSkillServer(Node):
    def __init__(self) -> None:
        super().__init__("arm_skill_server")
        self.declare_parameter("timeout_sec", 20.0)
        self.timeout = float(self.get_parameter("timeout_sec").value)
        if self.timeout <= 0.0:
            raise ValueError("timeout_sec must be positive")

        callbacks = ReentrantCallbackGroup()
        self.command_publisher = self.create_publisher(
            String, "manipulation/command", 10
        )
        self.create_subscription(
            String,
            "manipulation/result",
            self._result_callback,
            10,
            callback_group=callbacks,
        )
        self._condition = threading.Condition()
        self._last_result = None
        self._active_lock = threading.Lock()
        self._active = False
        self.action_server = ActionServer(
            self,
            ExecuteArmSkill,
            "manipulation/execute_skill",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callbacks,
        )
        self.get_logger().info("Manipulation action is ready")

    def _goal_callback(self, request) -> GoalResponse:
        if not request.skill.strip():
            return GoalResponse.REJECT
        if self.command_publisher.get_subscription_count() == 0:
            self.get_logger().error(
                "Rejecting arm skill because no manipulation device adapter is connected"
            )
            return GoalResponse.REJECT
        with self._active_lock:
            if self._active:
                return GoalResponse.REJECT
            self._active = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _result_callback(self, message: String) -> None:
        result = parse_arm_result(message.data)
        if result is None:
            return
        with self._condition:
            self._last_result = result
            self._condition.notify_all()

    def _publish_command(self, value: str) -> None:
        command = String()
        command.data = value
        self.command_publisher.publish(command)

    def _action_result(self, success: bool, detail: str):
        result = ExecuteArmSkill.Result()
        result.success = success
        result.detail = detail
        return result

    def _execute(self, goal_handle):
        skill = goal_handle.request.skill.strip()
        try:
            with self._condition:
                self._last_result = None
            feedback = ExecuteArmSkill.Feedback()
            feedback.phase = "EXECUTING"
            goal_handle.publish_feedback(feedback)
            self._publish_command(skill)
            deadline = time.monotonic() + self.timeout

            with self._condition:
                while rclpy.ok():
                    if goal_handle.is_cancel_requested:
                        self._publish_command("release_arm")
                        goal_handle.canceled()
                        return self._action_result(False, "canceled; release requested")
                    if self._last_result is not None:
                        success, result_skill, detail = self._last_result
                        if result_skill == skill:
                            if success:
                                goal_handle.succeed()
                                return self._action_result(True, "completed")
                            goal_handle.abort()
                            return self._action_result(False, detail or "hardware failure")
                    if time.monotonic() >= deadline:
                        self._publish_command("release_arm")
                        goal_handle.abort()
                        return self._action_result(False, "hardware result timed out")
                    self._condition.wait(timeout=0.1)

            goal_handle.abort()
            return self._action_result(False, "ROS shutdown")
        finally:
            with self._active_lock:
                self._active = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmSkillServer()
    executor = MultiThreadedExecutor(num_threads=2)
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
