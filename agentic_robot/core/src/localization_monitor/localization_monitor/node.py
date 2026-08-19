"""Combine legacy registration signals into one stamped status message."""

from __future__ import annotations

import time

import rclpy
from holoagent_interfaces.msg import LocalizationStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

from .health import localization_state


class LocalizationMonitor(Node):
    def __init__(self) -> None:
        super().__init__("localization_monitor")
        self.declare_parameter("failure_limit", 3)
        self.declare_parameter("max_quality_age_sec", 1.0)
        self.failure_limit = int(self.get_parameter("failure_limit").value)
        self.max_quality_age = float(self.get_parameter("max_quality_age_sec").value)
        if self.failure_limit <= 0 or self.max_quality_age <= 0.0:
            raise ValueError("localization monitor parameters must be positive")

        self.registration_success = None
        self.score = float("nan")
        self.last_quality_received = 0.0
        self.consecutive_failures = 0
        self.publisher = self.create_publisher(
            LocalizationStatus, "localization/status", 10
        )
        self.create_subscription(
            Bool,
            "localization/registration_success",
            self._success_callback,
            10,
        )
        self.create_subscription(
            Float64, "localization/fitness_score", self._score_callback, 10
        )
        self.create_subscription(Odometry, "localization/odom", self._odom_callback, 10)

    def _success_callback(self, message: Bool) -> None:
        self.registration_success = bool(message.data)
        self.consecutive_failures = (
            0 if message.data else self.consecutive_failures + 1
        )
        self.last_quality_received = time.monotonic()

    def _score_callback(self, message: Float64) -> None:
        self.score = float(message.data)
        self.last_quality_received = time.monotonic()

    def _odom_callback(self, message: Odometry) -> None:
        stale = (
            self.last_quality_received == 0.0
            or time.monotonic() - self.last_quality_received > self.max_quality_age
        )
        status = LocalizationStatus()
        status.header = message.header
        status.fitness_score = self.score
        status.consecutive_failures = self.consecutive_failures
        if stale:
            status.localized = False
            status.state = "STALE"
            status.detail = "registration quality is stale"
        else:
            status.state = localization_state(
                self.registration_success,
                self.score,
                self.consecutive_failures,
                self.failure_limit,
            )
            status.localized = status.state == "TRACKING"
            status.detail = "registration update received"
        self.publisher.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
