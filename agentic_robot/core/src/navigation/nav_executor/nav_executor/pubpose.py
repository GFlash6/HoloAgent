#!/usr/bin/env python3
"""
nav_executor — 导航中枢调度节点.

从 robot × map 双层 yaml 加载 signal 注册表，实现 waypoint / arm_skill /
nav_command 三类分发；同时保留旧硬编码分支作为 fallback，双轨并行验证。

ROS parameters:
  robot_name       (string, default "unitree")  — robots/<robot_name>/ 目录
  map_name         (string, default "")          — config/maps/<map_name>/ 子目录；空则仅加载 common
  signals_base_dir (string, default "")          — signals yaml 搜索根目录；
                                                   空则自动推断为本 package share 目录的
                                                   ../../../../robots/（适合 install 树）

Topics subscribed:
  chat_signal_pub  (std_msgs/String) — 命名 signal 命令
  object_pose      (geometry_msgs/PoseStamped) — 来自 semantic_goal / relative_goal
  semantic_approach_pose (geometry_msgs/PoseStamped) — 语义粗导航内部目标

Topics published:
  waypoint_reached (std_msgs/String) — 到点通知（包含 waypoint name 或 "struck"）
  semantic_approach_status (std_msgs/String) — 语义粗导航内部状态
  arm_signal_pub   (std_msgs/String) — arm skill 命令（替换原 waypoint_reached 回环）
"""

import os
import pathlib

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Empty, String
import math
import yaml


# ---------------------------------------------------------------------------
# signal registry helpers
# ---------------------------------------------------------------------------

def _load_yaml_safe(path: str) -> dict:
    """加载 yaml 文件，文件不存在时返回空 dict。"""
    p = pathlib.Path(path)
    if not p.exists():
        return {}
    with open(p, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_signal_registry(
        base_dir: str,
        robot_name: str,
        map_name: str) -> dict:
    """
    合并 config/signals_common.yaml 与 config/maps/<map>/signals.yaml，返回 {name:
    spec}。

    加载顺序：common → map（map key 覆盖 common 同名 key）
    """
    common_path = os.path.join(
        base_dir,
        robot_name,
        "config",
        "signals_common.yaml")
    common_data = _load_yaml_safe(common_path)
    registry = dict(common_data.get("signals", {}))

    if map_name:
        map_path = os.path.join(
            base_dir,
            robot_name,
            "config",
            "maps",
            map_name,
            "signals.yaml")
        map_data = _load_yaml_safe(map_path)
        registry.update(map_data.get("signals", {}))

    return registry


# ---------------------------------------------------------------------------
# main node
# ---------------------------------------------------------------------------

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        print('waiting for start...')

        # ---------- ROS parameters ----------
        self.declare_parameter('robot_name', 'unitree')
        self.declare_parameter('map_name', '')
        self.declare_parameter('signals_base_dir', '')

        robot_name = self.get_parameter(
            'robot_name').get_parameter_value().string_value
        map_name = self.get_parameter(
            'map_name').get_parameter_value().string_value
        signals_base_dir = self.get_parameter(
            'signals_base_dir').get_parameter_value().string_value

        if not signals_base_dir:
            # 自动推断：本文件所在 install/lib/nav_executor/nav_executor/ 往上 5 层到 workspace
            # 开发时 src 树：…/core/src/navigation/nav_executor/nav_executor/pubpose.py
            # 均尝试 workspace 根下的 robots/ 目录
            _this_file = pathlib.Path(__file__).resolve()
            for candidate in _this_file.parents:
                robots_dir = candidate / "robots"
                if robots_dir.is_dir():
                    signals_base_dir = str(robots_dir)
                    break
            if not signals_base_dir:
                signals_base_dir = str(_this_file.parent)

        self.get_logger().info(
            f"signals_base_dir={signals_base_dir}  robot={robot_name}  map={map_name}")

        # ---------- signal registry ----------
        self.signal_registry = load_signal_registry(
            signals_base_dir, robot_name, map_name)
        if self.signal_registry:
            self.get_logger().info(
                f"Loaded signal registry: {list(self.signal_registry.keys())}"
            )
        else:
            self.get_logger().warn(
                "Signal registry is EMPTY — falling back to hardcoded waypoints"
            )

        # ---------- subscriptions ----------
        self.signal_navigation_sub = self.create_subscription(
            String,
            'chat_signal_pub',
            self.chatsignal_navigation_callback,
            10
        )
        self.semantic_navigation_sub = self.create_subscription(
            PoseStamped,
            'navigation/goal_pose',
            self.semantic_callback,
            10
        )
        self.semantic_approach_sub = self.create_subscription(
            PoseStamped,
            'navigation/semantic/coarse_goal',
            self.semantic_approach_callback,
            10
        )
        self.cancel_sub = self.create_subscription(
            Empty, 'navigation/cancel', self.cancel_navigation_callback, 10)

        # ---------- publishers ----------
        self.waypoint_reached_pub = self.create_publisher(
            String, 'waypoint_reached', 10)
        self.goal_status_pub = self.create_publisher(
            String, 'navigation/goal_status', 10)
        self.semantic_approach_status_pub = self.create_publisher(
            String, 'navigation/semantic/coarse_status', 10)
        self.arm_signal_pub = self.create_publisher(
            String, 'manipulation/command', 10)

        # ---------- nav2 ----------
        self.navigator = BasicNavigator()

        self.waypoints = []
        self.current_waypoint_index = 0
        self.total_waypoints = 0
        self.navigation_active = False
        self.recoveries = 0
        self.single_pose_status_pub = self.goal_status_pub

        # Expose registry as ROS parameters (~/signals/<name>/*)
        self._declare_registry_params()

    # ------------------------------------------------------------------
    # ROS parameter exposure
    # ------------------------------------------------------------------

    def _declare_registry_params(self):
        """把 signal_registry 写入 ~/signals/* ROS parameters，供 chatbot 拉取。"""
        for name, spec in self.signal_registry.items():
            prefix = f"signals.{name}"
            action = spec.get("action", {})
            try:
                self.declare_parameter(f"{prefix}.action_type",
                                       action.get("type", ""))
                self.declare_parameter(f"{prefix}.description",
                                       str(spec.get("description", "")))
                chat_id = spec.get("chat_id", None)
                if chat_id is not None:
                    self.declare_parameter(f"{prefix}.chat_id", int(chat_id))
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to declare param for {name}: {e}")

    # ------------------------------------------------------------------
    # semantic (object_pose) callback
    # ------------------------------------------------------------------

    def semantic_callback(self, msg):
        self.get_logger().info("Navigating to target pose from object_pose...")
        self._start_single_pose(msg, self.goal_status_pub)

    def semantic_approach_callback(self, msg):
        self.get_logger().info("Navigating to internal semantic approach pose...")
        self._start_single_pose(msg, self.semantic_approach_status_pub)

    def cancel_navigation_callback(self, _msg):
        """Cancel the active Nav2 task through the canonical ROS control topic."""
        if not self.navigation_active:
            return
        self.navigator.cancelTask()

    def _start_single_pose(self, msg, status_publisher):
        if self.navigation_active:
            self.get_logger().warning("Navigation already active, ignoring pose goal")
            rejected = String()
            rejected.data = "nav_failed"
            status_publisher.publish(rejected)
            return
        self.navigation_active = True
        self.single_pose_status_pub = status_publisher
        self.recoveries = 0
        self.navigator.goToPose(msg)
        self.navigation_timer = self.create_timer(
            0.5, self.check_navigation_status2)

    def _publish_single_pose_status(self, status):
        msg = String()
        msg.data = status
        self.single_pose_status_pub.publish(msg)
        if self.single_pose_status_pub == self.goal_status_pub:
            self.waypoint_reached_pub.publish(msg)
        self.get_logger().info(f"Published single-pose status: {status}")

    # ------------------------------------------------------------------
    # named signal callback — registry-first, fallback to hardcoded
    # ------------------------------------------------------------------

    def chatsignal_navigation_callback(self, msg):
        signal_name = msg.data
        print(signal_name)

        # ---- registry dispatch ----
        if signal_name in self.signal_registry:
            self._dispatch_signal(
                signal_name, self.signal_registry[signal_name])
            return

        # ---- fallback: hardcoded legacy behaviour ----
        self.get_logger().warn(
            f"Signal '{signal_name}' not in registry — trying hardcoded fallback"
        )
        self._legacy_chatsignal_callback(signal_name)

    def _dispatch_signal(self, name: str, spec: dict):
        """根据 spec.action.type 分发到对应执行器。"""
        action = spec.get("action", {})
        action_type = action.get("type", "")

        if action_type == "nav_command":
            command = action.get("command", "")
            if command == "cancel":
                print(f"nav_command: cancel — 停止当前导航任务")
                self.navigator.cancelTask()
                self.navigation_active = False
                self.publish_waypoint_reached("nav_canceled")
            else:
                self.get_logger().warn(f"Unknown nav_command: {command}")

        elif action_type == "arm_skill":
            skill = action.get("skill", name)
            print(f"arm_skill: {skill}")
            msg = String()
            msg.data = skill
            self.arm_signal_pub.publish(msg)

        elif action_type == "waypoint":
            if self.navigation_active:
                self.get_logger().info(
                    f"Navigation already active, ignoring duplicate signal '{name}'"
                )
                return
            points = action.get("points", [])
            if not points:
                self.get_logger().warn(
                    f"Signal '{name}' has no waypoints defined")
                return
            self.waypoints = [
                [p["x"], p["y"], p["yaw"], p.get("name", f"{name}_wp{i}")]
                for i, p in enumerate(points)
            ]
            self.current_waypoint_index = 0
            self.total_waypoints = len(self.waypoints)
            self.get_logger().info(
                f"Starting waypoint navigation for '{name}' "
                f"({self.total_waypoints} point(s))"
            )
            self.navigate_to_next_waypoint()
            if self.current_waypoint_index == 0:
                self.navigation_timer = self.create_timer(
                    0.5, self.check_navigation_status
                )
        else:
            self.get_logger().warn(
                f"Unknown action type '{action_type}' for signal '{name}'"
            )

    # ------------------------------------------------------------------
    # LEGACY fallback — original hardcoded if-else (kept for dual-rail)
    # ------------------------------------------------------------------

    def _legacy_chatsignal_callback(self, signal_name: str):
        """原始硬编码分支，在 registry 找不到时作为 fallback。"""
        msg_data = signal_name

        if msg_data == 'stop':
            print('stop...停止当前导航任务...')
            self.navigator.cancelTask()
            self.navigation_active = False
            self.publish_waypoint_reached("nav_canceled")
            return

        # if msg_data in ('grab_hello_action', 'wave_hands'):
        #     print('grab_hello_action or wave_hands to say hello...')
        #     self.publish_waypoint_reached("grab_hello_action")
        #     return

        # if msg_data == 'wave_under_head':
        #     print('wave_under_head to say goodbye...')
        #     self.publish_waypoint_reached("wave_under_head")
        #     return

        if self.navigation_active:
            self.get_logger().info("Navigation already active, ignoring duplicate.")
            return

        if msg_data.startswith('custom_one_point_1_'):
            try:
                # 去除前缀，剩下x_y_yaw
                prefix = 'custom_one_point_1_'
                payload = msg_data[len(prefix):]
                # 支持负数和小数，允许下划线分隔
                vals = payload.split('_')
                if len(vals) != 3:
                    print(
                        f"custom_one_point_1_ format error: expected 3 values (x, y, yaw), got {len(vals)}: {vals}")
                    return
                x, y, yaw = map(float, vals)
                print(
                    f'start one_point_1 with custom pos: x={x}, y={y}, yaw={yaw}')
                self.waypoints = [[x, y, yaw, "one_point_1_waypoint_1"]]
                self.current_waypoint_index = 0
                self.total_waypoints = 1
                self.get_logger().info(
                    f"Received navigation command: {msg_data}")
                self.navigate_to_next_waypoint()
                if self.current_waypoint_index == 0:
                    self.navigation_timer = self.create_timer(
                        0.5, self.check_navigation_status)
                return
            except Exception as e:
                print(
                    f'Error parsing custom_one_point_1_: {e}. 输入格式应为 custom_one_point_1_x_y_yaw，如 custom_one_point_1_1.23_-4.56_0.78')
                return

        # # hardcoded waypoint table (legacy, maintained for reference only)
        # _hardcoded = {
        #     'one_point_1': [
        #         [19.50, -1.05, -1.60, "one_point_1_waypoint_1"],
        #     ],
        #     'one_point_2': [
        #         [4.3874, -2.5845, 1.31854, "one_point_2_waypoint_1"],
        #     ],
        #     'one_point_3': [
        #         [22.38, 1.70, 3.14, "one_point_3_waypoint_1"],
        #     ],
        #     'one_point_4': [
        #         [22.38, 1.70, 0.0, "one_point_4_waypoint_1"],
        #     ],
        #     'multi_point_1': [
        #         [22.38, 1.70,  3.14, "multi_point_1_waypoint_1"],
        #         [22.38, 1.70, -1.70, "multi_point_1_waypoint_2"],
        #     ],
        #     'multi_point_2': [
        #         [22.38, 1.70,  0.0, "multi_point_2_waypoint_1"],
        #         [39.8,  -1.38, 0.0, "multi_point_2_waypoint_2"],
        #     ],
        # }

        # if msg_data not in _hardcoded:
        #     self.get_logger().warn(f"Unknown signal in fallback: {msg_data}")
        #     return

        # print(f'start {msg_data} (hardcoded fallback)...')
        # self.waypoints = _hardcoded[msg_data]
        # self.current_waypoint_index = 0
        # self.total_waypoints = len(self.waypoints)

        # if self.current_waypoint_index >= self.total_waypoints:
        #     self.get_logger().info("All waypoints completed!")
        #     return

        # self.get_logger().info(f"Received navigation command: {msg_data}")
        # self.navigate_to_next_waypoint()
        # if self.current_waypoint_index == 0:
        #     self.navigation_timer = self.create_timer(0.5, self.check_navigation_status)

    # ------------------------------------------------------------------
    # pose helpers
    # ------------------------------------------------------------------

    def create_pose_stamped(self, position_x, position_y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal_pose

    def publish_waypoint_reached(self, waypoint_name):
        msg = String()
        msg.data = waypoint_name
        self.waypoint_reached_pub.publish(msg)
        self.get_logger().info(f"Published waypoint_reached: {waypoint_name}")

    def navigate_to_next_waypoint(self):
        self.navigation_active = True
        wp = self.waypoints[self.current_waypoint_index]
        goal_pose = self.create_pose_stamped(wp[0], wp[1], wp[2])
        self.get_logger().info(
            f"Navigating to {wp[3]} ({wp[0]:.2f}, {wp[1]:.2f}, yaw={wp[2]:.2f})...")
        self.navigator.goToPose(goal_pose)

    # ------------------------------------------------------------------
    # navigation status polling
    # ------------------------------------------------------------------

    def check_navigation_status(self):
        """多航点顺序执行的状态轮询。"""
        if not self.navigation_active:
            return
        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            wp_name = self.waypoints[self.current_waypoint_index][3]
            self.get_logger().info(f"Reached waypoint: {wp_name}")
            self.publish_waypoint_reached(wp_name)
            self.current_waypoint_index += 1
            self.navigation_active = False

            if self.current_waypoint_index >= self.total_waypoints:
                self.get_logger().info("All waypoints completed! Destroying timer.")
                self.destroy_timer(self.navigation_timer)
            else:
                self.navigate_to_next_waypoint()
        else:
            self.get_logger().error(f"Navigation failed with result: {result}")
            status = ("nav_canceled" if result == TaskResult.CANCELED
                      else "nav_failed")
            self.publish_waypoint_reached(status)
            self.navigation_active = False
            self.destroy_timer(self.navigation_timer)

    def check_navigation_status2(self):
        """单 Pose 目标（来自 object_pose）的状态轮询，支持 stuck 检测。"""
        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and feedback.number_of_recoveries > self.recoveries + 2:
                self.get_logger().info("Navigation stuck!")
                self.recoveries = feedback.number_of_recoveries
                self._publish_single_pose_status("struck")
            return

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            wp_name = "nav_finish"
            self._publish_single_pose_status(wp_name)
            self.get_logger().info("Navigation to pose completed.")
        else:
            self.get_logger().error(f"Navigation to pose failed: {result}")
            status = ("nav_canceled" if result == TaskResult.CANCELED
                      else "nav_failed")
            self._publish_single_pose_status(status)
        self.navigation_active = False
        self.destroy_timer(self.navigation_timer)


def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()
    try:
        rclpy.spin(waypoint_navigator)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                waypoint_navigator.navigator.lifecycleShutdown()
            waypoint_navigator.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
