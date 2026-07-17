#!/usr/bin/env python3
"""
按顺序循环导航至多个目标点，统计每次到达后的位置/朝向误差。

用法：     python3 send_nav_goal.py [loops]

loops  循环整个航点列表的次数，不填时使用脚本内 DEFAULT_LOOPS。

示例：     python3 send_nav_goal.py        # 默认循环次数     python3 send_nav_goal.py 5
# 循环 5 次
"""

import sys
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# ── 循环次数 ─────────────────────────────────────────────────────────────────
DEFAULT_LOOPS = 1

# ── 航点列表 (x, y, yaw_deg) ─────────────────────────────────────────────────
WAYPOINTS = [
    # (-3.61,  3.29,  -98.3),   # 2F 茶水间
    # (-3.30,  8.40, 0.0),      # 2F 微波炉
    # (-2.63, 3.96, 0.0),       # 2F 植物
    # (-0.15,  1.00,  0.0),     # 2F 原点
    # ( 0.13,  2.80, -168.0),   # 2F 沙发
    (3.959, -9.696, -83.565),  # 3F 操作台
]

# ── 实际位姿 topic ────────────────────────────────────────────────────────────
POSE_TOPIC = "/pose"            # nav_msgs/msg/Odometry
# ─────────────────────────────────────────────────────────────────────────────


def yaw_to_quaternion(yaw_rad: float):
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return 0.0, 0.0, qz, qw


def quaternion_to_yaw(qx, qy, qz, qw) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d <= -math.pi:
        d += 2 * math.pi
    return d


class NavLooper(Node):
    def __init__(self, loops: int):
        super().__init__("nav_goal_looper")
        self._loops = loops
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._actual_pose = None

        # 记录每次到达的误差：(loop_idx, wp_idx, dx, dy, dist_err, yaw_err_deg)
        self._records: list[tuple] = []

        # 当前执行进度
        self._loop_idx = 0
        self._wp_idx = 0

        self.create_subscription(Odometry, POSE_TOPIC, self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry):
        self._actual_pose = msg.pose.pose

    # ── 启动 ──────────────────────────────────────────────────────────────────
    def start(self):
        self.get_logger().info("等待 navigate_to_pose action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server 未响应，请确认 Nav2 已启动。")
            rclpy.shutdown()
            return
        self._send_next()

    # ── 发送当前航点 ──────────────────────────────────────────────────────────
    def _send_next(self):
        if self._loop_idx >= self._loops:
            self._print_summary()
            rclpy.shutdown()
            return

        gx, gy, gyaw_deg = WAYPOINTS[self._wp_idx]
        yaw_rad = math.radians(gyaw_deg)
        qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(
            f"[循环 {self._loop_idx + 1}/{self._loops}  "
            f"航点 {self._wp_idx + 1}/{len(WAYPOINTS)}] "
            f"→ x={gx:.3f}  y={gy:.3f}  yaw={gyaw_deg:.1f}°"
        )
        future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal 被拒绝！停止。")
            rclpy.shutdown()
            return
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f"  剩余距离: {dist:.3f} m", throttle_duration_sec=1.0
        )

    def _result_cb(self, future):
        status = future.result().status
        gx, gy, gyaw_deg = WAYPOINTS[self._wp_idx]

        if status == 4:
            self.get_logger().info("✓ 到达！")
            self._record_error(gx, gy, gyaw_deg)
        else:
            self.get_logger().warn(f"✗ 导航结束，状态码: {status}，跳过误差记录。")

        # 推进航点/循环计数
        self._wp_idx += 1
        if self._wp_idx >= len(WAYPOINTS):
            self._wp_idx = 0
            self._loop_idx += 1

        self._send_next()

    # ── 单次误差记录并实时打印 ────────────────────────────────────────────────
    def _record_error(self, gx, gy, gyaw_deg):
        if self._actual_pose is None:
            self.get_logger().warn("未收到位姿数据，跳过误差记录。")
            return

        p = self._actual_pose
        ax, ay = p.position.x, p.position.y
        actual_yaw_deg = math.degrees(
            quaternion_to_yaw(
                p.orientation.x, p.orientation.y,
                p.orientation.z, p.orientation.w,
            )
        )
        dx = ax - gx
        dy = ay - gy
        dist_err = math.hypot(dx, dy)
        yaw_err_deg = math.degrees(
            angle_diff(math.radians(actual_yaw_deg), math.radians(gyaw_deg))
        )

        self._records.append(
            (self._loop_idx + 1, self._wp_idx + 1,
             dx, dy, dist_err, yaw_err_deg)
        )

        self.get_logger().info(
            f"  误差 │ Δx={dx:+.4f} m  Δy={dy:+.4f} m  "
            f"距离={dist_err:.4f} m  Δyaw={yaw_err_deg:+.2f}°"
        )

    # ── 汇总统计 ──────────────────────────────────────────────────────────────
    def _print_summary(self):
        if not self._records:
            self.get_logger().info("无误差记录，汇总跳过。")
            return

        sep = "═" * 72
        thin = "─" * 72
        lines = [
            f"\n{sep}",
            f"  导航精度汇总  （{self._loops} 次循环 × {len(WAYPOINTS)} 个航点）",
            f"{thin}",
            f"  {'循环':>4}  {'航点':>4}  {'Δx(m)':>9}  {'Δy(m)':>9}  "
            f"{'距离误差(m)':>12}  {'Δyaw(°)':>9}",
            f"{thin}",
        ]
        for loop, wp, dx, dy, dist, yaw_e in self._records:
            lines.append(
                f"  {loop:>4}  {wp:>4}  {dx:>+9.4f}  {dy:>+9.4f}  "
                f"{dist:>12.4f}  {yaw_e:>+9.2f}"
            )

        def stats(vals, signed=True):
            """返回 (均值, 最大, 最小) 字符串，signed=True 保留符号。"""
            fmt = "+.2f" if signed else ".4f"
            mean = sum(vals) / len(vals)
            return (
                f"均值={mean:{fmt}}"
                f"  最大={max(vals):{fmt}}"
                f"  最小={min(vals):{fmt}}"
            )

        # 按航点分组统计
        lines.append(f"{thin}")
        wp_count = len(WAYPOINTS)
        for wi in range(wp_count):
            subset = [r for r in self._records if r[1] == wi + 1]
            if not subset:
                continue
            dists = [r[4] for r in subset]
            yaws = [r[5] for r in subset]
            yaws_abs = [abs(e) for e in yaws]
            lines += [
                f"  ── 航点 {wi+1} (n={len(subset)}) ──",
                f"    距离误差(m)    {stats(dists, signed=False)}",
                f"    朝向误差 有符号(°)  {stats(yaws, signed=True)}",
                f"    朝向误差 绝对值(°)  {stats(yaws_abs, signed=False)}",
            ]

        all_dists = [r[4] for r in self._records]
        all_yaws = [r[5] for r in self._records]
        all_yaws_abs = [abs(e) for e in all_yaws]
        lines += [
            f"{thin}",
            f"  ── 全局汇总 (n={len(self._records)}) ──",
            f"    距离误差(m)    {stats(all_dists, signed=False)}",
            f"    朝向误差 有符号(°)  {stats(all_yaws, signed=True)}",
            f"    朝向误差 绝对值(°)  {stats(all_yaws_abs, signed=False)}",
            f"{sep}\n",
        ]
        self.get_logger().info("\n".join(lines))


def main():
    args = sys.argv[1:]
    try:
        loops = int(args[0]) if args else DEFAULT_LOOPS
    except ValueError:
        print("参数格式错误，用法: python3 send_nav_goal.py [loops]")
        sys.exit(1)

    rclpy.init()
    node = NavLooper(loops)
    node.start()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
