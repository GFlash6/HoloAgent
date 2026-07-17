# relative_nav_node.py
# 相对导航节点 - 将相对位置转换为绝对目标位姿

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from transforms3d import euler, quaternions
import math


class RelativeNavNode(Node):
    def __init__(self):
        super().__init__('relative_nav_node')

        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅相对导航命令 (格式: "forward,left,degrees")
        self.relative_sub = self.create_subscription(
            String,
            '/relative_nav',
            self.relative_nav_callback,
            10
        )

        # 发布绝对目标位姿
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/object_pose',
            10
        )

        self.get_logger().info('Relative navigation node initialized')
        self.get_logger().info('Subscribing to: /relative_nav')
        self.get_logger().info('Publishing to: /object_pose')

    def get_current_pose(self):
        """获取机器人当前位姿 (base_link相对于map)"""
        try:
            # 等待transform可用
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 提取位置
            position = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]

            # 提取四元数
            orientation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]

            return position, orientation

        except Exception as e:
            self.get_logger().error(f'Failed to get transform: {str(e)}')
            return None, None

    def relative_nav_callback(self, msg):
        """处理相对导航消息."""
        try:
            # 解析相对位置参数
            data = msg.data.strip()
            parts = data.split(',')

            if len(parts) != 3:
                self.get_logger().error(
                    f'Invalid format: {data}, expected "forward,left,degrees"')
                return

            forward = float(parts[0].strip())
            left = float(parts[1].strip())
            degrees = float(parts[2].strip())

            self.get_logger().info(
                f'Received relative command: forward={forward}, left={left}, degrees={degrees}')

            # 获取当前位置
            position, orientation = self.get_current_pose()
            if position is None:
                self.get_logger().error('Cannot get current pose, aborting')
                return

            # 计算目标绝对位置
            # 1. 将相对偏移转换为全局偏移
            # 使用四元数旋转局部坐标到全局坐标
            # transforms3d quaternions模块使用 (w, x, y, z) 格式
            q = orientation  # (x, y, z, w) from ROS

            # 构建局部偏移向量 (forward, left, 0)
            local_offset = [forward, left, 0.0]

            # 四元数转旋转矩阵
            # quat2mat takes (w, x, y, z)
            rotation_matrix = quaternions.quat2mat((q[3], q[0], q[1], q[2]))

            # 旋转局部偏移
            global_offset = [
                rotation_matrix[0][0] *
                local_offset[0] +
                rotation_matrix[0][1] *
                local_offset[1] +
                rotation_matrix[0][2] *
                local_offset[2],
                rotation_matrix[1][0] *
                local_offset[0] +
                rotation_matrix[1][1] *
                local_offset[1] +
                rotation_matrix[1][2] *
                local_offset[2],
                rotation_matrix[2][0] *
                local_offset[0] +
                rotation_matrix[2][1] *
                local_offset[1] +
                rotation_matrix[2][2] *
                local_offset[2]]

            # 计算目标位置
            goal_x = position[0] + global_offset[0]
            goal_y = position[1] + global_offset[1]
            goal_z = position[2] + global_offset[2]

            # 2. 计算目标偏航角
            # 四元数转欧拉角 (roll, pitch, yaw)
            current_euler = euler.quat2euler((q[3], q[0], q[1], q[2]))
            # current_euler: (roll, pitch, yaw) in radians
            current_yaw = current_euler[2]

            # 目标偏航角 = 当前偏航角 + 旋转角度(度转弧度)
            goal_yaw = current_yaw + math.radians(degrees)

            # 欧拉角转四元数
            goal_quat = euler.euler2quat(0.0, 0.0, goal_yaw)
            # euler2quat returns (w, x, y, z), we need (x, y, z, w) for ROS

            # 创建PoseStamped消息
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.header.stamp = self.get_clock().now().to_msg()

            pose_msg.pose.position.x = goal_x
            pose_msg.pose.position.y = goal_y
            pose_msg.pose.position.z = 0.0

            pose_msg.pose.orientation.x = goal_quat[1]
            pose_msg.pose.orientation.y = goal_quat[2]
            pose_msg.pose.orientation.z = goal_quat[3]
            pose_msg.pose.orientation.w = goal_quat[0]

            # 发布目标位姿
            self.goal_pub.publish(pose_msg)

            self.get_logger().info(
                f'Goal published: x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(goal_yaw):.2f} deg')

        except Exception as e:
            self.get_logger().error(f'Error processing relative nav: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = RelativeNavNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
