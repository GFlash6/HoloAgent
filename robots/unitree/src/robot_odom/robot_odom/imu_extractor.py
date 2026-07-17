#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from unitree_go.msg import SportModeState


class ImuExtractor(Node):
    def __init__(self):
        super().__init__('robot_odom')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=2000,
        )

        self.subscription = self.create_subscription(
            SportModeState,
            '/lf/odommodestate',  # default, 20hz, for slam
            # '/odommodestate', # 500hz, use for handeye-calib
            self.sportmode_callback,
            qos_profile,
        )

        self.imu_publisher = self.create_publisher(
            Imu,
            '/robot_imu',
            2000,
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/robot_odom',
            2000,
        )

        self.get_logger().info('IMU and Odometry Extractor Node has been started')

    def sportmode_callback(self, msg):
        current_time = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.orientation.w = float(msg.imu_state.quaternion[0])
        imu_msg.orientation.x = float(msg.imu_state.quaternion[1])
        imu_msg.orientation.y = float(msg.imu_state.quaternion[2])
        imu_msg.orientation.z = float(msg.imu_state.quaternion[3])

        imu_msg.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu_msg.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu_msg.angular_velocity.z = float(msg.imu_state.gyroscope[2])

        imu_msg.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu_msg.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu_msg.linear_acceleration.z = float(msg.imu_state.accelerometer[2])

        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance = [0.0] * 9

        self.imu_publisher.publish(imu_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'imu_link'

        odom_msg.pose.pose.position.x = float(msg.position[0])
        odom_msg.pose.pose.position.y = float(msg.position[1])
        odom_msg.pose.pose.position.z = float(msg.position[2])

        odom_msg.twist.twist.linear.x = float(msg.velocity[0])
        odom_msg.twist.twist.linear.y = float(msg.velocity[1])
        odom_msg.twist.twist.linear.z = float(msg.velocity[2])

        odom_msg.pose.pose.orientation = imu_msg.orientation
        odom_msg.twist.twist.angular = imu_msg.angular_velocity

        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.twist.covariance = [0.0] * 36

        self.odom_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    imu_extractor = ImuExtractor()

    executor = MultiThreadedExecutor()
    executor.add_node(imu_extractor)

    try:
        executor.spin()
    finally:
        imu_extractor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
