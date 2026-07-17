from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_odom',
            executable='imu_extractor',
            name='robot_odom',
            output='screen',
        )
    ])
