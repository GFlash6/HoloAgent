"""Task-level ROS graph; model HTTP processes remain separate dependencies."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    common = [{"use_sim_time": True}]
    return LaunchDescription(
        [
            Node(
                package="semantic_map_bridge",
                executable="semantic_map_bridge_node",
                parameters=common,
                output="screen",
            ),
            Node(
                package="nav_executor",
                executable="nav_executor_node",
                parameters=common,
                output="screen",
            ),
            Node(
                package="semantic_goal",
                executable="semantic_goal_node",
                parameters=common,
                output="screen",
            ),
            Node(
                package="relative_goal",
                executable="relative_goal_node",
                parameters=common,
                output="screen",
            ),
            Node(
                package="manipulation",
                executable="arm_skill_server",
                parameters=common,
                output="screen",
            ),
        ]
    )
