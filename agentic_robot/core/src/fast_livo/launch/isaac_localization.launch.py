"""Canonical LIO + prior-map localization graph for the Isaac G1 runtime."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("fast_livo")
    base_lio_params = LaunchConfiguration("base_lio_params")
    lio_params = LaunchConfiguration("lio_params")
    camera_params = LaunchConfiguration("camera_params")
    relo_params = LaunchConfiguration("relo_params")
    prior_map = LaunchConfiguration("prior_map")
    localization_status_topic = LaunchConfiguration("localization_status_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_lio_params",
                default_value=os.path.join(
                    package_dir, "config", "mid360_online_livo.yaml"
                ),
                description="Complete FAST-LIVO parameter set",
            ),
            DeclareLaunchArgument(
                "lio_params",
                default_value=os.path.join(
                    package_dir, "config", "mid360_isaac_lio.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "relo_params",
                default_value=os.path.join(
                    package_dir, "config", "mid360_isaac_reloc.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "camera_params",
                default_value=os.path.join(
                    package_dir, "config", "camera_d435i.yaml"
                ),
                description="Required FAST-LIVO camera model; images remain disabled",
            ),
            DeclareLaunchArgument(
                "prior_map",
                description="Directory containing the prepared relocation map",
            ),
            DeclareLaunchArgument(
                "localization_status_topic",
                default_value="localization/status",
                description="LocalizationStatus output; remap only for explicit fault tests",
            ),
            Node(
                package="fast_livo",
                executable="fastlivo_mapping",
                name="lio_frontend",
                parameters=[
                    base_lio_params,
                    camera_params,
                    lio_params,
                    {"use_sim_time": True},
                ],
                remappings=[
                    ("/undistort_cloud", "lio/undistorted_points"),
                    ("/aft_mapped_to_init", "lio/odom"),
                    ("/path", "lio/path"),
                    ("fast_livo/save_map", "lio/save_map"),
                ],
                output="screen",
            ),
            Node(
                package="fast_livo",
                executable="online_relo",
                name="prior_map_localizer",
                parameters=[
                    relo_params,
                    {"use_sim_time": True, "relo.priorDir": prior_map},
                ],
                remappings=[
                    ("/pose", "localization/odom"),
                    ("/reloc_body_cloud", "perception/obstacles"),
                    (
                        "/relocalization/registration_success",
                        "localization/registration_success",
                    ),
                    (
                        "/relocalization/fitness_score",
                        "localization/fitness_score",
                    ),
                ],
                output="screen",
            ),
            Node(
                package="localization_monitor",
                executable="localization_monitor_node",
                parameters=[{"use_sim_time": True}],
                remappings=[("localization/status", localization_status_topic)],
                output="screen",
            ),
        ]
    )
