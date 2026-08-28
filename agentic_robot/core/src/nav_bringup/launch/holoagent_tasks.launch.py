"""Task-level ROS graph; model HTTP processes remain separate dependencies."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    common = [{"use_sim_time": True}]
    semantic_map_mode = LaunchConfiguration("semantic_map_mode")
    contract_trace_path = LaunchConfiguration("contract_trace_path")
    contract_policy_path = LaunchConfiguration("contract_policy_path")
    require_localization_tracking = LaunchConfiguration(
        "require_localization_tracking"
    )
    require_plannable_goal = LaunchConfiguration("require_plannable_goal")
    require_backend_threshold = LaunchConfiguration("require_backend_threshold")
    require_completion_confirmation = LaunchConfiguration(
        "require_completion_confirmation"
    )
    goal_candidate_count = LaunchConfiguration("goal_candidate_count")
    max_observation_retries = LaunchConfiguration("max_observation_retries")
    navigation_timeout_sec = LaunchConfiguration("navigation_timeout_sec")
    completion_goal_tolerance_m = LaunchConfiguration(
        "completion_goal_tolerance_m"
    )
    observation_max_path_m = LaunchConfiguration("observation_max_path_m")
    execution_stall_timeout_sec = LaunchConfiguration("execution_stall_timeout_sec")
    max_execution_recoveries = LaunchConfiguration("max_execution_recoveries")
    exploration_max_steps = LaunchConfiguration("exploration_max_steps")
    exploration_timeout_sec = LaunchConfiguration("exploration_timeout_sec")
    exploration_max_path_m = LaunchConfiguration("exploration_max_path_m")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "semantic_map_mode",
                default_value="online",
                description="Use the online OVO map or optional prior HMSG baseline",
            ),
            DeclareLaunchArgument(
                "contract_trace_path",
                default_value="",
                description="Optional JSONL path for semantic-navigation evidence traces",
            ),
            DeclareLaunchArgument(
                "contract_policy_path",
                default_value="",
                description="Optional independently certified ContractNav policy",
            ),
            DeclareLaunchArgument(
                "require_localization_tracking",
                default_value="true",
                description="Reject semantic dispatch unless localization is TRACKING",
            ),
            DeclareLaunchArgument(
                "require_plannable_goal",
                default_value="true",
                description="Reject semantic dispatch when Nav2 cannot plan a path",
            ),
            DeclareLaunchArgument(
                "require_backend_threshold",
                default_value="true",
                description="Apply the original semantic backend threshold baseline",
            ),
            DeclareLaunchArgument(
                "require_completion_confirmation",
                default_value="false",
                description="Require post-arrival online semantic confirmation",
            ),
            DeclareLaunchArgument(
                "goal_candidate_count",
                default_value="1",
                description="Shared Nav2-probed standoff candidates (1 or up to 16)",
            ),
            DeclareLaunchArgument(
                "max_observation_retries",
                default_value="0",
                description="Maximum calibrated semantic observation interventions",
            ),
            DeclareLaunchArgument(
                "navigation_timeout_sec",
                default_value="300.0",
                description="Per-stage navigation timeout in seconds",
            ),
            DeclareLaunchArgument(
                "completion_goal_tolerance_m",
                default_value="0.5",
                description="Independent localization-to-commanded-goal postcondition",
            ),
            DeclareLaunchArgument(
                "observation_max_path_m",
                default_value="2.0",
                description="Maximum Nav2 path length for one observation intervention",
            ),
            DeclareLaunchArgument(
                "execution_stall_timeout_sec",
                default_value="20.0",
                description="Navigation-clock seconds without 0.05 m progress",
            ),
            DeclareLaunchArgument(
                "max_execution_recoveries",
                default_value="3",
                description="Maximum Nav2 recovery behaviors per navigation leg",
            ),
            DeclareLaunchArgument(
                "exploration_max_steps",
                default_value="8",
                description="Maximum frontier goals before safe refusal",
            ),
            DeclareLaunchArgument(
                "exploration_timeout_sec",
                default_value="300.0",
                description="Total frontier exploration time budget",
            ),
            DeclareLaunchArgument(
                "exploration_max_path_m",
                default_value="8.0",
                description="Maximum known-space path length for one frontier",
            ),
            Node(
                package="semantic_map_bridge",
                executable="semantic_map_bridge_node",
                parameters=common + [{"backend_mode": semantic_map_mode}],
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
                parameters=common + [{
                    "trace_path": contract_trace_path,
                    "contract_policy_path": contract_policy_path,
                    "require_localization_tracking": require_localization_tracking,
                    "require_plannable_goal": require_plannable_goal,
                    "require_backend_threshold": require_backend_threshold,
                    "require_completion_confirmation": require_completion_confirmation,
                    "goal_candidate_count": goal_candidate_count,
                    "max_observation_retries": max_observation_retries,
                    "navigation_timeout_sec": navigation_timeout_sec,
                    "completion_goal_tolerance_m": completion_goal_tolerance_m,
                    "observation_max_path_m": observation_max_path_m,
                    "execution_stall_timeout_sec": execution_stall_timeout_sec,
                    "max_execution_recoveries": max_execution_recoveries,
                    "exploration_max_steps": exploration_max_steps,
                    "exploration_timeout_sec": exploration_timeout_sec,
                    "exploration_max_path_m": exploration_max_path_m,
                }],
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
