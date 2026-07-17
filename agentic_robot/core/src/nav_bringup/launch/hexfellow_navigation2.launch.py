# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_bringup')
    hexfellow_dir = '/home/workspace/holoagent/agentic_robot_system/robots/hexfellow/'
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    map_dir = LaunchConfiguration(
        'map',
        default='/home/workspace/test_map/all_maps/sh3_513/map/grid_map.yaml'
    )

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(hexfellow_dir, 'config', 'nav_params.yaml')
    )

    rviz_config = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(
            hexfellow_dir,
            'config',
            'rviz',
            'hexfellow_nav.rviz'))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to the Nav2 params yaml file to load'),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config,
            description='Full path to the RViz2 config file to use'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 alongside Nav2 (true/false)'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'chassis_cmd_vel',
            default_value='/chassis/cmd_vel',
            description='Final cmd_vel destination for multi_bringup chassis namespace'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
            }.items(),
        ),

        # Nav2 velocity_smoother publishes to /cmd_vel; relay to
        # /chassis/cmd_vel for multi_bringup.
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_to_chassis',
            arguments=[
                '/cmd_vel',
                LaunchConfiguration('chassis_cmd_vel'),
            ],
            output='screen',
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
