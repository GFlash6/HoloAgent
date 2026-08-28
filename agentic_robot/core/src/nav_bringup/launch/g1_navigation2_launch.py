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
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
# ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    online_mapping = LaunchConfiguration('online_mapping')
    use_rviz = LaunchConfiguration('use_rviz')
    map_dir = LaunchConfiguration('map')
    default_map = '/workspace/hts_demo_maps/ic4f_demo_map/map/grid_map/grid_map.yaml'

    param_file_name = 'g1.yaml'
    param_dir = LaunchConfiguration('params_file')
    default_params = os.path.join(
        get_package_share_directory('nav_bringup'), 'param', param_file_name)

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav_bringup'),
        'rviz',
        'nav_bringup.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'online_mapping',
            default_value='false',
            description='Consume a live /map instead of starting map_server'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            condition=UnlessCondition(online_mapping),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/navigation_launch.py']),
            condition=IfCondition(online_mapping),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
