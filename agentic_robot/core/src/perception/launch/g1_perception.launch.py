from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                # 'config_file': 'config/config.yaml'
            }],
        ),
    ])
