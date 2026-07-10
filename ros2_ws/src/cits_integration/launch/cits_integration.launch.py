import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cits_integration'), 'config', 'cits_integration.yaml')

    return LaunchDescription([
        Node(
            package='cits_integration',
            executable='cits_integration_node',
            name='cits_integration_node',
            output='screen',
            parameters=[config],
        ),
    ])
