import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cits_adapter'), 'config', 'cits_adapter.yaml')

    return LaunchDescription([
        Node(
            package='cits_adapter',
            executable='cits_adapter_node',
            name='cits_adapter_node',
            output='screen',
            parameters=[config],
        ),
    ])
