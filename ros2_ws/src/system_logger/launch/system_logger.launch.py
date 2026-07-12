import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('system_logger'), 'config', 'system_logger.yaml')

    return LaunchDescription([
        Node(
            package='system_logger',
            executable='system_logger_node',
            name='system_logger_node',
            output='screen',
            parameters=[config],
        ),
    ])
