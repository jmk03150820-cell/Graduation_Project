import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('module_integrate'), 'config', 'module_integrate.yaml')

    return LaunchDescription([
        Node(
            package='module_integrate',
            executable='module_integrate_node',
            name='module_integrate_node',
            output='screen',
            parameters=[config],
        ),
    ])
