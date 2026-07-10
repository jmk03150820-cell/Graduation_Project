import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('fake_rsu'), 'config', 'fake_rsu.yaml')

    return LaunchDescription([
        Node(
            package='fake_rsu',
            executable='fake_rsu_node',
            name='fake_rsu_node',
            output='screen',
            parameters=[config],
        ),
    ])
