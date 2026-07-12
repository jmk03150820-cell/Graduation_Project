import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('perception_markers'), 'config', 'perception_markers.yaml')

    return LaunchDescription([
        Node(
            package='perception_markers',
            executable='perception_markers_node',
            name='perception_markers_node',
            output='screen',
            parameters=[config],
        ),
    ])
