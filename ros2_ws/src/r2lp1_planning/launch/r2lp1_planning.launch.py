import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('r2lp1_planning'), 'config', 'r2lp1_planning.yaml')

    return LaunchDescription([
        Node(
            package='r2lp1_planning',
            executable='r2lp1_planning_node',
            name='r2lp1_planning_node',
            output='screen',
            parameters=[config],
        ),
    ])
