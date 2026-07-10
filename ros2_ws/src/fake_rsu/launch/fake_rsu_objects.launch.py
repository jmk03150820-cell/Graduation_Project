from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fake_rsu',
            executable='fake_rsu_objects_node',
            name='fake_rsu_objects_node',
            output='screen',
            parameters=[{
                'topic': 'cits/rsu/detected_objects',
                'publish_rate_hz': 5.0,
                'frame_id': 'rsu',
            }],
        ),
    ])
