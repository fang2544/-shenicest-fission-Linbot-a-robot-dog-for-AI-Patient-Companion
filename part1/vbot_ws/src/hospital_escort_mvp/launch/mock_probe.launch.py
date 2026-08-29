from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hospital_escort_mvp',
            executable='mock_vbot',
            name='mock_vbot',
            output='screen',
        ),
        Node(
            package='hospital_escort_mvp',
            executable='capability_probe',
            name='vbot_capability_probe',
            output='screen',
            parameters=[{'discovery_seconds': 2.0}],
        ),
    ])
