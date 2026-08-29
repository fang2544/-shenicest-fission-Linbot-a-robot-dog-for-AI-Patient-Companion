from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('restroom_priority_assistance'))
    return LaunchDescription([
        Node(
            package='restroom_priority_assistance',
            executable='restroom_priority_node',
            name='restroom_priority_node',
            output='screen',
            parameters=[{'config_path': str(share / 'config' / 'restroom_priority.yaml')}],
        )
    ])
