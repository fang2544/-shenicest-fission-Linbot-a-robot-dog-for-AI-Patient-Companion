from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('aed_emergency_response'))
    return LaunchDescription([
        Node(
            package='aed_emergency_response',
            executable='aed_emergency_node',
            name='aed_emergency_node',
            output='screen',
            parameters=[str(share / 'config' / 'aed_emergency.yaml')],
        )
    ])
