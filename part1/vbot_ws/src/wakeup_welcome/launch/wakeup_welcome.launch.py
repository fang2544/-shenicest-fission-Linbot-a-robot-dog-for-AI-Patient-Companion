from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory("wakeup_welcome"))
        / "config"
        / "wakeup_welcome.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="wakeup_welcome",
                executable="wakeup_welcome_node",
                name="wakeup_welcome_node",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )
