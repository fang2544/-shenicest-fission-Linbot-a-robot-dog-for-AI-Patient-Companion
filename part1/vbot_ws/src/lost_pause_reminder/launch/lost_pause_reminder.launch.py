from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory("lost_pause_reminder"))
        / "config"
        / "lost_pause_reminder.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="lost_pause_reminder",
                executable="lost_pause_reminder_node",
                name="lost_pause_reminder_node",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )
