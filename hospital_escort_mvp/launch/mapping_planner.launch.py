from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory('hospital_escort_mvp'))
        / 'config' / 'mapping_planner.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(
            package='hospital_escort_mvp',
            executable='mapping_planner',
            name='hospital_mapping_planner',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
