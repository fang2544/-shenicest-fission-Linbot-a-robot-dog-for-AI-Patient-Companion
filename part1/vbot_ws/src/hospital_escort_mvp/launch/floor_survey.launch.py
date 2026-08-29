from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory('hospital_escort_mvp'))
        / 'config'
        / 'floor_survey.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('floor', default_value='F1'),
        DeclareLaunchArgument('map_name', default_value='surveyed_F1'),
        Node(
            package='hospital_escort_mvp',
            executable='floor_surveyor',
            name='hospital_floor_surveyor',
            output='screen',
            parameters=[
                config,
                {
                    'floor': LaunchConfiguration('floor'),
                    'map_name': LaunchConfiguration('map_name'),
                },
            ],
        ),
    ])
