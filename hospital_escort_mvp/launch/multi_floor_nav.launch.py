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
        / 'multi_floor_nav.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('start_node', default_value='parking'),
        DeclareLaunchArgument('goal_node', default_value='cardiology'),
        Node(
            package='hospital_escort_mvp',
            executable='multi_floor_navigator',
            name='multi_floor_navigator',
            output='screen',
            parameters=[
                config,
                {
                    'start_node': LaunchConfiguration('start_node'),
                    'goal_node': LaunchConfiguration('goal_node'),
                },
            ],
        ),
    ])
