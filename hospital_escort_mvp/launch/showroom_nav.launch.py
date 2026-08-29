from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('hospital_escort_mvp'))
    return LaunchDescription([
        DeclareLaunchArgument('start_node', default_value='cardiology_3f'),
        DeclareLaunchArgument('goal_node', default_value='garage_pickup'),
        Node(
            package='hospital_escort_mvp',
            executable='multi_floor_navigator',
            name='multi_floor_navigator',
            output='screen',
            parameters=[
                str(share / 'config' / 'multi_floor_nav.yaml'),
                {
                    'graph_path': str(share / 'config' / 'showroom_graph.yaml'),
                    'start_node': LaunchConfiguration('start_node'),
                    'goal_node': LaunchConfiguration('goal_node'),
                    'auto_confirm_elevator': False,
                    'voice_prompts_enabled': True,
                },
            ],
        ),
    ])
