from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('hospital_escort_mvp'))
    default_graph = str(share / 'config' / 'showroom_graph.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('graph_path', default_value=default_graph),
        DeclareLaunchArgument('home_node', default_value='cardiology_3f'),
        Node(
            package='hospital_escort_mvp',
            executable='pickup_dispatcher',
            name='hospital_pickup_dispatcher',
            output='screen',
            parameters=[
                str(share / 'config' / 'pickup_dispatch.yaml'),
                {
                    'graph_path': LaunchConfiguration('graph_path'),
                    'home_node': LaunchConfiguration('home_node'),
                },
            ],
        ),
    ])
