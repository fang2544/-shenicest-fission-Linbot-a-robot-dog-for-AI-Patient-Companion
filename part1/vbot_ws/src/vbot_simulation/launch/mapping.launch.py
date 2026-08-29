from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('vbot_simulation'))
    gui = LaunchConfiguration('gui')
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'sim_core.launch.py')),
        launch_arguments={
            'gui': gui,
            'dynamic_obstacle': 'false',
            'initial_mode': '2',
        }.items(),
    )
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[str(share / 'config' / 'slam_toolbox.yaml'), {'use_sim_time': True}],
    )
    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': True, 'save_map_timeout': 15.0}],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_saver']}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        core,
        slam,
        map_saver,
        lifecycle,
    ])
