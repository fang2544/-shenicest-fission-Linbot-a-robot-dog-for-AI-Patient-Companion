from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('vbot_simulation'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'navigation.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gazebo_gui'),
            'map': LaunchConfiguration('map'),
            'dynamic_obstacle': LaunchConfiguration('dynamic_obstacle'),
        }.items(),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(nav2_share / 'rviz' / 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('dynamic_obstacle', default_value='true'),
        DeclareLaunchArgument('map', default_value='/vbot_ws/maps/hospital_sim.yaml'),
        navigation,
        rviz,
    ])
