from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = Path(get_package_share_directory('vbot_simulation'))
    aed_share = Path(get_package_share_directory('aed_emergency_response'))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / 'launch' / 'navigation.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gazebo_gui'),
            'map': LaunchConfiguration('map'),
            'dynamic_obstacle': LaunchConfiguration('dynamic_obstacle'),
        }.items(),
    )
    responder = Node(
        package='aed_emergency_response',
        executable='aed_emergency_node',
        name='aed_emergency_node',
        output='screen',
        parameters=[
            str(aed_share / 'config' / 'aed_emergency.yaml'),
            {
                'use_sim_time': True,
                'guidance_interval_sec': LaunchConfiguration('guidance_interval_sec'),
                'navigation_timeout_sec': LaunchConfiguration('navigation_timeout_sec'),
            },
        ],
    )
    harness = Node(
        package='vbot_simulation',
        executable='aed_sim_harness',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'request_delay_sec': LaunchConfiguration('request_delay_sec'),
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', str(sim_share / 'rviz' / 'second_dev_demo.rviz')],
        parameters=[{'use_sim_time': True}], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('dynamic_obstacle', default_value='false'),
        DeclareLaunchArgument('map', default_value='/vbot_ws/maps/hospital_sim.yaml'),
        DeclareLaunchArgument('request_delay_sec', default_value='45.0'),
        DeclareLaunchArgument('guidance_interval_sec', default_value='0.5'),
        DeclareLaunchArgument('navigation_timeout_sec', default_value='300.0'),
        navigation, responder, harness, rviz,
    ])
