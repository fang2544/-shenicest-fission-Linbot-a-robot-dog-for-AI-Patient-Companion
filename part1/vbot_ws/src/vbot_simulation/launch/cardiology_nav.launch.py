from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = Path(get_package_share_directory('vbot_simulation'))
    escort_share = Path(get_package_share_directory('hospital_escort_mvp'))
    restroom_share = Path(get_package_share_directory('restroom_priority_assistance'))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_share / 'launch' / 'navigation.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'map': LaunchConfiguration('map'),
            'dynamic_obstacle': LaunchConfiguration('dynamic_obstacle'),
        }.items(),
    )
    dispatcher = Node(
        package='hospital_escort_mvp',
        executable='pickup_dispatcher',
        name='hospital_mission_dispatcher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'graph_path': str(sim_share / 'config' / 'simulation_cardiology_graph.yaml'),
            'home_node': 'cardiology_3f',
            'navigator_auto_confirm_elevator': True,
            'navigator_navigation_timeout_sec': LaunchConfiguration(
                'navigator_navigation_timeout_sec'
            ),
            'navigator_report_path': '/vbot_ws/reports/cardiology_nav2_route.json',
        }],
    )
    itinerary = Node(
        package='hospital_escort_mvp',
        executable='cardiology_itinerary',
        name='cardiology_itinerary',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scenario_path': str(escort_share / 'config' / 'cardiology_itinerary.yaml'),
            'graph_path': str(sim_share / 'config' / 'simulation_cardiology_graph.yaml'),
            'use_demo_timing': True,
            'demo_wait_duration_sec': LaunchConfiguration('demo_wait_duration_sec'),
            'report_path': '/vbot_ws/reports/cardiology_nav2_dialogue.json',
        }],
    )
    restroom_priority = Node(
        package='restroom_priority_assistance',
        executable='restroom_priority_node',
        name='restroom_priority_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_path': str(restroom_share / 'config' / 'restroom_priority.yaml'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('dynamic_obstacle', default_value='true'),
        DeclareLaunchArgument('map', default_value='/vbot_ws/maps/hospital_sim.yaml'),
        DeclareLaunchArgument('demo_wait_duration_sec', default_value='3.0'),
        DeclareLaunchArgument(
            'navigator_navigation_timeout_sec', default_value='240.0'
        ),
        navigation,
        dispatcher,
        restroom_priority,
        itinerary,
    ])
