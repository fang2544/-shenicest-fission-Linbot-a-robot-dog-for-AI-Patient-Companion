from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('hospital_escort_mvp'))
    restroom_share = Path(get_package_share_directory('restroom_priority_assistance'))
    graph = str(share / 'config' / 'showroom_graph.yaml')
    scenario = str(share / 'config' / 'cardiology_itinerary.yaml')
    wait_duration = LaunchConfiguration('demo_wait_duration_sec')
    navigation_step = LaunchConfiguration('navigation_step_sec')
    return LaunchDescription([
        DeclareLaunchArgument('demo_wait_duration_sec', default_value='30.0'),
        DeclareLaunchArgument('navigation_step_sec', default_value='0.05'),
        Node(
            package='hospital_escort_mvp', executable='mock_vbot',
            name='mock_vbot', output='screen',
            parameters=[{'navigation_step_sec': navigation_step}],
        ),
        Node(
            package='hospital_escort_mvp', executable='pickup_dispatcher',
            name='hospital_mission_dispatcher', output='screen',
            parameters=[{
                'graph_path': graph,
                'home_node': 'cardiology_3f',
                'navigator_auto_confirm_elevator': True,
                'navigator_report_path': '/vbot_ws/reports/cardiology_route.json',
            }],
        ),
        Node(
            package='restroom_priority_assistance',
            executable='restroom_priority_node',
            name='restroom_priority_node',
            output='screen',
            parameters=[{
                'config_path': str(restroom_share / 'config' / 'restroom_priority.yaml')
            }],
        ),
        Node(
            package='hospital_escort_mvp', executable='cardiology_itinerary',
            name='cardiology_itinerary', output='screen',
            parameters=[{
                'scenario_path': scenario,
                'graph_path': graph,
                'use_demo_timing': True,
                'demo_wait_duration_sec': wait_duration,
            }],
        ),
    ])
