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
    lost_share = Path(get_package_share_directory('lost_pause_reminder'))
    wake_share = Path(get_package_share_directory('wakeup_welcome'))

    cardiology = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(sim_share / 'launch' / 'cardiology_nav.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gazebo_gui'),
            'map': LaunchConfiguration('map'),
            'dynamic_obstacle': LaunchConfiguration('dynamic_obstacle'),
            'demo_wait_duration_sec': LaunchConfiguration('demo_wait_duration_sec'),
            'navigator_navigation_timeout_sec': LaunchConfiguration(
                'navigator_navigation_timeout_sec'
            ),
        }.items(),
    )
    wakeup = Node(
        package='wakeup_welcome',
        executable='wakeup_welcome_node',
        name='wakeup_welcome_node',
        output='screen',
        parameters=[
            str(wake_share / 'config' / 'wakeup_welcome.yaml'),
            {'use_sim_time': True},
        ],
    )
    lost = Node(
        package='lost_pause_reminder',
        executable='lost_pause_reminder_node',
        name='lost_pause_reminder_node',
        output='screen',
        parameters=[
            str(lost_share / 'config' / 'lost_pause_reminder.yaml'),
            {
                'use_sim_time': True,
                'control_following_service': '/control_following',
                'goal_nav_action': '/goal_nav',
            },
        ],
    )
    harness = Node(
        package='vbot_simulation',
        executable='second_dev_demo',
        name='second_dev_demo',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'auto_start_cardiology': LaunchConfiguration('auto_start_cardiology'),
            'graph_path': str(
                sim_share / 'config' / 'simulation_cardiology_graph.yaml'
            ),
            'report_path': LaunchConfiguration('report_path'),
            'html_report_path': LaunchConfiguration('html_report_path'),
        }],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(sim_share / 'rviz' / 'second_dev_demo.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('dynamic_obstacle', default_value='true'),
        DeclareLaunchArgument('auto_start_cardiology', default_value='true'),
        DeclareLaunchArgument('demo_wait_duration_sec', default_value='3.0'),
        DeclareLaunchArgument(
            'navigator_navigation_timeout_sec', default_value='900.0'
        ),
        DeclareLaunchArgument(
            'report_path',
            default_value='/vbot_ws/reports/second_dev_acceptance.json',
        ),
        DeclareLaunchArgument(
            'html_report_path',
            default_value='/vbot_ws/reports/second_dev_visual_report.html',
        ),
        DeclareLaunchArgument('map', default_value='/vbot_ws/maps/hospital_sim.yaml'),
        cardiology,
        wakeup,
        lost,
        harness,
        rviz,
    ])
