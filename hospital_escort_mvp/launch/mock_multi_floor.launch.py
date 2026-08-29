from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    mock = Node(
        package='hospital_escort_mvp',
        executable='mock_vbot',
        name='mock_vbot',
        output='screen',
    )
    navigator = Node(
        package='hospital_escort_mvp',
        executable='multi_floor_navigator',
        name='multi_floor_navigator',
        output='screen',
        parameters=[{
            'start_node': 'parking',
            'goal_node': 'cardiology',
            'auto_confirm_elevator': True,
            'interface_timeout_sec': 10.0,
            'navigation_timeout_sec': 10.0,
            'map_switch_timeout_sec': 10.0,
        }],
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=navigator,
            on_exit=[EmitEvent(event=Shutdown(reason='multi-floor demo complete'))],
        )
    )
    return LaunchDescription([mock, navigator, shutdown])
