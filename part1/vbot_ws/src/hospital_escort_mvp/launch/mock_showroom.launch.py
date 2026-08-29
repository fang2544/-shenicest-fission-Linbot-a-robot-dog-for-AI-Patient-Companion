from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    graph = str(
        Path(get_package_share_directory('hospital_escort_mvp'))
        / 'config'
        / 'showroom_graph.yaml'
    )
    mock = Node(
        package='hospital_escort_mvp', executable='mock_vbot',
        name='mock_vbot', output='screen',
    )
    navigator = Node(
        package='hospital_escort_mvp', executable='multi_floor_navigator',
        name='multi_floor_navigator', output='screen',
        parameters=[{
            'graph_path': graph,
            'start_node': 'cardiology_3f',
            'goal_node': 'garage_pickup',
            'auto_confirm_elevator': True,
            'voice_prompts_enabled': True,
            'interface_timeout_sec': 10.0,
            'navigation_timeout_sec': 10.0,
            'map_switch_timeout_sec': 10.0,
        }],
    )
    return LaunchDescription([
        mock,
        navigator,
        RegisterEventHandler(
            OnProcessExit(
                target_action=navigator,
                on_exit=[EmitEvent(event=Shutdown(reason='showroom demo complete'))],
            )
        ),
    ])
