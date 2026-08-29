from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    graph = str(
        Path(get_package_share_directory('hospital_escort_mvp'))
        / 'config'
        / 'showroom_graph.yaml'
    )
    return LaunchDescription([
        Node(
            package='hospital_escort_mvp', executable='mock_vbot',
            name='mock_vbot', output='screen',
        ),
        Node(
            package='hospital_escort_mvp', executable='pickup_dispatcher',
            name='hospital_pickup_dispatcher', output='screen',
            parameters=[{
                'graph_path': graph,
                'home_node': 'cardiology_3f',
                'navigator_auto_confirm_elevator': True,
            }],
        ),
        Node(
            package='hospital_escort_mvp', executable='showroom_voice',
            name='showroom_voice_interaction', output='screen',
        ),
    ])
