from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory('hospital_escort_mvp'))
        / 'config' / 'mapping_planner.yaml'
    )
    return LaunchDescription([
        Node(
            package='hospital_escort_mvp', executable='mock_vbot',
            name='mock_vbot', output='screen',
        ),
        Node(
            package='hospital_escort_mvp', executable='mapping_planner',
            name='hospital_mapping_planner', output='screen',
            parameters=[config, {'cloud_topic': '/mock/lidar/points'}],
        ),
    ])
