from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fail_navigation', default_value='false'),
        Node(
            package='hospital_escort_mvp',
            executable='mock_vbot',
            name='mock_vbot',
            output='screen',
            parameters=[{
                'navigation_step_sec': 0.05,
                'fail_navigation': ParameterValue(
                    LaunchConfiguration('fail_navigation'), value_type=bool
                ),
            }],
        ),
        Node(
            package='hospital_escort_mvp',
            executable='escort_demo',
            name='escort_demo',
            output='screen',
            parameters=[{
                'interface_timeout_sec': 10.0,
                'navigation_timeout_sec': 10.0,
                'arrival_wait_sec': 0.05,
            }],
        ),
    ])
