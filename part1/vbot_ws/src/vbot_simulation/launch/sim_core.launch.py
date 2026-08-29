from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('vbot_simulation'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    gui = LaunchConfiguration('gui')
    dynamic_obstacle = LaunchConfiguration('dynamic_obstacle')
    initial_mode = LaunchConfiguration('initial_mode')
    world = LaunchConfiguration('world')
    robot_description = ParameterValue(
        Command(['xacro ', str(share / 'urdf' / 'vbot_simple.urdf.xacro')]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / 'launch' / 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': 'false',
            'pause': 'false',
        }.items(),
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'vbot', '-x', '-4.5', '-y', '0.0', '-z', '0.08'],
        output='screen',
    )
    compatibility = Node(
        package='vbot_simulation',
        executable='vbot_compat_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'initial_mode': ParameterValue(initial_mode, value_type=int),
        }],
    )
    moving_obstacle = Node(
        package='vbot_simulation',
        executable='dynamic_obstacle',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(dynamic_obstacle),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('dynamic_obstacle', default_value='false'),
        DeclareLaunchArgument('initial_mode', default_value='1'),
        DeclareLaunchArgument('world', default_value=str(share / 'worlds' / 'hospital.world')),
        gazebo,
        state_publisher,
        spawn,
        compatibility,
        moving_obstacle,
    ])
