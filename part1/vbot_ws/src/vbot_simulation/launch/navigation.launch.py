from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = Path(get_package_share_directory('vbot_simulation'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    gui = LaunchConfiguration('gui')
    map_yaml = LaunchConfiguration('map')
    params = LaunchConfiguration('params_file')
    dynamic = LaunchConfiguration('dynamic_obstacle')
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / 'sim_core.launch.py')),
        launch_arguments={
            'gui': gui,
            'dynamic_obstacle': dynamic,
            'initial_mode': '3',
        }.items(),
    )
    tuned_params = RewrittenYaml(
        source_file=params,
        root_key='',
        param_rewrites={
            # 避免 amd64 模拟短暂停顿导致生命周期节点被误重置。
            'bond_timeout': '0.0',
            # 医院机器人需要允许等待横穿通道的行人。
            'movement_time_allowance': '60.0',
            'xy_goal_tolerance': '0.25',
            'yaw_goal_tolerance': '0.5',
        },
        convert_types=True,
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'true',
            'params_file': tuned_params,
            'autostart': 'true',
            # 将 Nav2 生命周期节点放入同一组件容器，减少 DDS 服务通信，
            # 提高 Apple Silicon 模拟 amd64 镜像时的稳定性。
            # Humble 在此处要求使用 Python 布尔值写法。
            'use_composition': 'True',
        }.items(),
    )
    action_bridge = Node(
        package='vbot_simulation',
        executable='goal_nav_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('dynamic_obstacle', default_value='true'),
        DeclareLaunchArgument('map', default_value='/vbot_ws/maps/hospital_sim.yaml'),
        DeclareLaunchArgument('params_file', default_value=str(nav2_share / 'params' / 'nav2_params.yaml')),
        core,
        # Apple Silicon 模拟 amd64 时 Gazebo 启动较慢；模型生成后再启动 Nav2，
        # 避免 odom 到 base_footprint 的 TF 尚未发布就触发生命周期超时。
        TimerAction(period=10.0, actions=[nav2, action_bridge]),
    ])
