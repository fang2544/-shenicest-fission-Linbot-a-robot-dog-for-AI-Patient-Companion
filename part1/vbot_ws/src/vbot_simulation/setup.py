from glob import glob
from setuptools import find_packages, setup


package_name = 'vbot_simulation'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='聆灵 VBot 二次开发团队',
    maintainer_email='developer@example.com',
    description='简化 VBot 的 Gazebo、SLAM 和 Nav2 仿真环境。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_obstacle = vbot_simulation.dynamic_obstacle_node:main',
            'goal_nav_bridge = vbot_simulation.goal_nav_bridge_node:main',
            'mapping_driver = vbot_simulation.mapping_driver_node:main',
            'simulation_acceptance = vbot_simulation.simulation_acceptance_node:main',
            'second_dev_demo = vbot_simulation.second_dev_demo_node:main',
            'aed_sim_harness = vbot_simulation.aed_sim_harness_node:main',
            'vbot_compat_bridge = vbot_simulation.vbot_compat_bridge_node:main',
        ],
    },
)
