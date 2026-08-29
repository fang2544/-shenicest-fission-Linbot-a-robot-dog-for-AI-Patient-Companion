from glob import glob
from setuptools import find_packages, setup


package_name = 'hospital_escort_mvp'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='聆灵 VBot 二次开发团队',
    maintainer_email='developer@example.com',
    description='医院建图、定位、陪诊导航和等待时间感知任务规划。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_vbot = hospital_escort_mvp.mock_vbot_node:main',
            'capability_probe = hospital_escort_mvp.capability_probe_node:main',
            'escort_demo = hospital_escort_mvp.escort_demo_node:main',
            'mapping_planner = hospital_escort_mvp.mapping_planner_node:main',
            'multi_floor_navigator = hospital_escort_mvp.multi_floor_nav_node:main',
            'hospital_graph_export = hospital_escort_mvp.graph_export:main',
            'floor_surveyor = hospital_escort_mvp.floor_survey_node:main',
            'pickup_dispatcher = hospital_escort_mvp.pickup_dispatch_node:main',
            'showroom_voice = hospital_escort_mvp.showroom_voice_node:main',
            'cardiology_itinerary = hospital_escort_mvp.cardiology_itinerary_node:main',
        ],
    },
)
