from glob import glob
from setuptools import find_packages, setup


package_name = 'restroom_priority_assistance'

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
    description='把找厕所语音转换为最高优先目的地请求。',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'restroom_priority_node = '
        'restroom_priority_assistance.restroom_priority_node:main',
    ]},
)
