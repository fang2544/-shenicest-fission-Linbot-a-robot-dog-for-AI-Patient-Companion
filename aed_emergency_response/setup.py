from glob import glob
from setuptools import find_packages, setup


package_name = 'aed_emergency_response'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='聆灵 VBot 二次开发团队',
    maintainer_email='developer@example.com',
    description='语音触发的 VBot 急救物资配送和现场指导模式。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aed_emergency_node = '
            'aed_emergency_response.aed_emergency_node:main',
        ],
    },
)
