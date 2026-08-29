from setuptools import find_packages, setup


package_name = "wakeup_welcome"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/wakeup_welcome.launch.py"]),
        ("share/" + package_name + "/config", ["config/wakeup_welcome.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="聆灵 VBot 二次开发团队",
    maintainer_email="developer@example.com",
    description="收到 VBot 唤醒事件后播放欢迎表情和语音。",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "wakeup_welcome_node = wakeup_welcome.wakeup_welcome_node:main",
        ],
    },
)
