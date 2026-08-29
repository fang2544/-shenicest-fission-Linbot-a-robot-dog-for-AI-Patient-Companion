from setuptools import find_packages, setup


package_name = "lost_pause_reminder"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/lost_pause_reminder.launch.py"]),
        ("share/" + package_name + "/config", ["config/lost_pause_reminder.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="聆灵 VBot 二次开发团队",
    maintainer_email="developer@example.com",
    description="UWB 跟随目标丢失后暂停运动并提醒用户。",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lost_pause_reminder_node = "
            "lost_pause_reminder.lost_pause_reminder_node:main",
        ],
    },
)
