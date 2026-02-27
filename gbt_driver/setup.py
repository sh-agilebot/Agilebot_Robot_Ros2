import os
from glob import glob

from setuptools import find_packages, setup

package_name = "gbt_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # luanch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*")),
        ),
        # config files
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Agilebot",
    maintainer_email="info@agilebot.com.cn",
    description="Agilebot Robot ROS2 Driver",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_controller = gbt_driver.robot_controller:main",
            "robot_bridge = gbt_driver.robot_bridge:main",
            "robot_status = gbt_driver.robot_status:main",
            "service_server = gbt_driver.service_server:main",
            "moveit_action_server = gbt_driver.moveit_action_server:main",
            "trajectory_server = gbt_driver.trajectory_server:main",
        ],
    },
)
