import os
from glob import glob

from setuptools import find_packages, setup

package_name = "gbt_stacking"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # rviz config files
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        # config yaml files
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Agilebot",
    maintainer_email="info@agilebot.com.cn",
    description="This package implements the robot stacking functionality.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_stacking = gbt_stacking.robot_stacking_node:main",
            "stacking_visualizer = gbt_stacking.stacking_visualizer:main",
        ],
    },
)
