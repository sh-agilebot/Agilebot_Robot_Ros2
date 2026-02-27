import os
from glob import glob
from typing import List
from setuptools import find_packages, setup

package_name = "gbt_description"


def generate_data_files(directory: str, types: List[str]):
    """
    Generate data files for the package.
    Args:
        directory (str): The directory to search for files._
        types (List[str]): The types of robot to search for.
    Returns:
        list: A list of tuples containing the directory and the files.
    """
    return [
        (
            os.path.join("share", package_name, directory, sub_dir),
            glob(os.path.join(directory, sub_dir, "*")),
        )
        for sub_dir in types
    ]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # meshes visual files
        *generate_data_files("meshes/visual", ["c5a", "c12a", "c7a", "c16a"]),
        # meshes collision files
        *generate_data_files("meshes/collision", ["c5a", "c12a", "c7a", "c16a"]),
        # urdf files
        (os.path.join("share", package_name, "urdf"), glob(os.path.join("urdf", "*"))),
        # launch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*")),
        ),
        # config files
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*")),
        ),
        # rviz files
        (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz", "*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Agilebot",
    maintainer_email="info@agilebot.com.cn",
    description="The gbt_description is a ROS 2 package used for describing robot models. The package contains URDF (Unified Robot Description Format) files, related 3D models, and configuration files. It can be loaded by other components in ROS 2 (such as MoveIt2, Gazebo, etc.) for tasks such as simulation, planning, and control.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
