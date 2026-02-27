from setuptools import find_packages, setup
import os
from glob import glob

package_name = "gbt_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # launch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gbt",
    maintainer_email="info@agilebot.com.cn",
    description="Agilebot robot vision package, used to connect Agilebot vision software AgileGaze, to achieve robot vision function",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # add node
            "gbt_agilegaze= gbt_vision.agilegaze_node:main",
        ],
    },
)
