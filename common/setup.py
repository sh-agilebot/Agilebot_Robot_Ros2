import os
from glob import glob

from setuptools import find_packages, setup

package_name = "common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # # enum files
        # (os.path.join('share', package_name, 'enum'), glob('enum/*')),
        # # utils files
        # (os.path.join('share', package_name, 'utils'), glob('utils/*')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Agilebot",
    maintainer_email="info@agilebot.com.cn",
    description="common tools",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
