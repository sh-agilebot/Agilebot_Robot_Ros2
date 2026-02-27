"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
This file is used to visualize the URDF robot model in RViz.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define package name and rviz config file name
    package_name = "gbt_description"
    rviz_name = "display.rviz"

    ld = LaunchDescription()

    # the path of package
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Construct  rviz config file path
    rviz_config_path = os.path.join(pkg_share, f"rviz/{rviz_name}")

    # Launch Argument: the path to the rviz config file
    rvizconfig_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=rviz_config_path,
        description="Absolute path to rviz config file",
    )

    # Declare robot_type argument
    robot_type_arg = DeclareLaunchArgument(
        "robot_type",
        default_value="C5A",
        description="robot_type, e.g. C5A",
    )

    # Read robot_type at runtime
    urdf_filename = PythonExpression(
        ["'GBT_", LaunchConfiguration("robot_type"), ".urdf'"]
    )

    # Join into a full path
    urdf_model_path = PathJoinSubstitution(
        [
            pkg_share,
            "urdf",
            urdf_filename,
        ]
    )
    ld.add_action(robot_type_arg)
    ld.add_action(LogInfo(msg=[urdf_model_path]))
    # Create robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_model_path],
    )

    # Create joint_state_publisher_gui node
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        arguments=[urdf_model_path],
    )

    # Create rviz2 node
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{"use_sim_time": False}],
    )

    # Add actions to launch description
    ld.add_action(rvizconfig_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld
