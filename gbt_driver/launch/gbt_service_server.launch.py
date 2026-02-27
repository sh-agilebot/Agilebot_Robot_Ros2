"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:

This file is used to start the service server node in the gbt_driver package, for specific services, please refer to: service_server.py
Start nodes: service_server, robot_bridge, rviz2, robot_state_publisher
"""


import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

def generate_launch_description():
    ld = LaunchDescription()

    # Define package name, URDF filename, and RViz config filename
    package_name = "gbt_description"
    rviz_name = "display.rviz"

    # Get the path to the package
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Construct the full path to the RViz config file
    rviz_config_path = os.path.join(pkg_share, f"rviz/{rviz_name}")

    # Declare robot_type argument
    robot_type_arg = DeclareLaunchArgument(
        "robot_type",
        default_value="C5A",
        description="Robot Type, e.g. C5A",
    )
    # Read robot_type at runtime
    urdf_filename = PythonExpression(
        ["'GBT_", LaunchConfiguration("robot_type"), ".urdf'"]
    )

    # Combine into a full path: .../urdf/GBT_C5A.urdf
    urdf_model_path = PathJoinSubstitution(
        [
            pkg_share,
            "urdf",
            urdf_filename,
        ]
    )
    ld.add_action(robot_type_arg)
    # Print the URDF file path
    ld.add_action(LogInfo(msg=urdf_model_path))

    # Create robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_model_path],
    )

    # Launch Argument: Path to RViz config file
    rvizconfig_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=rviz_config_path,
        description="Absolute path to rviz config file",
    )
    enable_rviz_arg = DeclareLaunchArgument(
        name="enable_rviz",
        default_value="true",
        description="Whether to launch rviz2",
    )

    # Create rviz2 node
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{"use_sim_time": False}],
    )

    # Create robot_bridge node (for syncing robot state with RViz)
    robot_bridge_node = Node(
        package="gbt_driver",
        executable="robot_bridge",
        name="robot_bridge",
        output="screen",
        parameters=[{"robot_type": LaunchConfiguration("robot_type")}],
    )

    # Create a node named service_server
    # Declare 2 arguments: 'UF_ID': 0, 'TF_ID': 0
    ld.add_action(
        DeclareLaunchArgument("UF_ID", default_value="0", description="User Frame ID")
    )
    ld.add_action(
        DeclareLaunchArgument("TF_ID", default_value="0", description="Tool Frame ID")
    )
    service_server_node = Node(
        package="gbt_driver",
        executable="service_server",
        name="service_server",
        output="screen",
        parameters=[
            {"UF_ID": LaunchConfiguration("UF_ID")},
            {"TF_ID": LaunchConfiguration("TF_ID")},
        ],
    )

    # Add nodes to LaunchDescription
    ld.add_action(rvizconfig_arg)
    ld.add_action(enable_rviz_arg)
    ld.add_action(rviz2_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(robot_bridge_node)
    ld.add_action(service_server_node)

    return ld
