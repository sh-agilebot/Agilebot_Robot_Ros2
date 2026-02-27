"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
This launch file is used to start the robot offline trajectory action server and start rviz2 to display the robot's real-time state
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
    # Define package name, urdf file name, and rviz configuration file name
    package_name = "gbt_description"
    rviz_name = "display.rviz"

    ld = LaunchDescription()

    # Find package path
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Construct rviz configuration file path
    rviz_config_path = os.path.join(pkg_share, f"rviz/{rviz_name}")

    robot_type_arg = DeclareLaunchArgument(
        "robot_type",
        default_value="C5A",
        description="Robot Type, e.g. C5A",
    )
    # Read robot_type at runtime
    urdf_filename = PythonExpression(
        ["'GBT_", LaunchConfiguration("robot_type"), ".urdf'"]
    )

    # Join into a full path: .../urdf/GBT_C5A.urdf
    urdf_model_path = PathJoinSubstitution(
        [
            pkg_share,
            "urdf",
            urdf_filename,
        ]
    )

    ld.add_action(robot_type_arg)
    # Print URDF file path
    ld.add_action(LogInfo(msg=urdf_model_path))

    # Create robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_model_path],
    )

    # Launch Argument: RViz configuration file path
    rvizconfig_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=rviz_config_path,
        description="Absolute path to rviz config file",
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

    # Create action_server node: Accepts MoveIt trajectories and synchronizes with physical robot
    trajectory_server_node = Node(
        package="gbt_driver",
        executable="trajectory_server",
        name="trajectory_server",
        output="screen",
    )

    # Create robot_bridge node (used to synchronize robot with RViz)
    robot_bridge_node = Node(
        package="gbt_driver",
        executable="robot_bridge",
        name="robot_bridge",
        output="screen",
        parameters=[{"robot_type": LaunchConfiguration("robot_type")}],
    )

    # Add Launch Argument
    ld.add_action(rvizconfig_arg)

    # Add robot_state_publisher node
    ld.add_action(robot_state_publisher_node)

    # Add rviz2 node
    ld.add_action(rviz2_node)

    # Add robot_bridge node
    ld.add_action(robot_bridge_node)
    # Add action_server node
    ld.add_action(trajectory_server_node)

    return ld
