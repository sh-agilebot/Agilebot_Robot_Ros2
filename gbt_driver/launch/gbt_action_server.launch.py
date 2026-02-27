"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:

This file launches the GBT robot action server node.
The node demonstrates MoveIt-based planning and control, and sends the resulting trajectory to the physical robot.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)

# event handlers
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package name and rviz config file name
    package_name = "gbt_description"
    rviz_name = "display.rviz"

    ld = LaunchDescription()

    # Find package share path
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # Construct RViz config absolute path
    rviz_config_path = os.path.join(pkg_share, f"rviz/{rviz_name}")

    # Launch arguments
    robot_type_arg = DeclareLaunchArgument(
        "robot_type",
        default_value="C5A",
        description="Robot Type, e.g. C5A",
    )

    interpolate_mode_arg = DeclareLaunchArgument(
        "interpolate_mode",
        default_value="quintic",
        description="Interpolation mode, e.g. quintic or spline",
    )

    # Build URDF filename from robot_type at runtime
    urdf_filename = PythonExpression(
        ["'GBT_", LaunchConfiguration("robot_type"), ".urdf'"]
    )

    # Full URDF path substitution
    urdf_model_path = PathJoinSubstitution([pkg_share, "urdf", urdf_filename])

    # Create robot_state_publisher node
    robot_stagte_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_model_path],
    )

    # Add launch arguments to LD
    ld.add_action(robot_type_arg)
    ld.add_action(interpolate_mode_arg)

    # Print the URDF model path (English message)
    ld.add_action(LogInfo(msg=["URDF model path: ", urdf_model_path]))

    # --- Primary node: robot_bridge (treat as the "primary" process) ---
    robot_bridge_node = Node(
        package="gbt_driver",
        executable="robot_bridge",
        name="robot_bridge",
        output="screen",
        parameters=[{"robot_type": LaunchConfiguration("robot_type")}],
    )

    # RViz config arg
    rvizconfig_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=rviz_config_path,
        description="Absolute path to rviz config file",
    )

    # Other nodes (these will start normally)
    action_server_node = Node(
        package="gbt_driver",
        executable="moveit_action_server",
        name="moveit_action_server",
        output="screen",
        parameters=[
            {"robot_type": LaunchConfiguration("robot_type")},
            {"interpolate_mode": LaunchConfiguration("interpolate_mode")},
        ],
    )

    # Prepare the MoveIt include launch, but DO NOT add it to LD directly.
    moveit_config_package = PythonExpression(
        ["'", LaunchConfiguration("robot_type"), "_moveit_config'.lower()"]
    )
    moveit_config_path = FindPackageShare(moveit_config_package)
    moveit_config_file = PathJoinSubstitution(
        [moveit_config_path, "launch", "moveit_physical_robot.launch.py"]
    )
    moveit_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_config_file),
    )

    # -----------------------
    # Strategy:
    # 1) When primary node starts, schedule a short TimerAction (delay).
    # 2) TimerAction's actions will include MoveIt (so inclusion happens only after the delay).
    # 3) If primary node exits before the delay completes, emit Shutdown() to cancel the rest.
    # -----------------------

    # How long to wait to consider the primary as "still alive" after start.
    # You can tune this value (in seconds). 0.5 - 1.0 is common.
    START_STABILITY_DELAY = 1.0

    # OnProcessStart: schedule a delayed include of moveit (only after primary seems stable)
    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=robot_bridge_node,
                on_start=[
                    LogInfo(
                        msg="Primary node (robot_bridge) started, scheduling MoveIt inclusion after delay..."
                    ),
                    TimerAction(
                        period=START_STABILITY_DELAY,
                        actions=[
                            LogInfo(msg="Delay passed; including MoveIt launch now."),
                            moveit_config_launch,
                        ],
                    ),
                ],
            )
        )
    )

    # OnProcessExit: if primary node exits at any time, shut down the whole launch.
    # This prevents the delayed TimerAction from including MoveIt if the primary died quickly.
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=robot_bridge_node,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    )

    # Add remaining actions to the launch description
    ld.add_action(rvizconfig_arg)
    ld.add_action(robot_bridge_node)
    ld.add_action(action_server_node)
    ld.add_action(robot_stagte_publisher_node)

    # Note: moveit_config_launch is NOT directly added to LD here (it is added by the TimerAction above).

    return ld
