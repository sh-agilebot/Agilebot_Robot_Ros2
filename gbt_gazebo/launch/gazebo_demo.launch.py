"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:

Gazebo demo launch file, load robot_description and start gazebo
"""

import os

import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.logging import get_logger
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Get Robot Type from arguments
    robot_type = LaunchConfiguration("robot_type").perform(context)
    if not robot_type:
        raise RuntimeError("robot_type is required but not provided")

    controller_name = context.launch_configurations.get("controller_name")
    if (
        controller_name is None or controller_name.strip() == ""
    ):  # if not specified,use default name
        controller_name = f"gbt_{robot_type.lower()}_arm_controller"

    _logger = get_logger(f"gazebo_{robot_type} launch")
    _logger.info(f"Launching robot: {robot_type}")
    _logger.info(f"controller: {controller_name}")

    # Check if controller name includes robot type
    # if you change the controller name, please comment out the following code
    expected_controller = f"gbt_{robot_type.lower()}_arm_controller"
    if robot_type.lower() not in controller_name.lower():
        _logger.error(
            f"Invalid controller name '{controller_name}': it must include the robot type '{robot_type}'. Example format: '{expected_controller}'."
        )
        raise ValueError(
            f"Controller name '{controller_name}' is invalid. "
            f"It must include '{robot_type}', e.g. '{expected_controller}'."
        )

    # Concatenate URDF file path
    pkg_share = FindPackageShare(package="gbt_gazebo").find("gbt_gazebo")
    urdf_file = PathJoinSubstitution(
        [pkg_share, "config", f"gazebo_{robot_type}_description.urdf.xacro"]
    )
    resolved_urdf_path = urdf_file.perform(context)
    _logger.info(f"Resolved URDF path: {resolved_urdf_path}")

    # Parse URDF file
    doc = xacro.parse(open(resolved_urdf_path))
    xacro.process_doc(doc)
    params = {"robot_description": doc.toxml()}

    # Start Gazebo simulation
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    # Start robot_state_publisher node
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}, params, {"publish_frequency": 15.0}],
        output="screen",
    )

    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", robot_type],
        output="screen",
    )

    # Load joint_state_controller
    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    # Load controller
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            controller_name,
        ],
        output="screen",
    )

    # Register event handler: Start load_joint_state_controller after spawn_entity exits
    close_evt1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )

    # Register event handler: Start load_joint_trajectory_controller after load_joint_state_controller exits
    close_evt2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_trajectory_controller],
        )
    )

    return [gazebo, node_robot_state_publisher, spawn_entity, close_evt1, close_evt2]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_type",
                default_value="C5A",
                description="Robot Type.Optional: C5A, C7A, C12A, C16A.Default: C5A",
            ),
            DeclareLaunchArgument(
                "controller_name",
                default_value="",
                description="moveit Controller name.Example: gbt_c5a_arm_controller",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
