"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.

Instruction:

Launches the stacking demo, including service_server, robot_stacking_node, and AgileGaze_node.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Package and file names
    pkg_name_desc = "gbt_description"
    rviz_config_file = "display.rviz"

    # Locate package share directories
    pkg_desc_share = FindPackageShare(package=pkg_name_desc).find(pkg_name_desc)
    pkg_stack_share = FindPackageShare(package="gbt_stacking").find("gbt_stacking")

    # Declare common launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            "robot_type", default_value="C5A", description="Robot Type, e.g. C5A"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=os.path.join(pkg_stack_share, "rviz", rviz_config_file),
            description="Absolute path to RViz configuration file",
        )
    )
    ld.add_action(
        DeclareLaunchArgument("UF_ID", default_value="0", description="User frame ID")
    )
    ld.add_action(
        DeclareLaunchArgument("TF_ID", default_value="0", description="Tool frame ID")
    )
    ld.add_action(
        DeclareLaunchArgument(
            "fake",
            default_value="False",
            description="Use fake AgileGaze data (True/False)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "host", default_value="172.17.24.70", description="AgileGaze host IP"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "port", default_value="5622", description="AgileGaze port"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "cmd", default_value="RUN_FIND, a1", description="AgileGaze command"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "publish_interval",
            default_value="0.1",
            description="AgileGaze data publish interval (seconds)",
        )
    )

    # Construct URDF file path dynamically based on robot_type
    urdf_filename = PythonExpression(
        ["'GBT_", LaunchConfiguration("robot_type"), ".urdf'"]
    )
    urdf_path = PathJoinSubstitution([pkg_desc_share, "urdf", urdf_filename])

    # Node: robot_state_publisher publishes TF frames based on URDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        arguments=[urdf_path],
    )

    # Node: RViz for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        parameters=[{"use_sim_time": False}],
    )

    # Node: robot_bridge connects to robot hardware/driver
    robot_bridge = Node(
        package="gbt_driver",
        executable="robot_bridge",
        name="robot_bridge",
        output="screen",
        parameters=[
            {"robot_type": LaunchConfiguration("robot_type")},
        ],
    )

    # Node: service_server provides motion services
    service_server = Node(
        package="gbt_driver",
        executable="service_server",
        name="service_server",
        output="screen",
        parameters=[
            {"UF_ID": LaunchConfiguration("UF_ID")},
            {"TF_ID": LaunchConfiguration("TF_ID")},
        ],
    )

    # Log the 'fake' flag value for debugging
    fake_flag = LaunchConfiguration("fake")
    ld.add_action(LogInfo(msg=["Fake vision flag: ", fake_flag]))

    # Node: AgileGaze for camera data (real or fake)
    AgileGaze = Node(
        package="gbt_vision",
        executable="gbt_agilegaze",
        name="gbt_agilegaze",
        output="screen",
        parameters=[
            {"host": LaunchConfiguration("host")},
            {"port": LaunchConfiguration("port")},
            {"cmd": LaunchConfiguration("cmd")},
            {"publish_interval": LaunchConfiguration("publish_interval")},
            {"srv_name": "/gbt_vision/service/AgileGaze"},
            {"fake": ParameterValue(fake_flag, value_type=bool)},
        ],
    )
    # Conditional log when using fake data
    ld.add_action(
        LogInfo(
            condition=IfCondition(PythonExpression([fake_flag, " == 'True'"])),
            msg="Using fake vision data",
        )
    )

    # Node: robot stacking logic
    stacking_params = PathJoinSubstitution(
        [pkg_stack_share, "config", "stacking_params.yaml"]
    )
    robot_stacking = Node(
        package="gbt_stacking",
        executable="robot_stacking",
        name="robot_stacking",
        output="screen",
        parameters=[stacking_params, {"robot_type": LaunchConfiguration("robot_type")}],
    )

    # Node: stacking_visualizer for pallet visualization (delayed start)
    pallet_viz_params = PathJoinSubstitution(
        [pkg_stack_share, "config", "pallet_viz_params.yaml"]
    )
    visualizer = Node(
        package="gbt_stacking",
        executable="stacking_visualizer",
        name="stacking_visualizer",
        output="screen",
        parameters=[pallet_viz_params],
    )

    # Add all nodes to the launch description
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz_node)
    ld.add_action(robot_bridge)
    ld.add_action(service_server)
    ld.add_action(AgileGaze)
    ld.add_action(robot_stacking)
    ld.add_action(visualizer)

    return ld
