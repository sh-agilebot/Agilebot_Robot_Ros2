"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
Start the gbt_AgileGaze node, receive the image processing results from AgileGaze and publish them to the topic /gbt_vision/AgileGaze
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    declare_host_arg = DeclareLaunchArgument(
        "host", default_value="172.17.26.57", description="AgileGaze host address"
    )
    declare_port_arg = DeclareLaunchArgument(
        "port", default_value="5622", description="AgileGaze port"
    )
    declare_cmd_arg = DeclareLaunchArgument(
        "cmd", default_value="RUN_FIND, a1\n", description="AgileGaze run-find command"
    )
    declare_publish_interval_arg = DeclareLaunchArgument(
        "publish_interval",
        default_value="0.1",
        description="Publishing interval for vision results",
    )
    declare_fake_arg = DeclareLaunchArgument(
        "fake",
        default_value="False",
        description="Whether to use fake AgileGaze data (True/False)",
    )

    # Substitutions
    host_lc = LaunchConfiguration("host")
    port_lc = LaunchConfiguration("port")
    cmd_lc = LaunchConfiguration("cmd")
    publish_interval_lc = LaunchConfiguration("publish_interval")
    fake_lc = LaunchConfiguration("fake")

    # Node definition
    AgileGaze_node = Node(
        package="gbt_vision",
        executable="gbt_agilegaze",
        name="gbt_agilegaze",
        output="screen",
        parameters=[
            {"host": host_lc},
            {"port": port_lc},
            {"cmd": cmd_lc},
            {"publish_interval": publish_interval_lc},
            {"fake": ParameterValue(fake_lc, value_type=bool)},
        ],
    )

    # Conditional logging when using fake data
    log_fake = LogInfo(
        msg="[gbt_AgileGaze] Running in FAKE mode (using preset data)",
        condition=IfCondition(fake_lc),
    )

    # Build LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_host_arg)
    ld.add_action(declare_port_arg)
    ld.add_action(declare_cmd_arg)
    ld.add_action(declare_publish_interval_arg)
    ld.add_action(declare_fake_arg)
    ld.add_action(log_fake)
    ld.add_action(AgileGaze_node)

    return ld
