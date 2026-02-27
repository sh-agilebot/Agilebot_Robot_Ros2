"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
This launch file starts the `robot_status` node, which continuously publishes the robot’s current state on the `/gbt_driver/feedback_states` topic.
Published data include the robot’s pose, tool pose, and other status information defined in the `gbt_interface/msg/FeedbackState.msg` message.
Users can subscribe to `/gbt_driver/feedback_states` to monitor real-time feedback from the robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # declare launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            "publish_interval", default_value="0.1", description="Publish interval"
        )
    )

    # create robot_status Node
    robot_status_node = Node(
        package="gbt_driver",
        executable="robot_status",
        name="robot_status",
        output="screen",
        parameters=[
            {
                "publish_interval": LaunchConfiguration("publish_interval"),
            }
        ],
    )

    ld.add_action(robot_status_node)

    return ld
