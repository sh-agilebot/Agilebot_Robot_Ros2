"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
Subscribe to the joint states of the physical robot and publish them to the ROS2 joint_states topic for RViz display.
"""

import os
import sys

import rclpy
import yaml
from Agilebot.IR.A.arm import Arm
from Agilebot.IR.A.common.const import const
from Agilebot.IR.A.status_code import StatusCodeEnum
from ament_index_python.packages import get_package_share_directory
from common.utils.arm_utils import get_current_pose, safe_call
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RobotBridge(Node):
    def __init__(self, verbose=False):
        super().__init__("robot_bridge")
        self.verbose = verbose

        params = [
            ("publish_interval", 0.1),
            ("robot_type", "C5A")
        ]
        self.declare_parameters(
            namespace="",
            parameters=params,
        )
        # print params
        for param in params:
            self.get_logger().info(
                f"param: {param[0]} = {self.get_parameter(param[0]).value}"
            )

        # Read parameters
        self.publish_interval = self.get_parameter("publish_interval").value
        self.robot_type = self.get_parameter("robot_type").value

        # Initialize robot connection and settings
        # Read robot IP address from config/robot_config.yaml file
        config_path = os.path.join(
            get_package_share_directory("gbt_driver"), "config/robot_config.yaml"
        )
        with open(config_path, "r", encoding="utf-8") as f:
            robot_config = yaml.safe_load(f)
            self.robot_ip_address = robot_config["robot_ip_address"]  # Robot IP address

        self.arm = Arm()  # Create robot arm object

        # Try to connect to robot
        safe_call(self.arm.connect, self.robot_ip_address, prefix="connect to robot")

        remote_robot_type, _ = safe_call(
            self.arm.get_arm_model_info, prefix="get arm model info"
        )
        if not remote_robot_type.endswith(self.robot_type):
            self.get_logger().error(
                f"expected robot type: {self.robot_type}, actual robot type: {remote_robot_type}"
            )
            sys.exit(1)

        # Create publisher to broadcast joint states to RViz
        self.joint_state_publisher = self.create_publisher(
            JointState,
            "/joint_states",  # Publish joint states to this topic
            10,  # Queue size
        )

        # Set timer to periodically publish joint states
        self.create_timer(
            self.publish_interval, self.publish_joint_states
        )  # Publish every 0.1 seconds
        self.get_logger().info("Robot bridge started")
        self.get_logger().info(f"Connected to robot at {self.robot_ip_address}")

    def publish_joint_states(self):
        """
        Publish current robot joint states to RViz.
        """
        try:
            # Get current joint states
            joint_states, _ = safe_call(
                get_current_pose,
                self.arm,
                self.get_clock().now().to_msg(),
                const.JOINT,
                prefix="get joint states",
            )

            # Publish joint states to /joint_states topic
            self.joint_state_publisher.publish(joint_states)
            if self.verbose:
                self.get_logger().info(
                    f"Published joint states: {joint_states.position}"
                )
        except Exception as e:
            self.get_logger().error(
                f"Error occurred while publishing joint states: {e}"
            )

    def __del__(self):
        if self.arm:
            self.arm.disconnect()
        self.get_logger().info("Robot disconnected")


def main(args=None):
    """
    Main function to initialize ROS2 node and start robot bridge.

    :param args: Command line arguments
    """
    rclpy.init(args=args)
    robot_bridge = RobotBridge()

    # Keep running ROS2 node
    rclpy.spin(robot_bridge)

    # Shutdown node and clean up resources
    robot_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
