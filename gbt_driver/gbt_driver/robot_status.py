"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
Subscribe to various states of the physical robot and publish to topic /gbt_driver/feedback_states,
for specific content, refer to gbt_interface/msg/FeedbackState.msg
"""

import os
from typing import List, Tuple

import rclpy
import yaml
from Agilebot.IR.A.arm import Arm
from Agilebot.IR.A.common.const import const
from Agilebot.IR.A.sdk_types import ParamType, SignalType
from Agilebot.IR.A.status_code import StatusCodeEnum
from ament_index_python.packages import get_package_share_directory
from common.enum.arm_soft_mode_enum import SoftModeEnum
from common.enum.controller_status_enum import ControllerStatusEnum
from common.enum.robot_status_enum import RobotStatusEnum
from common.enum.servo_status_enum import ServoStatusEnum
from common.utils.arm_utils import get_current_pose, safe_call
from gbt_interface.msg import (
    AlarmCode,
    ArmSoftModeStatus,
    ControllerStatus,
    FeedbackState,
    IOType,
    RobotStatus,
    ServoStatus,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RobotStatusNode(Node):
    def __init__(self):
        super().__init__("robot_status")
        self.get_logger().info("robot_status node is starting...")

        # Define parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("publish_interval", 0.1),  # Publish interval
            ],
        )
        # Get parameters
        self.publish_interval = self.get_parameter("publish_interval").value
        self.get_logger().info(f"Publish interval: {self.publish_interval}")

        # Initialize robot connection and settings
        # Read robot IP address from config/robot_config.yaml file
        config_path = os.path.join(
            get_package_share_directory("gbt_driver"), "config/robot_config.yaml"
        )
        with open(config_path, "r", encoding="utf-8") as f:
            self.robot_config = yaml.safe_load(f)
            self.robot_ip_address = self.robot_config[
                "robot_ip_address"
            ]  # Robot IP address
        self.arm = Arm()  # Create robot arm object

        # Attempt to connect to robot
        safe_call(self.arm.connect, self.robot_ip_address,prefix=f"connect to robot at {self.robot_ip_address}")

        self.uf_id, self.tf_id = self.get_UF_TF()
        self.get_logger().info(f"Activated UF ID: {self.uf_id}, TF ID: {self.tf_id}")

        # Create publisher to read and publish robot status
        self.feedback_State_publisher = self.create_publisher(
            FeedbackState, "/gbt_driver/feedback_states", 10  # Queue size
        )

        self.joint_state_publisher = self.create_publisher(
            JointState,
            "/joint_states",  # Topic for publishing joint states
            10,  # Queue size
        )

        # Set timer to periodically publish joint states
        self.create_timer(self.publish_interval, self.publish_feedback_states)
        self.get_logger().info("robot_status node is started")
        self.get_logger().info(f"Connected to robot at {self.robot_ip_address}...")

    def __del__(self):
        if self.arm:
            self.arm.disconnect()
        self.get_logger().info("Robot disconnected")

    def publish_feedback_states(self):
        """
        Publish robot feedback states. For content details, refer to gbt_interface/msg/FeedbackState.msg
        """
        try:

            self.uf_id, self.tf_id = self.get_UF_TF()
            # Create feedback status message
            feedback_status = FeedbackState()
            feedback_status.header.stamp = self.get_clock().now().to_msg()
            feedback_status.is_connected = self.arm.is_connect()
            feedback_status.robot_type = self.get_robot_model_info()

            # Get robot status
            feedback_status.robot_status = self.get_robot_state()

            # Get controller status
            feedback_status.controller_status = self.get_controller_state()

            # Get servo controller status
            feedback_status.servo_status = self.get_servo_state()

            # Get arm soft mode status
            feedback_status.arm_soft_mode_status = self.get_arm_soft_mode_status()

            # Get current joint states
            pose, ret_code = get_current_pose(
                self.arm, self.get_clock().now().to_msg(), const.JOINT
            )
            if ret_code == StatusCodeEnum.OK:
                feedback_status.joint_states = pose
            else:
                self.get_logger().error(
                    f"Failed to get joint states, reason: {ret_code.name}"
                )

            # Get flange center pose
            pose, ret_code = get_current_pose(
                self.arm, self.get_clock().now().to_msg(), const.CART, self.uf_id, 0
            )
            if ret_code == StatusCodeEnum.OK:
                feedback_status.flange_pose = pose
            else:
                self.get_logger().error(
                    f"Failed to get flange pose, reason: {ret_code.name}"
                )

            # Get tool pose
            pose, ret_code = get_current_pose(
                self.arm,
                self.get_clock().now().to_msg(),
                const.CART,
                self.uf_id,
                self.tf_id,
            )
            if ret_code == StatusCodeEnum.OK:
                feedback_status.tool_pose = pose
            else:
                self.get_logger().error(
                    f"Failed to get tool pose, reason: {ret_code.name}"
                )

            # Alarm codes
            feedback_status.alarm_code_list[:] = self.get_alarm_code_list()

            # TODO Publish predefined IO: Not recommended as batch get is not supported
            # feedback_status.io = self.get_io_list()

            # Publish global speed ratio
            feedback_status.overall_speed = self.get_overall_speed()

            # coordinates
            feedback_status.actived_uf_id= self.uf_id
            feedback_status.actived_tf_id = self.tf_id

            # Publish robot feedback status to topic
            self.feedback_State_publisher.publish(feedback_status)
            # Publish joint states
            self.joint_state_publisher.publish(feedback_status.joint_states)
            self.get_logger().debug(
                f"Joint states: {feedback_status.joint_states.position}"
            )

        except Exception as e:
            self.get_logger().error(f"Error publishing robot status: {e}")

    def get_robot_model_info(self) -> str:
        """
        Get robot information

        Returns:
            str: Robot model information
        """
        # Get robot model information
        model_info,_=safe_call(self.arm.get_arm_model_info,prefix="get remote arm model info")
        # Return robot model information
        return model_info

    def get_robot_state(self) -> RobotStatus:
        """Get robot status

        Returns:
            RobotStatus: Robot status message
        """
        state, _=safe_call(self.arm.get_robot_status,prefix="get robot status")
        robot_state = RobotStatus()
        robot_state.status_code = state.code
        robot_state.status_description = RobotStatusEnum.from_id(state.code).errmsg_en

        return robot_state

    # TODO: Get top error
    def get_top_error(self) -> int:
        pass

    def get_overall_speed(self) -> float:
        """Get robot Overall speed (%)

        Returns:
            float: Robot Overall speed (%)
        """
        speed,_=safe_call(self.arm.motion.get_param,param_name=ParamType.OVC,prefix="get overall speed")
        return speed

    def get_controller_state(self) -> ControllerStatus:
        """Get controller status

        Returns:
            ControllerStatus: Controller status message
        """
        controller_state = ControllerStatus()
        state,_=safe_call(self.arm.get_ctrl_status,prefix="get controller status")
        controller_state.status_code = state.code
        # controller_state.status_description = state.msg
        controller_state.status_description = ControllerStatusEnum.from_id(
            state.code
        ).errmsg_en
        return controller_state

    def get_UF_TF(self) -> Tuple[int, int]:
        """
        Get UF ID and TF ID

        Returns:
            Tuple[int, int]: UF ID and TF ID
        """
        _UF_ID, ret_code = self.arm.motion.get_UF()
        if ret_code != StatusCodeEnum.OK:
            self.get_logger().error(f"Failed to get UF ID, {ret_code.name}")
            return -1, -1
        _TF_ID, ret_code = self.arm.motion.get_TF()
        if ret_code != StatusCodeEnum.OK:
            self.get_logger().error(f"Failed to get TF ID, {ret_code.name}")
            return -1, -1
        return _UF_ID, _TF_ID

    # TODO Current IO doesn't support batch get, not recommended
    def get_io_list(self) -> List[IOType]:
        """
        Read IO configuration from file and publish IO status

        Returns:
            List[IOType]: List of IO status messages
        """
        io_list = []
        for io in self.robot_config["io_config"]:
            msg = IOType()
            msg.io_type = getattr(
                IOType, io["type"]
            )  # Convert type name to msg enum value
            msg.ports = io["ports"]
            # Convert io_type to corresponding SignalType
            _io_type = SignalType(msg.io_type)

            if _io_type == IOType.DO:
                _values, ret_code = self.arm.signals.multi_read(
                    SignalType.DO, msg.ports
                )
                if ret_code == StatusCodeEnum.OK:
                    msg.values = _values
                    self.get_logger().debug(
                        f"Read IO {io['type']} ports: {msg.ports} Values: {msg.values}"
                    )
                else:
                    self.get_logger().error(f"Failed to read IO ports: {ret_code.name}")
            else:
                # Warning: This is a workaround for batch get not supported,will read each port individually.
                # This may cause performance issues
                self.get_logger().warn(
                    f"Only DO supports batch reading. Other IO ports are accessed individually, potentially reducing overall IO efficiency."
                )
                for port in io["ports"]:
                    value, ret_code = self.arm.signals.read(_io_type, port)
                    if ret_code == StatusCodeEnum.OK:
                        msg.values.append(value)
                        self.get_logger().debug(
                            f"Read IO {io['type']} port {port}: {value}"
                        )
                    else:
                        self.get_logger().error(
                            f"Failed to read IO port {port}: {ret_code.name}"
                        )

            # Optional: Add description information
            # msg.descriptions = [f"Port {port}" for port in io["ports"]]

            self.get_logger().debug(
                f"Published IO Type: {io['type']}, Ports: {msg.ports} Values: {msg.values}"
            )
            io_list.append(msg)
        return io_list

    def get_servo_state(self) -> ServoStatus:
        """Get servo controller status

        Returns:
            ServoStatus: Servo controller status message
        """
        state,_=safe_call(self.arm.get_servo_status,prefix="get servo status")

        servo_state = ServoStatus()
        servo_state.status_code = state.code
        # servo_state.status_description = state.msg
        servo_state.status_description = ServoStatusEnum.from_id(state.code).errmsg_en

        return servo_state

    def get_arm_soft_mode_status(self) -> ArmSoftModeStatus:
        """Get arm soft mode status

        Returns:
            ArmSoftModeStatus: Arm soft mode status message
        """
        result,_=safe_call(self.arm.get_op_mode,prefix="get op mode")

        arm_soft_mode_status = ArmSoftModeStatus()
        # Style not consistent with above because SDK enum values are int type
        arm_soft_mode_status.status_code = result
        arm_soft_mode_status.status_description = SoftModeEnum.from_id(result).errmsg_en
        return arm_soft_mode_status

    def get_alarm_code_list(self) -> List[AlarmCode]:
        """Get alarm code list

        Returns:
            List[AlarmCode]: List of alarm code messages
        """
        alarm_code_list,_=safe_call(self.arm.alarm.get_all_active_alarms,prefix="get alarm code list")

        alarm_code_list_msg = []
        for item in alarm_code_list:
            alarm_code = AlarmCode()
            alarm_code.user_code = item.UserCode
            alarm_code.inner_code = item.InnerCode
            alarm_code.name = item.Name
            alarm_code.reason = item.Reason[0]
            alarm_code.suggest = item.Suggest[0]
            alarm_code.consequence = item.Consequence[0]
            alarm_code.ext_desc = item.Description[0]

            alarm_code_list_msg.append(alarm_code)

        return alarm_code_list_msg


def main(args=None):
    """
    Main function to initialize ROS2 node and start robot bridge

    :param args: Command line arguments
    """
    rclpy.init(args=args)
    robot_bridge = RobotStatusNode()
    try:
        # Continuously run ROS2 node
        rclpy.spin(robot_bridge)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C interrupt
    finally:
        # Shutdown node and clean up resources
        robot_bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
