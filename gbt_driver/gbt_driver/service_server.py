"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:

Service server contains the following services:
1. Set io
2. Move to pose
3. Program control
4. Servo controller power on/off
5. Send script
6. emergency stop
"""

import os
import time

import rclpy
import yaml
from Agilebot.IR.A import execution, signals
from Agilebot.IR.A.arm import Arm, ServoStatusEnum
from Agilebot.IR.A.common.const import const
from Agilebot.IR.A.sdk_types import SignalType
from Agilebot.IR.A.status_code import StatusCodeEnum
from ament_index_python.packages import get_package_share_directory
from common.utils.arm_utils import (
    check_robot_status,
    get_current_pose,
    move_to_pose_cart_without_posture,
    pose_to_xyz_rpy_deg,
    safe_call,
)
from gbt_interface.action import MoveToPose
from gbt_interface.srv import IO, LED, EmergencyStop, ProgramControl, SendScript, Servo
from rclpy.action import ActionServer
from rclpy.node import Node


class ServiceServer(Node):
    def __init__(self):
        super().__init__("service_server")

        # Parameter initialization
        self.parameters = {
            "UF_ID": 0,  # User coordinate frame ID
            "TF_ID": 0,  # tool coordinate frame ID
            "io_server_name": "/gbt_driver/service_server/io",
            "move_action_server_name": "/gbt_driver/move_to_pose",
            "program_control_server_name": "/gbt_driver/service_server/program_control",
            "servo_power_server_name": "/gbt_driver/service_server/servo_power",
            "send_script_server_name": "/gbt_driver/service_server/send_script",
            "emergency_stop_server_name": "/gbt_driver/service_server/emergency_stop",
            "led_server_name": "/gbt_driver/service_server/led",
        }

        # Declare parameters in ROS2 parameter server
        self.declare_parameters(
            namespace="",
            parameters=[
                (param_name, default_value)
                for param_name, default_value in self.parameters.items()
            ],
        )

        # print parameters
        for param in self.get_parameters(self.parameters.keys()):
            self.get_logger().info(f"{param.name}: {param.value}")

        self.uf_id = self.get_parameter("UF_ID").value  # user frame id
        self.tf_id = self.get_parameter("TF_ID").value  # tool frame id

        # Initialize robot connection
        self.robot_ip_address = self.load_robot_config()
        self.arm = Arm()

        # Try to connect to the robot
        if not self.connect_to_robot():
            return
        self.get_logger().info(f"Connected to robot at {self.robot_ip_address}")

        # Get WebServer for SDK 2.0 API compatibility
        # Note: _Arm__web_server is the internal WebServer object used by Signals/Execution
        self._web_server = self.arm._Arm__web_server

        # Create Signals and Execution objects for SDK 2.0
        self.signals = signals.Signals(self._web_server)
        self.execution = execution.Execution(self._web_server)

        # Create services and action servers
        self.create_services_and_action_servers()

    def load_robot_config(self):
        """
        Load robot configuration (IP address) from the YAML file.

        Returns:
            str: Robot IP address
        """
        config_path = os.path.join(
            get_package_share_directory("gbt_driver"), "config/robot_config.yaml"
        )
        with open(config_path, "r", encoding="utf-8") as f:
            robot_config = yaml.safe_load(f)
        return robot_config["robot_ip_address"]

    def connect_to_robot(self):
        """
        Try to establish a connection with the robot.

        Returns:
            bool: True if connection is successful, False otherwise
        """
        safe_call(
            self.arm.connect,
            self.robot_ip_address,
            prefix=f"connect to robot at {self.robot_ip_address}",
        )
        return True

    def create_services_and_action_servers(self):
        """
        Create all necessary services and action servers for the robot control.

        Services:
            - IO Service
            - Program Control Service
            - Servo Control Service
            - Send Script Service
        """
        # Create IO service
        self.create_service(IO, self.parameters["io_server_name"], self.io_callback)
        self.get_logger().info("IO Service Node is on.")

        # Create MoveToPose action server
        self.action_server = ActionServer(
            self,
            MoveToPose,
            self.parameters["move_action_server_name"],
            self.move_action_server_execute_callback,
        )
        self.get_logger().info("MoveToPose Action Server Node is on.")

        # Create ProgramControl service
        self.create_service(
            ProgramControl,
            self.parameters["program_control_server_name"],
            self.program_control_callback,
        )
        self.get_logger().info("ProgramControl Service Node is on.")

        # Create Servo power service
        self.create_service(
            Servo,
            self.parameters["servo_power_server_name"],
            self.servo_on_off_callback,
        )
        self.get_logger().info("Servo Power Service Node is on.")

        # Create SendScript service
        self.create_service(
            SendScript,
            self.parameters["send_script_server_name"],
            self.send_script_callback,
        )
        self.get_logger().info("SendScript Service Node is on.")

        # Create EmergencyStop service
        self.create_service(
            EmergencyStop,
            self.parameters["emergency_stop_server_name"],
            self.emergency_stop_callback,
        )
        self.get_logger().info("EmergencyStop Service Node is on.")

        # Create LED service
        self.create_service(
            LED,
            self.parameters["led_server_name"],
            self.led_callback,
        )
        self.get_logger().info("LED Service Node is on.")

    def io_callback(self, request, response):
        """
        Callback function to handle IO requests (set/get).

        Args:
            request (any): Set/Get IO request message
            response (any): Response message containing result

        Returns:
            any: Updated response message
        """
        command = request.command
        if command == "set":
            response = self.set_io(request, response)
        elif command == "get":
            response = self.get_io(request, response)
        else:
            response.success = False
            response.message = "Invalid command."
            self.get_logger().error("Invalid command.")
        return response

    def set_io(self, request, response):
        """
        Set the IO state according to the request.

        Args:
            request (any): Set IO request message
            response (any): Response message

        Returns:
            any: Updated response message
        """
        ret_code = self.signals.write(
            SignalType(request.signal_type), request.signal_port, request.signal_value
        )
        return self.handle_io_response(ret_code, response, request)

    def get_io(self, request, response):
        """
        Get the current IO state according to the request.

        Args:
            request (any): Get IO request message
            response (any): Response message

        Returns:
            any: Updated response message
        """
        do_value, ret_code = self.signals.read(
            SignalType(request.signal_type), request.signal_port
        )
        return self.handle_io_response(ret_code, response, request, do_value)

    def handle_io_response(self, ret_code, response, request, signal_value=None):
        """
        Handle the response for IO set/get operations.

        Args:
            ret_code (StatusCodeEnum): Return status of the operation
            response (any): Response message
            request (any): IO request message
            signal_value (optional): Value read from IO, if applicable

        Returns:
            any: Updated response message
        """
        if ret_code == StatusCodeEnum.OK:
            response.success = True
            if request.command == "set":
                response.message = f"{SignalType(request.signal_type).name}[{request.signal_port}] set to {signal_value or request.signal_value}."
            else:
                response.message = f"{SignalType(request.signal_type).name}[{request.signal_port}] is {signal_value or request.signal_value}."
                response.signal_value = signal_value

            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"Failed to set/get IO. Reason: {ret_code.name}"
            self.get_logger().error(ret_code.name)
        return response

    async def move_action_server_execute_callback(self, goal_handle):
        """
        Callback function to handle MoveToPose action requests.

        Args:
            goal_handle: The goal handle containing the request data

        Returns:
            MoveToPose.Result: Result message containing success status and any relevant information
        """
        self.get_logger().info(f"Received goal: {goal_handle.request}")
        ret_code = move_to_pose_cart_without_posture(
            arm=self.arm,
            logger=self.get_logger(),
            x=goal_handle.request.x,
            y=goal_handle.request.y,
            z=goal_handle.request.z,
            c=goal_handle.request.c,
            b=goal_handle.request.b,
            a=goal_handle.request.a,
            vel=goal_handle.request.vel,
            acc=goal_handle.request.acc,
        )
        return await self.handle_move_to_pose_result(ret_code, goal_handle)

    async def handle_move_to_pose_result(self, ret_code, goal_handle):
        """
        Handle the result of the MoveToPose action.

        Args:
            ret_code (StatusCodeEnum): Return code of the operation
            goal_handle: The goal handle containing the request data

        Returns:
            MoveToPose.Result: Result message containing success status and any relevant information
        """
        if ret_code != StatusCodeEnum.OK:
            self.get_logger().error(f"Move to pose failed. Error: {ret_code.name}")
            goal_handle.abort()
            return MoveToPose.Result(
                success=False, message=f"Move failed: {ret_code.name}"
            )

        # Wait for the robot to complete the movement
        while not check_robot_status(self.arm, self.get_logger()):
            posestamp, ret_code = get_current_pose(
                self.arm,
                self.get_clock().now().to_msg(),
                const.CART,
                self.uf_id,
                self.tf_id,
            )
            if ret_code != StatusCodeEnum.OK:
                self.get_logger().error(
                    f"Get current pose failed. Error: {ret_code.name}"
                )
                continue

            feedback_msg = MoveToPose.Feedback()
            feedback_msg.current_pose = posestamp.pose
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(
                f"current pose: {pose_to_xyz_rpy_deg(posestamp.pose)}"
            )
            time.sleep(1)

        #  check the current pose
        _posestamp, ret_code = get_current_pose(
            self.arm,
            self.get_clock().now().to_msg(),
            const.CART,
            self.uf_id,
            self.tf_id,
        )

        if ret_code != StatusCodeEnum.OK:
            self.get_logger().error(f"Get current pose failed. Error: {ret_code.name}")
            goal_handle.abort()
            return MoveToPose.Result(
                success=False, message=f"Move failed: {ret_code.name}"
            )

        goal_handle.succeed()
        self.get_logger().info(f"Move to pose finished.")
        return MoveToPose.Result(success=True, message="Move to pose successfully.")

    def program_control_callback(self, request, response):
        """
        Callback function to handle program control requests (start, stop, pause, resume).

        Args:
            request (any): ProgramControl request message
            response (any): Response message containing result

        Returns:
            any: Updated response message
        """
        command_map = {
            "start": self.start_program,
            "stop": self.stop_program,
            "pause": self.pause_program,
            "resume": self.resume_program,
        }

        command = request.command.lower()
        program_name = request.program_name
        handler = command_map.get(command)

        if handler:
            response = handler(program_name, response)
        else:
            response.success = False
            response.message = f"Unknown command: {command}"
            self.get_logger().error(f"Unknown command: {command}")
        return response

    def start_program(self, program_name, response):
        """
        Start the specified program.

        Args:
            program_name (str): Name of the program to start
            response (any): ProgramControl response message

        Returns:
            any: Updated response message
        """
        return self.control_program(program_name, "start", response)

    def stop_program(self, program_name, response):
        """
        Stop the specified program.

        Args:
            program_name (str): Name of the program to stop
            response (any): ProgramControl response message

        Returns:
            any: Updated response message
        """
        return self.control_program(program_name, "stop", response)

    def pause_program(self, program_name, response):
        """
        Pause the specified program.

        Args:
            program_name (str): Name of the program to pause
            response (any): ProgramControl response message

        Returns:
            any: Updated response message
        """
        return self.control_program(program_name, "pause", response)

    def resume_program(self, program_name, response):
        """
        Resume the specified program.

        Args:
            program_name (str): Name of the program to resume
            response (any): ProgramControl response message

        Returns:
            any: Updated response message
        """
        return self.control_program(program_name, "resume", response)

    def control_program(self, program_name, action, response):
        """
        Perform the specified action (start, stop, pause, resume) on the program.

        Args:
            program_name (str): Name of the program
            action (str): Action to perform (start, stop, pause, resume)
            response (any): ProgramControl response message

        Returns:
            any: Updated response message
        """
        ret_code = getattr(self.execution, f"{action}")(program_name)
        if ret_code != StatusCodeEnum.OK:
            response.success = False
            response.message = (
                f"Failed to {action} program '{program_name}', reason: {ret_code.name}"
            )
            self.get_logger().error(
                f"Failed to {action} program '{program_name}', reason: {ret_code.name}"
            )
        else:
            response.success = True
            response.message = f"Program '{program_name}' {action}ed successfully."
            self.get_logger().info(response.message)
        return response

    def servo_on_off_callback(self, request, response):
        """
        Callback function to handle servo on/off requests.

        Args:
            request (any): Servo on/off request message
            response (any): Servo on/off response message

        Returns:
            any: Updated response message
        """
        is_servo_on = request.servo_on
        ret_code = (
            self.arm.servo_on()
            if is_servo_on
            else self.arm.servo_off()
        )
        if is_servo_on:
            ret_code = self._servo_on_compat(ret_code)
        return self.handle_servo_response(ret_code, response, is_servo_on)

    def _servo_on_compat(self, ret_code):
        """
        Compatibility handler for SDK behavior where servo_on may return
        CONTROLLER_INVALID_OPERATION_RESET even when reset+retry can recover.
        """
        if ret_code == StatusCodeEnum.OK:
            return ret_code

        if ret_code.name != "CONTROLLER_INVALID_OPERATION_RESET":
            return ret_code

        servo_status, status_ret = self.arm.get_servo_status()
        if (
            status_ret == StatusCodeEnum.OK
            and servo_status in (ServoStatusEnum.SERVO_IDLE, ServoStatusEnum.SERVO_RUNNING)
        ):
            self.get_logger().warning(
                "servo_on returned CONTROLLER_INVALID_OPERATION_RESET, but servo is already active"
            )
            return StatusCodeEnum.OK

        reset_ret = self.arm.alarm.reset()
        if reset_ret != StatusCodeEnum.OK:
            self.get_logger().error(f"alarm reset failed: {reset_ret.name}")
            return ret_code

        retry_ret = self.arm.servo_on()
        if retry_ret == StatusCodeEnum.OK:
            self.get_logger().info("servo on recovered after alarm reset")
            return retry_ret

        if retry_ret.name == "CONTROLLER_INVALID_OPERATION_RESET":
            servo_status, status_ret = self.arm.get_servo_status()
            if (
                status_ret == StatusCodeEnum.OK
                and servo_status
                in (ServoStatusEnum.SERVO_IDLE, ServoStatusEnum.SERVO_RUNNING)
            ):
                self.get_logger().warning(
                    "servo_on retry returned CONTROLLER_INVALID_OPERATION_RESET, but servo is active"
                )
                return StatusCodeEnum.OK

        return retry_ret

    def handle_servo_response(self, ret_code, response, is_servo_on):
        """
        Handle the response for servo on/off requests.

        Args:
            ret_code (StatusCodeEnum): Return status of the servo operation
            response (any): Response message
            is_servo_on (bool): Whether the servo should be turned on or off

        Returns:
            any: Updated response message
        """
        if ret_code != StatusCodeEnum.OK:
            response.success = False
            response.message = f"Failed to {'servo on' if is_servo_on else 'servo off'}, reason: {ret_code.name}"
        else:
            response.success = True
            response.message = f"Servo {'on' if is_servo_on else 'off'} successfully."
        self.get_logger().info(response.message)
        return response

    def send_script_callback(self, request, response):
        """
        Callback function to handle send script requests.

        Args:
            request (any): Send script request message
            response (any): Send script response message

        Returns:
            any: Updated response message
        """
        script_name = request.script_name
        script_content = request.script_content.replace("\\n", "\n")
        bs = [script_name] + script_content.split("\n")

        ret_code = self.execution.execute_bas_script(bs)
        if ret_code != StatusCodeEnum.OK:
            response.success = False
            response.message = (
                f"Failed to send script '{script_name}', reason: {ret_code.name}"
            )
        else:
            response.success = True
            response.message = f"Script '{script_name}' sent successfully."
        return response

    def emergency_stop_callback(self, request, response):
        """
        Callback function to handle emergency stop requests.

        Args:
            request (any): Emergency stop request message
            response (any): Emergency stop response message

        Returns:
            any: Updated response message
        """
        # Check if the arm supports emergency stop (estop)
        if not hasattr(self.arm, "estop"):
            msg = "The emergency stop feature is not available in the current SDK.Next version will support this feature."
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response
        ret_code = self.arm.estop()
        if ret_code != StatusCodeEnum.OK:
            response.success = False
            response.message = f"Failed to emergency stop"
            self.get_logger().error(f"Failed to emergency stop, reason: {ret_code.name}")
            return response

        response.success = True
        response.message = "Emergency stop initiated."
        return response

    def led_callback(self, request, response):
        """
        Callback function to handle LED control requests.

        Args:
            request (any): LED control request message
            response (any): LED control response message

        Returns:
            any: Updated response message
        """
        led_on = request.led_on
        safe_call(self.arm.switch_led_light, led_on, prefix=f"set led light {led_on}")
        response.success = True
        response.message = f"LED {'on' if led_on else 'off'} successfully."
        self.get_logger().info(response.message)
        return response

    def __del__(self):
        """Disconnect from the robot when the object is destroyed."""
        if self.arm:
            self.arm.disconnect()
        self.get_logger().info("Disconnected from the robot.")


def main(args=None):
    """
    Main function to initialize and spin the ROS2 node.

    :param args: Command line arguments
    """
    rclpy.init(args=args)
    node = ServiceServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
