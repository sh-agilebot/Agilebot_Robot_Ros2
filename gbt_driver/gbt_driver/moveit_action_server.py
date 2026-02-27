"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
action server is used to receive trajectory points from moveit2 and call the robot control interface for motion control
"""

import os
import sys
import time
from functools import wraps

import rclpy
import yaml
from Agilebot.IR.A.arm import Arm
from Agilebot.IR.A.common.const import const
from Agilebot.IR.A.file_manager import ROBOT_TMP, FileManager
from Agilebot.IR.A.sdk_classes import SoftModeEnum
from Agilebot.IR.A.sdk_types import (
    RobotStatusEnum,
    ServoStatusEnum,
)
from Agilebot.IR.A.status_code import StatusCodeEnum
from ament_index_python.packages import get_package_share_directory
from common.utils.arm_utils import (
    get_current_pose,
    prepare_offline_trajectory_compat,
    safe_call,
    wait_for_robot_ready,
    wait_for_transform_ready,
)
from common.utils.trajectory_utils import (
    save_joint_trajectory_to_file,
    trajectory_interpolate,
)
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.node import Node


def method_timeit(func):
    """
    Decorator for class methods to log execution time. Assumes the class instance has a logger attribute.
    """

    @wraps(func)
    def wrapper(self, *args, **kwargs):
        start = time.perf_counter()
        result = func(self, *args, **kwargs)
        elapsed = time.perf_counter() - start
        self._logger.info(
            f"[TIMING] '{func.__name__}' executed in {elapsed:.4f} seconds"
        )
        return result

    return wrapper


class FollowJointTrajectoryServer(Node):
    def __init__(self):
        super().__init__("action_server")
        self.get_logger().info("Initializing FollowJointTrajectory action server...")

        self.trajectory_file_name = "action_server_trajectory.csv"
        params = [
            ("robot_type", "C5A"),
            ("interpolate_mode", "quintic"),  # "quintic" or "spline"
        ]

        # Declare action server parameters
        self.declare_parameters(namespace="", parameters=params)
        # get parameters
        self.robot_type = self.get_parameter("robot_type").value
        self.action_server_name = (
            f"/gbt_{self.robot_type.lower()}_arm_controller/follow_joint_trajectory"
        )
        self.interpolate_mode = self.get_parameter("interpolate_mode").value

        # Log parameters
        for name, _ in params:
            value = self.get_parameter(name).value
            self.get_logger().info(f"Parameter: {name} = {value}")

        # Load robot configuration file
        config_path = os.path.join(
            get_package_share_directory("gbt_driver"), "config/robot_config.yaml"
        )
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found: {config_path}")
            raise FileNotFoundError(f"Missing robot configuration file: {config_path}")

        with open(config_path, "r", encoding="utf-8") as f:
            robot_config = yaml.safe_load(f)
            self.robot_ip_address = robot_config.get("robot_ip_address")
            if not self.robot_ip_address:
                self.get_logger().error(
                    "Robot IP address is missing in the configuration file."
                )
                raise ValueError(
                    "Missing 'robot_ip_address' field in the configuration file."
                )

        # Initialize robot connection
        self._arm = Arm()
        safe_call(self._arm.connect, self.robot_ip_address, prefix="connect")

        # Servo reset and enable
        safe_call(self._arm.servo_reset, prefix="servo reset")

        # Check robot operation mode (soft mode)
        # NOTE: Real robot soft mode can ONLY be changed using the physical key switch on the teach pendant.
        # There is NO software interface (set_op_mode) to change the soft mode - this is a hardware safety feature.
        op_mode, _ = safe_call(self._arm.get_op_mode, prefix="get op mode")
        if op_mode != SoftModeEnum.AUTO:
            self.get_logger().warning(
                f"Robot is in {op_mode.name} mode. MoveIt2 control requires AUTO mode."
            )
            self.get_logger().warning(
                "Please switch to AUTO mode using the key switch on the teach pendant."
            )
            # Do not return - allow server to start for testing purposes, but execution will fail if not in AUTO mode

        remote_robot_type, _ = safe_call(
            self._arm.get_arm_model_info, prefix="get arm model info"
        )
        if not remote_robot_type.endswith(self.robot_type):
            self.get_logger().error(
                f"Robot Type does not match,expected {self.robot_type}, got {remote_robot_type}"
            )
            sys.exit(1)

        # Initialize moveit action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_server_name,
            self.moveit_callback,
        )
        self.get_logger().info("FollowJointTrajectory action server is ready.")

    @method_timeit
    def run_trajectory(self, arm: Arm, local_file_path, goal_handle) -> StatusCodeEnum:
        """
        Complete process for uploading, converting, and running a trajectory file.

        :param arm: Connected Arm instance.
        :param local_file_path: Local path to the CSV trajectory file.
        :param goal_handle: Action goal handle for feedback and result.
        :return: StatusCodeEnum indicating the result.
        """
        file_name = os.path.basename(local_file_path)
        file_manager = FileManager(arm.controller_ip)

        # Upload file to robot
        safe_call(
            file_manager.upload,
            local_file_path,
            ROBOT_TMP,
            True,
            prefix=" trajectory file upload",
        )

        # Convert CSV to trajectory format
        file_dir, _ = safe_call(
            arm.trajectory.transform_csv_to_trajectory,
            file_name,
            io_flag="2",
            prefix="transform csv to trajectory",
        )

        transformed_file_name = os.path.basename(file_dir)

        self._logger.info(f"trajectory file name:{transformed_file_name}")

        # Check conversion status
        self._logger.info("waiting for transform...")
        start_time = time.time()
        if not wait_for_transform_ready(arm, transformed_file_name, self._logger):
            self._logger.error("trajectory transform failed")
            return StatusCodeEnum.SERVER_ERR
        self._logger.info(f"transform finished.Use time: {time.time() - start_time}s")

        # Set offline trajectory file
        safe_call(
            arm.trajectory.set_offline_trajectory_file,
            transformed_file_name,
            prefix="set offline trajectory file",
        )

        # Prepare offline trajectory execution
        safe_call(
            prepare_offline_trajectory_compat,
            arm,
            self._logger,
            prefix="prepare offline trajectory",
        )

        # Wait for robot and servo to be ready
        self._logger.info("wait for preparing...")
        while True:
            robot_status, ret_robot = arm.get_robot_status()
            servo_status, ret_servo = arm.get_servo_status()

            feedback_msg = FollowJointTrajectory.Feedback()
            current_positions = get_current_pose(
                self._arm, self.get_clock().now().to_msg(), const.JOINT
            )[0].position
            feedback_msg.joint_names = goal_handle.request.trajectory.joint_names
            feedback_msg.desired.positions = []
            feedback_msg.actual.positions = current_positions
            feedback_msg.error.positions = []
            goal_handle.publish_feedback(feedback_msg)

            if (
                ret_robot != StatusCodeEnum.OK
                or ret_servo != StatusCodeEnum.OK
                or (
                    robot_status == RobotStatusEnum.ROBOT_IDLE
                    and servo_status == ServoStatusEnum.SERVO_IDLE
                )
            ):
                break
            time.sleep(0.5)

        # Execute offline trajectory
        return arm.trajectory.execute_offline_trajectory()

    @method_timeit
    def _trajectory_interpolate(self, trajectory, local_file_path) -> bool:
        """
        Interpolate trajectory points and save to file.

        :param trajectory: FollowJointTrajectory.Goal
        :param local_file_path: Path to save the interpolated CSV file
        :return: True on success, False on failure
        """
        return trajectory_interpolate(
            trajectory, local_file_path, self.interpolate_mode
        )

    async def moveit_callback(self, goal_handle):
        """
        Callback to execute received trajectory goals.

        :param goal_handle: Action goal handle containing the trajectory goal
        :return: FollowJointTrajectory.Result
        """
        self.get_logger().info("Received a trajectory goal from MoveIt.")

        trajectory = goal_handle.request.trajectory

        if not hasattr(trajectory, "joint_names") or not hasattr(trajectory, "points"):
            self.get_logger().error(
                "Invalid trajectory: Missing joint names or trajectory points."
            )
            goal_handle.abort()
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.INVALID_GOAL
            )

        try:
            # Determine local file path
            file_path = os.path.join("/tmp", self.trajectory_file_name)

            # Interpolate and save
            self._trajectory_interpolate(trajectory, file_path)
            self.get_logger().info(
                f"Interpolation and save completed，file save to  {file_path}"
            )

            # Run trajectory
            ret_code = self.run_trajectory(
                self._arm, file_path, goal_handle=goal_handle
            )

            if ret_code == StatusCodeEnum.OK:
                # wait for completion and check status every 0.5s
                while not wait_for_robot_ready(self._arm, self._logger, 0.5):
                    pass
                goal_handle.succeed()
                self.get_logger().info("Trajectory execution completed successfully.")
                return FollowJointTrajectory.Result(
                    error_code=FollowJointTrajectory.Result.SUCCESSFUL
                )
            else:
                goal_handle.abort()
                self.get_logger().error(f"Trajectory execution failed: {ret_code.name}")
                return FollowJointTrajectory.Result(
                    error_code=FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                )
        except Exception as e:
            self.get_logger().error(f"Error while executing the goal: {str(e)}")
            goal_handle.abort()
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.INVALID_GOAL
            )


def main(args=None):
    rclpy.init(args=args)
    node = FollowJointTrajectoryServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down safely.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
