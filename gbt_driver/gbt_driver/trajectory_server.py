"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
This node is used to control the robot arm to execute a trajectory.

If you need to interplate the trajectory, please refer to the following file:
Agilebot_Robot_Ros2/common/common/utils/trajectory_utils.py

"""

import os
from concurrent.futures import ThreadPoolExecutor

import rclpy
import yaml
from Agilebot.IR.A.arm import Arm
from Agilebot.IR.A.common.const import const
from Agilebot.IR.A.file_manager import ROBOT_TMP, FileManager
from ament_index_python.packages import get_package_share_directory
from common.utils.arm_utils import (
    get_current_pose,
    prepare_offline_trajectory_compat,
    safe_call,
    wait_for_robot_ready,
    wait_for_transform_ready,
)
from gbt_interface.action import OfflineTrajectory
from rclpy.action import ActionServer
from rclpy.node import Node


class TrajectoryActionServer(Node):
    def __init__(self):
        super().__init__("trajectory_action_server")

        # parameters
        _parameters = [
            # ('use_interpolation', True), # Whether to use interpolation when executing the trajectory
        ]
        self.declare_parameters(namespace="", parameters=_parameters)
        # print parameters
        for param in _parameters:
            self.get_logger().info(
                f"Parameter {param[0]}: {self.get_parameter(param[0]).value}"
            )

        # get parameters
        # self.use_interpolation= self.get_parameter('use_interpolation').value

        # Load robot configuration from YAML file
        config_path = os.path.join(
            get_package_share_directory("gbt_driver"), "config/robot_config.yaml"
        )
        with open(config_path, "r", encoding="utf-8") as f:
            robot_config = yaml.safe_load(f)
            self.robot_ip_address = robot_config[
                "robot_ip_address"
            ]  # IP address of the robot controller

        self.arm = Arm()  # Initialize robot arm interface in development mode

        # Attempt to establish connection with robot controller
        safe_call(self.arm.connect, self.robot_ip_address,prefix=f"connect to robot at {self.robot_ip_address}")

        # Create action server for trajectory execution
        self._action_server = ActionServer(
            self,
            OfflineTrajectory,
            "gbt_driver/trajectory",
            execute_callback=self.execute_callback,
        )

        # Thread pool for asynchronous trajectory execution
        self.thread_pool = ThreadPoolExecutor(max_workers=1)
        self.get_logger().info(
            f"Successfully connected to robot at {self.robot_ip_address}"
        )
        self.get_logger().info("Trajectory action server initialized and running.")

    async def execute_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle)->OfflineTrajectory.Result:
        """
        Callback function for trajectory execution action server
        Args:
            goal_handle (rclpy.action.server.ServerGoalHandle): the goal handle of the action server
        Returns:
            OfflineTrajectory.Result: the result of the action server

        """
        self.get_logger().info("New trajectory execution request received")
        feedback_msg = OfflineTrajectory.Feedback()
        result = OfflineTrajectory.Result()

        # Extract trajectory file path from goal request
        local_file_path = goal_handle.request.trajectory_path

        if not os.path.isfile(local_file_path):
            self.get_logger().error(
                f"Trajectory file not found at specified path: {local_file_path}"
            )
            result.result = -1
            result.message = "Trajectory file not found."
            goal_handle.abort()
            return result

        # Execute trajectory in separate thread to avoid blocking ROS2 event loop
        future = self.thread_pool.submit(
            self.run_trajectory_with_feedback, local_file_path, feedback_msg,goal_handle
        )
        try:
            success = future.result()  # Wait for trajectory execution completion
        except Exception as e:
            self.get_logger().error(
                f"Trajectory execution failed with exception: {str(e)}"
            )
            result.result = -2
            result.message = f"Execution error: {str(e)}"
            goal_handle.abort()
            return result

        if not success:
            result.result = -1
            result.message = "Trajectory execution failed."
            goal_handle.abort()
            return result

        result.result = 0
        result.message = "Trajectory executed successfully."
        goal_handle.succeed()
        return result

    def run_trajectory_with_feedback(self, local_file_path:str, feedback_msg:OfflineTrajectory.Feedback,goal_handle:rclpy.action.server.ServerGoalHandle)->bool:
        """
        Execute trajectory file with feedback updates.

        Args:
            local_file_path (str): the path to the local trajectory file.(CSV file)
            feedback_msg (OfflineTrajectory.Feedback): the feedback message to be updated during execution.
            goal_handle (rclpy.action.server.ServerGoalHandle): the goal handle for the action server.

        Returns:
        
            bool: True if trajectory execution is successful, False otherwise.

        """
        file_name = os.path.basename(local_file_path)
        file_manager = FileManager(self.arm.controller_ip)

        # 1. Upload trajectory file to robot controller
        safe_call(file_manager.upload, local_file_path, ROBOT_TMP, True,prefix=f"upload {file_name} to robot")

        # 2. Convert CSV file to robot-specific trajectory format 
        file_dir,_=safe_call(self.arm.trajectory.transform_csv_to_trajectory,file_name, io_flag="2", prefix=f"convert {file_name} to trajectory")
        self.get_logger().info("CSV to trajectory conversion successful")

        transformed_file_name = os.path.basename(file_dir)

        # 3. Monitor trajectory conversion status
        self._logger.info("wait for transform...")
        if not wait_for_transform_ready(self.arm, transformed_file_name, self.get_logger()):
            return False
        self.get_logger().info("Trajectory conversion validated successfully")

        # 5. Set converted trajectory as active offline program
        safe_call(self.arm.trajectory.set_offline_trajectory_file, transformed_file_name, prefix="set offline trajectory file")
      
        # 6. Prepare trajectory execution on robot
        safe_call(
            prepare_offline_trajectory_compat,
            self.arm,
            self.get_logger(),
            prefix="prepare offline trajectory",
        )
     
        # 7. Wait for robot system to be ready (robot and servos),Check every 0.5 seconds.
        while not wait_for_robot_ready(self.arm, self._logger, 0.5):
            self.get_logger().info("Waiting for robot system to become ready...")
            

        # 8. Execute the prepared trajectory
        safe_call(self.arm.trajectory.execute_offline_trajectory, prefix="execute offline trajectory")
 
        # 9. Monitor execution progress and provide feedback
        while not wait_for_robot_ready(self.arm, self._logger, 0.5):
            # Get current joint positions from robot
            joint_state,_=safe_call(get_current_pose,self.arm, stamp=self.get_clock().now().to_msg(), pose_type=const.JOINT,
                                    prefix="get current joint positions")
           
            # Update action feedback with current joint state
            feedback_msg.joint_states = joint_state
            goal_handle.publish_feedback(feedback_msg)

        return True


def main(args=None):
    rclpy.init(args=args)

    action_server = TrajectoryActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
