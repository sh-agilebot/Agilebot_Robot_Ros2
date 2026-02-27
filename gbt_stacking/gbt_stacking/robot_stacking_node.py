"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
Send request to AgileGaze to get detected objects, then complete the grasp
- Read parameters from configuration file
- Connect to robot
- Create user coordinate systems (grasp surface user coordinate system and placement surface user coordinate system) according to configuration file
- Get grasp points according to object information returned by AgileGaze, and calculate stacking layout to get placement points according to the specifications of the box
- Write the above points to PR registers and send them to the visualization node stacking_visualizer
- Generate BAS script according to the above registers and send it to the robot
- The robot executes the BAS script to complete the grasp and placement
"""

import math
import os
import sys
import threading
from typing import Dict, List

import numpy as np
import rclpy
import yaml
from Agilebot.IR.A.arm import Arm
from Agilebot.IR.A.bas_script import BasScript
from Agilebot.IR.A.coordinate_system import CoordinateSystemType
from Agilebot.IR.A.script_types import (
    AssignType,
    IOStatus,
    MovePoseType,
    OtherType,
    ParamType,
    SmoothType,
    SpeedType,
    ValueType,
)
from Agilebot.IR.A.sdk_classes import PoseRegister, Posture
from Agilebot.IR.A.sdk_types import PoseType
from Agilebot.IR.A.status_code import StatusCodeEnum
from ament_index_python.packages import get_package_share_directory
from common.utils.arm_utils import safe_call, wait_for_robot_ready
from common.utils.coord_utils import coord_id_exists, three_points_to_pose
from gbt_interface.srv import SendScript
from gbt_stacking.utils import generate_stacking_layout_3d
from gbt_stacking_interface.srv import (
    ExternalStackingTrigger,
    GetAgileGaze,
    PalletVisualizer,
)
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# from tf_transformations import quaternion_from_euler
from scipy.spatial.transform import Rotation as R


def main(args=None):
    """
    Main function to run the robot_stacking_node.
    """
    rclpy.init(args=args)
    node = RobotStackingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


class RobotStackingNode(Node):
    """
    ROS2 node for vision-guided robotic stacking operations.
    - Calls vision service to detect objects.
    - Computes pallet stacking layout.
    - Registers pose registers for grasp/place.
    - Generates and sends BAS scripts to robot.
    """

    def __init__(self):
        super().__init__("robot_stacking_node")
        self._declare_and_get_params()
        self._log_parameters()
        self._init_robot_arm()
        self._wait_for_services()
        self._create_user_frames()
        self._register_home_pose()
        self.get_logger().info("RobotStackingNode ready.")
        self._create_services_start()
        self.get_logger().info("RobotStackingNode services created.")

        # State management variables
        self._current_layout_item_index = 0
        self._stacking_in_progress = False
        self._vision_data = None
        self._layout_data = None
        self._script_future = None

    def _declare_and_get_params(self) -> None:
        """Declare and retrieve all required ROS parameters."""
        params = [
            ("grasp_uf_id", 1),
            ("place_uf_id", 2),
            ("tf_id", 0),
            ("vision_service", "/gbt_vision/service/AgileGaze"),
            ("send_script_service", "/gbt_driver/service_server/send_script"),
            ("pallet_w", 250.0),
            ("pallet_l", 250.0),
            ("box_w", 100.0),
            ("box_l", 100.0),
            ("box_h", 100.0),
            ("layers", 3),
            ("spacing_x", 20.0),
            ("spacing_y", 20.0),
            ("spacing_z", 20.0),
            ("svr_timeout", 5.0),
            ("grasp_top", 20.0),
            ("grasp_lift", 110.0),
            (
                "uf_grasp_points",
                [580.0, 175.0, 70.0, 700.0, 175.0, 70.0, 580.0, 190.0, 70.0],
            ),
            (
                "uf_place_points",
                [-540.0, -45.0, 70.0, -400.0, -45.0, 70.0, -540.0, 172.65, 70.0],
            ),
            ("home_point", [90.934, 77.43, -61.098, 73.667, -90.0, -179.066]),
            ("grasp_time", 2.0),
            ("IO_type", "DO"),
            ("IO_id", 1),
            ("global_speed", 100.0),
            ("close_speed", 100.0),
            ("move_speed", 500.0),
            ("robot_type", "C5A"),
            ("external_trigger_service", "/gbt_stacking/external_trigger"),
            ("script_timeout", 35.0),
        ]
        self.declare_parameters(namespace="", parameters=params)

        # Retrieve parameters
        self.grasp_uf_id = self.get_parameter("grasp_uf_id").value
        self.place_uf_id = self.get_parameter("place_uf_id").value
        self.tf_id = self.get_parameter("tf_id").value
        self.vision_service = self.get_parameter("vision_service").value
        self.send_script_service = self.get_parameter("send_script_service").value
        self.pallet_w = self.get_parameter("pallet_w").value
        self.pallet_l = self.get_parameter("pallet_l").value
        self.box_w = self.get_parameter("box_w").value
        self.box_l = self.get_parameter("box_l").value
        self.box_h = self.get_parameter("box_h").value
        self.layers = self.get_parameter("layers").value
        self.spacing_x = self.get_parameter("spacing_x").value
        self.spacing_y = self.get_parameter("spacing_y").value
        self.spacing_z = self.get_parameter("spacing_z").value
        self.svr_timeout = self.get_parameter("svr_timeout").value
        self.grasp_top = self.get_parameter("grasp_top").value
        self.grasp_lift = self.get_parameter("grasp_lift").value
        self.uf_grasp_points = np.array(
            self.get_parameter("uf_grasp_points").value
        ).reshape(-1, 3)
        self.uf_place_points = np.array(
            self.get_parameter("uf_place_points").value
        ).reshape(-1, 3)
        self.home_point = self.get_parameter("home_point").value
        self.grasp_time = self.get_parameter("grasp_time").value
        self.IO_type = self.get_parameter("IO_type").value
        self.IO_type = AssignType(self.IO_type)
        self.IO_id = self.get_parameter("IO_id").value
        self.global_speed = self.get_parameter("global_speed").value
        self.close_speed = self.get_parameter("close_speed").value
        self.move_speed = self.get_parameter("move_speed").value
        self.robot_type = self.get_parameter("robot_type").value
        self.external_trigger_service = self.get_parameter(
            "external_trigger_service"
        ).value
        self.script_timeout = self.get_parameter("script_timeout").value

    def _log_parameters(self) -> None:
        """Log all parameters for debugging."""
        for p in [
            "grasp_uf_id",
            "place_uf_id",
            "tf_id",
            "vision_service",
            "send_script_service",
            "pallet_w",
            "pallet_l",
            "box_w",
            "box_l",
            "box_h",
            "layers",
            "spacing_x",
            "spacing_y",
            "spacing_z",
            "svr_timeout",
            "grasp_top",
            "grasp_lift",
            "grasp_time",
            "global_speed",
            "close_speed",
            "move_speed",
        ]:
            val = self.get_parameter(p).value
            self.get_logger().info(f"Param {p}: {val}")

    def _init_robot_arm(self) -> None:
        """Initialize and connect to the robot arm using YAML config."""
        cfg_path = os.path.join(
            get_package_share_directory("gbt_driver"), "config", "robot_config.yaml"
        )
        with open(cfg_path, "r") as f:
            cfg = yaml.safe_load(f)
        self.robot_ip = cfg["robot_ip_address"]

        # connect to robot
        self.arm = Arm()
        code = self.arm.connect(self.robot_ip)
        if code != StatusCodeEnum.OK:
            self.get_logger().error(f"Failed to connect: {StatusCodeEnum(code).errmsg}")
            sys.exit(1)

        self.get_logger().info(f"Connected to robot at {self.robot_ip}")

        # check robot type
        remote_robot_type = self.arm.get_arm_model_info()[0]
        self.get_logger().info(f"Robot info: {remote_robot_type}")
        if not remote_robot_type.endswith(self.robot_type):
            self.get_logger().error(
                f"robot type does not match,expected {self.robot_type}, got {remote_robot_type}"
            )
            sys.exit(1)

        # safe_call(self.arm.servo_reset)

        # check robot status
        if not wait_for_robot_ready(self.arm, self.get_logger(), max_wait_time=7):
            self.get_logger().error("Robot not ready; please check its status.")
            raise RuntimeError("Robot not ready; please check its status.")

        # set posture
        self.posture = Posture()
        self.posture.turnCircle = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.posture.wrist_flip = 1
        self.posture.arm_up_down = 1
        self.posture.arm_back_front = 1
        self.posture.arm_left_right = 0
        self.get_logger().info(f"arm posture:{vars(self.posture)}")

    def _wait_for_services(self) -> None:
        """Wait for AgileGaze and send_script services."""
        # ReentrantCallbackGroup allows concurrent service calls
        reentrant_cb_group = ReentrantCallbackGroup()

        # Vision service for object detection
        self.vision_client = self.create_client(
            GetAgileGaze, self.vision_service, callback_group=reentrant_cb_group
        )
        if not self.vision_client.wait_for_service(timeout_sec=self.svr_timeout):
            self.get_logger().error("Vision service unavailable.")
            sys.exit(1)

        # Script service for sending scripts to the robot
        self.script_client = self.create_client(
            SendScript, self.send_script_service, callback_group=reentrant_cb_group
        )
        if not self.script_client.wait_for_service(timeout_sec=self.svr_timeout):
            self.get_logger().error("SendScript service unavailable.")
            sys.exit(1)

        # Pallet visualizer service.Send the points to the pallet visualizer.
        self.points_client = self.create_client(
            PalletVisualizer,
            "gbt_stacking/pallet_visualizer",
            callback_group=reentrant_cb_group,
        )
        if not self.points_client.wait_for_service(timeout_sec=self.svr_timeout):
            self.get_logger().error("Pallet visualizer service unavailable.")
            sys.exit(1)

    def _create_services_start(self) -> None:
        """
        Create services and start its.
        Receive the trigger from the external service.Then start the stacking process.
        """
        # Shared state variable
        self._trigger_requested = False
        self._stacking_in_progress = False
        self._last_request_time = None

        def _task_trigger_callback(
            req: ExternalStackingTrigger.Request, resp: ExternalStackingTrigger.Response
        ) -> ExternalStackingTrigger.Response:
            """Callback for the task trigger service.

            Args:
                req (ExternalStackingTrigger.Request): _request_
                resp (ExternalStackingTrigger.Response): _response_
            Returns:
                ExternalStackingTrigger.Response: _description_

            """
            current_time = self.get_clock().now().nanoseconds

            # Check request frequency
            if (
                self._last_request_time
                and (current_time - self._last_request_time) < 1e9
            ):  # 1s
                resp.success = False
                resp.message = "Request too frequent. Please wait."
                self.get_logger().warn(resp.message)
                return resp

            self._last_request_time = current_time

            # check if stacking is in progress
            if self._stacking_in_progress:
                resp.success = False
                resp.message = "Stacking in progress. Try again later."
                self.get_logger().warn(resp.message)
                return resp

            # Accept request
            if req.trigger:
                self._trigger_requested = True
                resp.success = True
                resp.message = "Trigger accepted. Execution will start shortly."
                self.get_logger().info(resp.message)
            else:
                resp.success = True
                resp.message = "Trigger reset."
                self._trigger_requested = False
                self.get_logger().info(resp.message)

            return resp

        def _timer_callback():
            """
            Timer callback to check if the trigger has been requested and the stacking process is not in progress.
            If so, start the stacking process.
            """
            if self._trigger_requested and not self._stacking_in_progress:
                self._stacking_in_progress = True
                self._trigger_requested = False
                self.get_logger().info("Starting stacking cycle...")

                # Start the async stacking process
                self._start_async_stacking()

        # create service of trigger.Receive the trigger from the external service.
        self._trigger_srv = self.create_service(
            ExternalStackingTrigger,
            self.external_trigger_service,
            _task_trigger_callback,
            # callback_group=MutuallyExclusiveCallbackGroup(),
            callback_group=ReentrantCallbackGroup(),
        )

        # create timer to check the trigger and start the stacking process.
        self._stacking_timer = self.create_timer(
            0.1, _timer_callback, callback_group=ReentrantCallbackGroup()  # 100ms
        )

    def _start_async_stacking(self):
        """start the async stacking process"""
        self._current_item_index = 0
        self._call_vision_async()

    def _call_vision_async(self):
        """Start the vision service asynchronously"""
        self.get_logger().info("Calling vision service...")
        req = GetAgileGaze.Request()
        future = self.vision_client.call_async(req)
        future.add_done_callback(self._vision_callback)

    def _vision_callback(self, future):
        """Callback for the vision service"""
        try:
            result = future.result()
            self._vision_data = result.agile_gaze
            self.get_logger().info(f"Vision response: {self._vision_data}")

            if self._vision_data.code != 0 or self._vision_data.quantity < 1:
                self.get_logger().error("No objects to stack.")
                self._stacking_in_progress = False
                return

            # compute layout of pallette
            if self._layout_data is None:
                self._layout_data = self._compute_layout()
                self._logger.info(f"Layout data: {self._layout_data}")

            if self._current_layout_item_index + self._vision_data.quantity > len(
                self._layout_data
            ):
                self.get_logger().error(
                    "No more layout items available.Pallette is full."
                )
                self._stacking_in_progress = False
                return

            # visualize points(place/pick)
            self._visualize_points(self._vision_data, self._layout_data)

            # process item
            self._process_item()

        except Exception as e:
            self.get_logger().error(f"Vision service failed: {str(e)}")
            self._stacking_in_progress = False

    def _process_item(self):
        """process item"""
        if self._current_item_index >= self._vision_data.quantity:
            self.get_logger().info("Stacking finished.")
            self._stacking_in_progress = False
            return
        if self._current_layout_item_index >= len(self._layout_data):
            self.get_logger().error("No more layout items available.Pallette is full.")
            self._stacking_in_progress = False
            return

        item = self._vision_data.vr_list[self._current_item_index]
        self.get_logger().info(
            f"Processing item {self._current_item_index+1}/{self._vision_data.quantity}"
        )

        # register PointRegister
        self._register_cartesian_prs(
            self._current_layout_item_index, item, self._layout_data
        )

        # generate script
        script = self._generate_bas_script(self._current_item_index)
        self.get_logger().info(f"Generated BAS script:\n{'='*50}\n{script}\n{'='*50}")

        # wait for robot ready
        if not wait_for_robot_ready(
            self.arm, self.get_logger(), max_wait_time=self.script_timeout
        ):
            self.get_logger().error("Robot not ready. Aborting stacking.")
            self._stacking_in_progress = False
            return

        # send script to robot
        self._send_script_async(script)

    def _send_script_async(self, script_text: str):
        """Send script to robot asynchronously"""
        req = SendScript.Request(script_name="stacking", script_content=script_text)
        self._script_future = self.script_client.call_async(req)
        self._script_future.add_done_callback(self._script_sent_callback)

    def _script_sent_callback(self, future):
        """Callback for the script service"""
        try:
            res = future.result()
            if not res.success:
                self.get_logger().error(f"Script send failed: {res.message}")
                self._stacking_in_progress = False
                return

            self.get_logger().info(
                "Script sent successfully. Waiting for completion..."
            )

            # Start a timer to check if the robot has completed the action
            self._completion_timer = self.create_timer(
                1.0,  # frequency: 1Hz
                self._check_robot_completion,
                callback_group=ReentrantCallbackGroup(),
            )

        except Exception as e:
            self.get_logger().error(f"Script send failed: {str(e)}")
            self._stacking_in_progress = False

    def _check_robot_completion(self):
        """Check if the robot has completed the action"""
        if not wait_for_robot_ready(self.arm, self.get_logger(), max_wait_time=0.1):
            self._logger.info("Robot still running.")
            return  # Robot still running

        # Complete the action
        self.destroy_timer(self._completion_timer)
        self.arm.execution.stop("stacking")
        self.get_logger().info(f"Item {self._current_item_index+1} stacking complete.")

        # Move to next item
        self._current_item_index += 1
        self._current_layout_item_index += 1
        self._process_item()

    def _create_user_frames(self) -> None:
        """
        Create or update grasp and place user frames via three-point definition.
        """
        existing, code = self.arm.coordinate_system.get_coordinate_list(
            CoordinateSystemType.UserFrame
        )
        if code != StatusCodeEnum.OK:
            self.get_logger().error(f"Cannot list user frames:{code.name}")
            return

        for name, uf_id, pts in [
            ("grasp_uf", self.grasp_uf_id, self.uf_grasp_points),
            ("place_uf", self.place_uf_id, self.uf_place_points),
        ]:
            p1, p2, p3 = pts
            uf_pose = three_points_to_pose(
                p1.tolist(),
                p2.tolist(),
                p3.tolist(),
                coord_id=uf_id,
                name=name,
                comment=f"{name} frame",
                group_id=1,
            )
            if not coord_id_exists(existing, uf_id):
                add_code = self.arm.coordinate_system.add(
                    CoordinateSystemType.UserFrame, uf_pose
                )
                if add_code == StatusCodeEnum.OK:
                    self.get_logger().info(f"Frame {name} (ID {uf_id}) added.")
                else:
                    self.get_logger().warning(
                        f"Add frame {name} failed: {add_code.name}"
                    )
            else:
                upd = self.arm.coordinate_system.update(
                    CoordinateSystemType.UserFrame, uf_pose
                )
                if upd == StatusCodeEnum.OK:
                    self.get_logger().info(f"Frame {name} (ID {uf_id}) updated.")
                else:
                    self.get_logger().error(f"Update frame {name} failed: {upd.name}")

    def _register_home_pose(self) -> None:
        """
        Register the robot's home pose as PR 1.
        """
        pr = PoseRegister(index=1)
        pr.name = "home_pr"
        pr.comment = "home pose"
        pr.poseRegisterData.pt = PoseType.JOINT
        for i, val in enumerate(self.home_point, start=1):
            setattr(pr.poseRegisterData.joint, f"j{i}", val)
        assert self.arm.register.write_PR(pr), "Write home PR failed"

    def _compute_layout(self) -> List[List[float]]:
        """
        Generate stacking layout on pallet.

        return:
            List of [idx, x, y, z] for each place point.
        """
        return generate_stacking_layout_3d(
            self.pallet_w,
            self.pallet_l,
            self.box_w,
            self.box_l,
            self.box_h,
            layers=self.layers,
            spacing_x=self.spacing_x,
            spacing_y=self.spacing_y,
            spacing_z=self.spacing_z,
        )

    def _euler_to_quaternion_msg(
        self, roll: float, pitch: float, yaw: float
    ) -> Quaternion:
        """
        Convert euler angles to quaternion message.
        Args:

            roll (float): Roll angle in radians
            pitch (float): Pitch angle in radians
            yaw (float): Yaw angle in radians
        Returns:
            Quaternion: Quaternion message
        """

        # Create Rotation object (rotation order zyx)
        rotation = R.from_euler("zyx", [yaw, pitch, roll], degrees=False)

        # Get quaternion (format [w, x, y, z])
        q_scipy = rotation.as_quat()

        #  Adjust the order to [x, y, z, w] (consistent with ROS's Quaternion)
        quat_msg = Quaternion()
        quat_msg.x = q_scipy[1]
        quat_msg.y = q_scipy[2]
        quat_msg.z = q_scipy[3]
        quat_msg.w = q_scipy[0]
        return quat_msg

    def _visualize_points(
        self, resp: GetAgileGaze.Response, layout: List[List[float]]
    ) -> None:
        """

        Send grasp & place points to visualization service.

        Args:
            resp (GetAgileGaze.Response): AgileGaze response
            layout (List[List[float]]): palletizing layout
        """
        msg = PalletVisualizer.Request()
        for item in resp.vr_list:
            p = Pose(
                position=Point(x=item.x, y=item.y, z=self.box_l),
                orientation=self._euler_to_quaternion_msg(0, 0, item.c),
            )
            msg.grasp_poses.append(p)
        for idx, x, y, z in layout[: self._current_layout_item_index + resp.quantity]:
            p = Pose(
                position=Point(x=x, y=y, z=z + self.box_l),
                orientation=self._euler_to_quaternion_msg(0, 0, 0),
            )
            msg.place_poses.append(p)
        self.points_client.call_async(msg)
        self.get_logger().info("Points sent to visualizer.")
        self.get_logger().info(f"Grasp points: {msg.grasp_poses}")
        self.get_logger().info(f"Place points: {msg.place_poses}")

    def _register_cartesian_prs(
        self, layout_idx: int, item: Dict, layout: List[List[float]]
    ) -> None:
        """
        Register all Cartesian PRs for one pick-and-place cycle.
        NOTICE:Please adjust the pose’s Euler angles as needed.


        Args:
            layout_idx (int): Item index(layout index)
            item (Dict): Item info
            layout (List[List[float]]): palletizing layout
        """
        base_z = self.box_l
        # grasp pose.
        # Notice:Please choose the sign of the yaw angle based on the actual rotation direction.
        self.pr_grasp = self._write_cart_pr(
            id=2,
            name=f"grasp_pr_{layout_idx}",
            pos=[item.x, item.y, base_z, 180, 0, 0 - math.degrees(item.c)],
        )
        # grasp lift
        self.pr_grasp_lift = self._write_cart_pr(
            id=3,
            name=f"grasp_lift_pr_{layout_idx}",
            pos=[item.x, item.y, base_z + self.grasp_lift, 180, 0, 0],
        )
        # place
        x_p, y_p, z_p = (
            layout[layout_idx][1],
            layout[layout_idx][2],
            layout[layout_idx][3],
        )

        self.pr_place_lift = self._write_cart_pr(
            id=4,
            name=f"place_lift_pr_{layout_idx}",
            pos=[x_p, y_p, base_z + z_p + self.grasp_lift, 180, 0, 0],
        )

        self.pr_place = self._write_cart_pr(
            id=5, name=f"place_pr_{layout_idx}", pos=[x_p, y_p, base_z + z_p, 180, 0, 0]
        )

    def _write_cart_pr(self, id: int, name: str, pos: List[float]) -> PoseRegister:
        """
        Write a Cartesian PoseRegister.

        Args:
            id (int): PoseRegister index
            name (str): PoseRegister name
            pos (List[float]): [x, y, z, a, b, c]

        Returns:
            PoseRegister: PoseRegister
        """
        pr = PoseRegister(index=id)
        pr.name = name
        pr.comment = name
        pr.poseRegisterData.pt = PoseType.CART
        pr.poseRegisterData.cartData.position.x = pos[0]
        pr.poseRegisterData.cartData.position.y = pos[1]
        pr.poseRegisterData.cartData.position.z = pos[2]
        pr.poseRegisterData.cartData.position.a = pos[3]
        pr.poseRegisterData.cartData.position.b = pos[4]
        pr.poseRegisterData.cartData.position.c = pos[5]
        pr.poseRegisterData.cartData.posture = self.posture
        assert self.arm.register.write_PR(pr), f"Write PR {name} failed"
        return pr

    def _generate_bas_script(self, index: int) -> str:
        """
        Build the BAS script content for pick-and-place.
        Args:
            index (int): Item index
        return:
            str: BAS script content
        """
        script = BasScript(name="stacking_script")
        script.set_param(ParamType.OVC, ValueType.VALUE, self.global_speed)

        if index == 0:
            start = True
        else:
            start = False
        if index == self._vision_data.quantity - 1:
            finish = True
        else:
            finish = False
        if start:
            # to Home pose.
            script.set_param(ParamType.UF_NO, ValueType.VALUE, 0)
            script.move_joint(
                pose_type=MovePoseType.PR,
                pose_index=1,
                speed_type=SpeedType.VALUE,
                speed_value=self.move_speed,
                smooth_type=SmoothType.SMOOTH_DISTANCE,
                smooth_distance=200.5,
            )
            # set tool frame
            script.set_param(ParamType.TF_NO, ValueType.VALUE, self.tf_id)
        # grasp sequence
        script.set_param(ParamType.UF_NO, ValueType.VALUE, self.grasp_uf_id)
        script.move_joint(
            pose_type=MovePoseType.PR,
            pose_index=self.pr_grasp_lift.id,
            speed_type=SpeedType.VALUE,
            speed_value=self.move_speed,
            smooth_type=SmoothType.SMOOTH_DISTANCE,
            smooth_distance=200.5,
        )
        script.move_line(
            pose_type=MovePoseType.PR,
            pose_index=self.pr_grasp.id,
            speed_type=SpeedType.VALUE,
            speed_value=self.close_speed,
            smooth_type=SmoothType.FINE,
        )
        # Grasp controlled via IO
        script.assign_value(
            self.IO_type, self.IO_id, OtherType.IO_STATUS, IOStatus.ON, opt_value=1
        )
        script.wait_time(ValueType.VALUE, self.grasp_time)
        script.move_line(
            pose_type=MovePoseType.PR,
            pose_index=self.pr_grasp_lift.id,
            speed_type=SpeedType.VALUE,
            speed_value=self.close_speed,
            smooth_type=SmoothType.FINE,
        )
        # place sequence
        script.set_param(ParamType.UF_NO, ValueType.VALUE, self.place_uf_id)

        script.move_joint(
            pose_type=MovePoseType.PR,
            pose_index=self.pr_place_lift.id,
            speed_type=SpeedType.VALUE,
            speed_value=self.move_speed,
            smooth_type=SmoothType.SMOOTH_DISTANCE,
            smooth_distance=200.5,
        )

        script.move_line(
            pose_type=MovePoseType.PR,
            pose_index=self.pr_place.id,
            speed_type=SpeedType.VALUE,
            speed_value=self.close_speed,
            smooth_type=SmoothType.FINE,
        )
        # Grasp controlled via IO
        script.assign_value(
            self.IO_type, self.IO_id, OtherType.IO_STATUS, IOStatus.OFF, opt_value=0
        )
        script.wait_time(ValueType.VALUE, self.grasp_time)
        script.move_line(
            pose_type=MovePoseType.PR,
            pose_index=self.pr_place_lift.id,
            speed_type=SpeedType.VALUE,
            speed_value=self.close_speed,
            smooth_type=SmoothType.FINE,
        )
        # return to home pose
        if finish:
            script.set_param(ParamType.UF_NO, ValueType.VALUE, 0)
            script.move_joint(
                pose_type=MovePoseType.PR,
                pose_index=1,
                speed_type=SpeedType.VALUE,
                speed_value=self.move_speed,
                smooth_type=SmoothType.SMOOTH_DISTANCE,
                smooth_distance=200.5,
            )
        script.content.extend(["  RETURN", "END"])
        return "\n".join(script.content[1:])


if __name__ == "__main__":
    main()
