"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
Accept requests from gbt_stacking to visualize the pallet grasping and placement process.
- Create a service
- Accept requests
- Get the grasp points and placement points from the request
- Generate MarkerArray based on the status and points
- Publish MarkerArray
"""

import os

import rclpy
import yaml
from Agilebot.IR.A.arm import Arm
from Agilebot.IR.A.common.const import const
from Agilebot.IR.A.sdk_classes import CoordinateSystemType
from Agilebot.IR.A.sdk_types import SignalType
from Agilebot.IR.A.status_code import StatusCodeEnum
from ament_index_python.packages import get_package_share_directory
from common.utils.coord_utils import transform_point_to_base_frame
from gbt_stacking_interface.srv import PalletVisualizer
from geometry_msgs.msg import Quaternion
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class PalletVisualizerNode(Node):
    """
    A ROS2 node for visualizing pallet stacking with MarkerArray.
    """

    def __init__(self):
        super().__init__("pallet_visualizer")

        # Declare parameters
        self.declare_parameter(
            "robot_config_path",
            "config/robot_config.yaml",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            "box_size",
            [100.0, 100.0, 100.0],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )
        self.declare_parameter(
            "timer_interval",
            0.05,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        )
        self.declare_parameter(
            "pose_tolerance",
            0.5,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        )

        # Read parameter values
        config_path = (
            self.get_parameter("robot_config_path").get_parameter_value().string_value
        )
        self.box_size = (
            self.get_parameter("box_size").get_parameter_value().double_array_value
        )
        interval = (
            self.get_parameter("timer_interval").get_parameter_value().double_value
        )
        self.tolerance = (
            self.get_parameter("pose_tolerance").get_parameter_value().double_value
        )

        # Initialize state
        self.grasp_points_base = []
        self.place_points_base = []
        self.grasp_status = []
        self.place_status = []

        # Create service for pallet visualization
        self.create_service(
            PalletVisualizer,
            "gbt_stacking/pallet_visualizer",
            self._on_visualize_request,
        )

        # Load robot configuration
        pkg_share = get_package_share_directory("gbt_driver")
        full_path = os.path.join(pkg_share, config_path)
        with open(full_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        robot_ip = cfg.get("robot_ip_address", "")

        # Connect to the robot arm
        self.arm = Arm()
        status = self.arm.connect(robot_ip)
        if status != StatusCodeEnum.OK:
            self.get_logger().error(
                f"Failed to connect to {robot_ip}: {status.name}"
            )
            return

        # Retrieve user frames for grasp and place
        grasp_frame, ret1 = self.arm.coordinate_system.get(
            CoordinateSystemType.UserFrame, 1
        )
        place_frame, ret2 = self.arm.coordinate_system.get(
            CoordinateSystemType.UserFrame, 2
        )
        if ret1 != StatusCodeEnum.OK or ret2 != StatusCodeEnum.OK:
            if ret1 != StatusCodeEnum.OK:
                self.get_logger().error(
                    f"Failed to get grasp frame: {ret1.name}"
                )
            if ret2 != StatusCodeEnum.OK:
                self.get_logger().error(
                    f"Failed to get place frame: {ret2.name}"
                )
            self.get_logger().error("Unable to get user coordinate frames.")
            return
        self.grasp_frame = grasp_frame
        self.place_frame = place_frame

        # Publisher and timer
        self.publisher = self.create_publisher(MarkerArray, "/pallet/markers", 10)
        self.create_timer(interval, self._on_timer)

        self.get_logger().info("Node initialized successfully.")

    def _on_visualize_request(self, request, response):
        """
        Service callback to transform and store grasp and place points.
        """
        self.get_logger().info("Visualization request received.")

        # self._logger.info(f"Grasp poses: {request.grasp_poses}")

        # Transform grasp points to base frame
        self.grasp_points_base = [
            transform_point_to_base_frame(
                [p.position.x, p.position.y, p.position.z], self.grasp_frame
            ).tolist()
            + [p.orientation.w,p.orientation.x, p.orientation.y, p.orientation.z]
            for p in request.grasp_poses
        ]
        # Transform place points to base frame
        self.place_points_base = [
            transform_point_to_base_frame(
                [p.position.x, p.position.y, p.position.z], self.place_frame
            ).tolist()
            + [p.orientation.w,p.orientation.x, p.orientation.y, p.orientation.z, ]
            for p in request.place_poses
        ]

        self.get_logger().info(f"Grasp points: {self.grasp_points_base}")
        self.get_logger().info(f"Place points: {self.place_points_base}")

        # Initialize statuses
        self.grasp_status = [False] * len(self.grasp_points_base)
        self.place_status = [False] * len(self.place_points_base)

        response.success = True
        response.message = "Markers ready for publishing."
        return response

    def _on_timer(self):
        """
        Timer callback to publish MarkerArray based on current robot state.
        """
        if not self.grasp_points_base or not self.place_points_base:
            return

        # Get current pose
        current_pose, ret = self.arm.motion.get_current_pose(const.CART, 0, 0)
        if ret != StatusCodeEnum.OK:
            self.get_logger().error(f"Failed to get current pose: {ret.name}")
            return

        markers = MarkerArray()

        # Grasp markers
        for idx, (x, y, z, _x, _y, _z, _w) in enumerate(self.grasp_points_base):
            px, py, pz = (
                current_pose.cartData.position.x,
                current_pose.cartData.position.y,
                current_pose.cartData.position.z,
            )
            if all(
                abs(v1 - v2) < self.tolerance for v1, v2 in zip((px, py, pz), (x, y, z))
            ):
                val, code = self.arm.signals.read(signal_type=SignalType.DO, index=1)
                if code == StatusCodeEnum.OK and val == 1:
                    self.grasp_status[idx] = True

            marker = self._create_box_marker(
                idx,
                x / 1000,  # Convert mm to m
                y / 1000,  # Convert mm to m
                z / 1000 - self.box_size[2] / 2 / 1000,
                Quaternion(x=_x, y=_y, z=_z, w=_w),
                self.box_size,
                self.grasp_status[idx],
            )
            markers.markers.append(marker)

        # Place markers
        offset = len(self.grasp_points_base)
        for idx, (x, y, z, _x, _y, _z, _w) in enumerate(self.place_points_base):
            if all(
                abs(getattr(current_pose.cartData.position, axis) - coord)
                < self.tolerance
                for axis, coord in zip(("x", "y", "z"), (x, y, z))
            ):
                val, code = self.arm.signals.read(signal_type=SignalType.DO, index=1)
                if code == StatusCodeEnum.OK and val == 0:
                    self.place_status[idx] = True

            if self.place_status[idx]:
                marker = self._create_box_marker(
                    offset + idx,
                    x / 1000,
                    y / 1000,
                    z / 1000 - self.box_size[2] / 2 / 1000,
                    Quaternion(x=_x, y=_y, z=_z, w=_w),
                    self.box_size,
                    False,
                )
                markers.markers.append(marker)

        self.publisher.publish(markers)

    def _create_box_marker(
        self, marker_id, x, y, z, orientation, size, completed=False
    ):
        """
        Helper to create a box marker.
        """

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pallet"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = [s / 1000 for s in size]
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation = orientation
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.2 if completed else 1.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = PalletVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
