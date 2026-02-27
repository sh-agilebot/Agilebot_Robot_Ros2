"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
Robot tool file, provides some commonly used robot operation functions.
"""

import math
import time
from typing import List, Optional, Tuple, Union

from Agilebot.IR.A.arm import Arm, CtrlStatusEnum, RobotStatusEnum, ServoStatusEnum
from Agilebot.IR.A.common.const import const
from Agilebot.IR.A.sdk_classes import MotionPose, Posture
from Agilebot.IR.A.sdk_types import MoveType, PoseType, TransformStatusEnum
from Agilebot.IR.A.status_code import StatusCodeEnum
from common.utils.math_utils import (
    degrees_to_radians,
    euler_to_quaternion,
    radians_to_degrees,
)
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.impl.rcutils_logger import RcutilsLogger
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState


class MethodExecutionError(Exception):
    """Raised when a call returns a non-OK StatusCodeEnum."""
    pass

def safe_call(fn, *args, prefix: str = None, **kwargs):
    """
    Safely invoke a function or method that returns a StatusCodeEnum.
    If the return code is not OK, raises MethodExecutionError.

    :param fn:      The function or method to call.
    :param args:    Positional arguments to pass to fn.
    :param prefix:  Optional error-message prefix; defaults to fn.__name__.
    :param kwargs:  Keyword arguments to pass to fn.
    :return:         The return value of fn.
    """
    ret = fn(*args, **kwargs)

    #  if the return value is a tuple and the last element is a StatusCodeEnum
    if isinstance(ret, tuple) and isinstance(ret[-1], StatusCodeEnum):
        ret_code = ret[-1]
    elif isinstance(ret, StatusCodeEnum): # if the return value is a StatusCodeEnum
        ret_code = ret
    else:  
        return ret 
    # If the code indicates failure, build and raise an exception.
    if ret_code != StatusCodeEnum.OK:
        label = prefix or fn.__name__
        msg = f"{label} error, {ret_code.name}"
        raise MethodExecutionError(msg)

    return ret


def wait_for_transform_ready(
    arm: Arm,
    transformed_file_name: str,
    logger: Optional[RcutilsLogger] = None,
    timeout_sec: float = 120.0,
    poll_sec: float = 1.0,
) -> bool:
    """
    Waits until CSV-to-trajectory conversion reaches a terminal state.

    New SDK behavior may return TRANSFORM_NOT_FOUND on subsequent checks after
    TRANSFORM_SUCCESS. Therefore, once TRANSFORM_SUCCESS is observed, treat it
    as final success and do not re-check.
    """
    start_time = time.perf_counter()
    while True:
        status, ret_code = arm.trajectory.check_transform_status(transformed_file_name)
        if ret_code != StatusCodeEnum.OK:
            if logger:
                logger.error(f"check transform status failed: {ret_code.name}")
            return False
        if status == TransformStatusEnum.TRANSFORM_SUCCESS:
            return True
        if status == TransformStatusEnum.TRANSFORM_FAILED:
            if logger:
                logger.error("trajectory transform failed")
            return False

        if time.perf_counter() - start_time >= timeout_sec:
            if logger:
                logger.error("wait transform status timeout")
            return False
        time.sleep(poll_sec)


def prepare_offline_trajectory_compat(
    arm: Arm, logger: Optional[RcutilsLogger] = None
) -> StatusCodeEnum:
    """
    Prepares offline trajectory execution with compatibility handling for newer SDK.

    - CONTROLLER_INVALID_OPERATION may be resolved by alarm reset.
    - INVALID_SEQUENCE can indicate the controller is already in a prepared state.
    """
    ret_code = arm.trajectory.prepare_offline_trajectory()
    if ret_code == StatusCodeEnum.OK:
        return ret_code

    ret_name = ret_code.name
    if ret_name == "INVALID_SEQUENCE":
        if logger:
            logger.warning(
                "prepare_offline_trajectory returned INVALID_SEQUENCE; continue as prepared"
            )
        return StatusCodeEnum.OK

    if ret_name == "CONTROLLER_INVALID_OPERATION":
        reset_ret = arm.alarm.reset()
        if reset_ret != StatusCodeEnum.OK:
            if logger:
                logger.error(f"alarm reset failed: {reset_ret.name}")
            return ret_code

        retry_ret = arm.trajectory.prepare_offline_trajectory()
        if retry_ret == StatusCodeEnum.OK:
            if logger:
                logger.info("prepare offline trajectory recovered after alarm reset")
            return retry_ret
        if retry_ret.name == "INVALID_SEQUENCE":
            if logger:
                logger.warning(
                    "prepare retry returned INVALID_SEQUENCE; continue as prepared"
                )
            return StatusCodeEnum.OK
        return retry_ret

    return ret_code

def check_robot_status(arm: Arm, logger: RcutilsLogger, verbose: bool = True) -> bool:
    """
    Checks the robot and servo status to ensure the robot is idle and ready to move.

    :return: Returns True if both the robot and servo are idle; otherwise, returns False.

    Args:
        arm (Arm): Robot arm object
        logger (RcutilsLogger): ROS2 logger object, obtained using self.get_logger()

    Returns:
        bool: Description of the return value.
    """
    try:
        ctrl_status, ret_code = arm.get_ctrl_status()
        if ret_code != StatusCodeEnum.OK:
            logger.error(f"Failed to get control status:{ret_code.name}")
            return False

        robot_status, ret_code = arm.get_robot_status()
        if ret_code != StatusCodeEnum.OK:
            logger.error(f"Failed to get robot status:{ret_code.name}")
            return False

        servo_status, ret_code = arm.get_servo_status()
        if ret_code != StatusCodeEnum.OK:
            logger.error(f"Failed to get servo status:{ret_code.name}")
            return False

        # check if the robot is idle and ready to move
        if (
            robot_status == RobotStatusEnum.ROBOT_IDLE
            and servo_status == ServoStatusEnum.SERVO_IDLE
            and ctrl_status == CtrlStatusEnum.CTRL_ENGAGED
        ):
            return True

        # Record the current status
        if verbose:
            logger.info("Robot is not ready")
            logger.info(
                f"Robot status: {robot_status}, Servo status: {servo_status}, Control status: {ctrl_status}"
            )
        return False
    except Exception as e:
        logger.error(f"Error occurred while checking robot status: {e}")
        return False


def wait_for_robot_ready(
    arm: Arm, logger: RcutilsLogger, max_wait_time: int = 30
) -> bool:
    """
    Waits for the robot to be ready.
    Args:
        arm (Arm): Robot arm object
        logger (RcutilsLogger): Logger object
        max_wait_time (int): Maximum wait time in seconds
    Returns:
        bool: Returns True if the robot is ready; otherwise, returns False.
    """

    start_time = time.perf_counter()
    while not check_robot_status(arm, logger):
        elapsed_time = time.perf_counter() - start_time
        if elapsed_time >= max_wait_time:
            # logger.info("Timed out waiting for robot to be ready.")
            return False
        time.sleep(0.1)

    # logger.info("Robot is ready to move.")
    return True


def move_to_joint(
    joints: List[float], arm: Arm, logger: RcutilsLogger
) -> StatusCodeEnum:
    """
    Moves the robot to the specified joint angles.
    Args:
        joints (List[float]): List of target joint angles in radians
        arm (Arm): Robot arm object
        logger (RcutilsLogger): Logger object
    Returns:
        StatusCodeEnum: Status code
    """

    # wait for the robot to be ready
    if not wait_for_robot_ready(arm, logger):
        return StatusCodeEnum.SERVER_ERR

    # Convert radians to degrees
    motion_pose = MotionPose()
    motion_pose.pt = PoseType.JOINT
    joints = radians_to_degrees(joints)

    # Set the joint angles
    for i, joint_angle in enumerate(joints):
        setattr(motion_pose.joint, f"j{i+1}", joint_angle)

    ret_code = arm.motion.move_to_pose(motion_pose, const.MOVE_JOINT)
    if ret_code != StatusCodeEnum.OK:
        # logger.error(f"Move to pose failed.Error:{ret_code}")
        return ret_code

    # wait for movement to complete
    if not wait_for_robot_ready(arm, logger):
        return StatusCodeEnum.SERVER_ERR

    logger.info("Move to pose success.")
    return StatusCodeEnum.OK


def move_to_pose_cart_without_posture(
    arm: Arm,
    logger: RcutilsLogger,
    x: float,
    y: float,
    z: float,
    c: float,
    b: float,
    a: float,
    vel: int = 1,
    acc: float = 1,
    timeout: int = 30,
) -> StatusCodeEnum:
    """
    Moves the robot to the specified Cartesian coordinates and sets the end effector's posture.
    Args:
        arm (Arm): Robot arm object
        logger (RcutilsLogger): Logger object
        x (float): X component of the target Cartesian coordinates
        y (float): Y component of the target Cartesian coordinates
        z (float): Z component of the target Cartesian coordinates
        c (float): C component of the target Cartesian coordinates
        b (float): B component of the target Cartesian coordinates
        a (float): A component of the target Cartesian coordinates
        vel (int, optional): Movement velocity. Defaults to 1. When the movement type is MOVE_LINE, the unit of vel is mm/s. When the movement type is MOVE_JOINT, vel is the multiple of the maximum speed, ranging from 0 to 1. Here, MOVE_JOINT is used.
        acc (float, optional): Movement acceleration. Defaults to 1.0. It represents the multiple of acceleration, ranging from 0 to 1
        posture (Posture, optional): End effector posture. Defaults to None.
        timeout (int, optional): Timeout period. Defaults to 30. Unit is seconds

    Returns:
        StatusCodeEnum: Status code
    """

    # check if the robot is ready
    if not wait_for_robot_ready(arm, logger, timeout):
        return StatusCodeEnum.SERVER_ERR

    # set the target Cartesian coordinates
    motion_pose = MotionPose()
    motion_pose.pt = PoseType.CART
    motion_pose.cartData.position.x = x
    motion_pose.cartData.position.y = y
    motion_pose.cartData.position.z = z
    motion_pose.cartData.position.c = c
    motion_pose.cartData.position.b = b
    motion_pose.cartData.position.a = a
    ret_code = arm.motion.move_to_pose(
        motion_pose, MoveType.MOVE_JOINT, vel=vel, acc=acc
    )

    if ret_code != StatusCodeEnum.OK:
        # logger.error(f"Move to pose failed.Error code:{ret_code}")
        return ret_code

    return StatusCodeEnum.OK


def move_to_pose_concrete(
    arm: Arm,
    logger: RcutilsLogger,
    motion_pose: MotionPose,
    velocity: float = 0.5,
    motion_type: int = const.MOVE_JOINT,
    tool_id: int = 0,
    frame_id: int = 0,
) -> StatusCodeEnum:
    """
    Move the robot to a specified position

    Args:
        arm (Arm): Robot arm object
        logger (RcutilsLogger): Logger object
        motion_pose (MotionPose): Motion posture object
        velocity (float, optional): Movement speed. Defaults to 0.5.
        motion_type (int, optional): Motion type. Defaults to const.MOVE_JOINT.
        tool_id (int, optional): Tool coordinate system ID. Defaults to 0.
        frame_id (int, optional): User coordinate system ID. Defaults to 0.

    Returns:
        StatusCodeEnum: Return status code
    """

    # Check robot status
    if not wait_for_robot_ready(arm, logger):
        return StatusCodeEnum.SERVER_ERR

    # Set robot motion posture, currently manually specified, need to automatically read robot motion posture later
    # TODO: Need to automatically read robot motion posture in the future
    motion_pose.cartData.posture = Posture()
    motion_pose.cartData.posture.wrist_flip = 1
    motion_pose.cartData.posture.arm_up_down = 1
    motion_pose.cartData.posture.arm_back_front = 1
    motion_pose.cartData.posture.arm_left_right = 0

    # Generate motion instruction with all parameters (SDK 2.0+ API)
    motion_instruction = motion_pose.generate_motion_instruction(
        motion_type=motion_type,
        vel=velocity,
        tool_id=tool_id,
        frame_id=frame_id
    )

    # Move the robot
    ret_code = arm.motion.move_to_pose_concrete(motion_instruction)

    if ret_code != StatusCodeEnum.OK:
        logger.error(f"Failed to move to pose. Error code:{ret_code}")
        return ret_code

    # Wait for robot motion completion
    if not wait_for_robot_ready(arm, logger):
        logger.error("Moving to pose timed out.")
        return StatusCodeEnum.SERVER_ERR

    logger.info("Successfully moved to pose.")
    return StatusCodeEnum.OK


def get_current_pose(
    arm: Arm,
    stamp: str,
    pose_type=const.JOINT,
    user_coordinate_id: int = -1,
    tool_coordinate_id: int = -1,
) -> Tuple[Optional[Union[JointState, PoseStamped]], StatusCodeEnum]:
    """
    Get current robot pose

    Args:
        arm (Arm): Robot arm object
        stamp (str): Timestamp
        pose_type (str, optional): Type of pose. Defaults to const.JOINT.
        user_coordinate_id (int, optional): User coordinate system ID. Defaults to -1.
        tool_coordinate_id (int, optional): Tool coordinate system ID. Defaults to -1.

    Returns:
        Optional[Union[JointState, PoseStamped]]: Current pose
        StatusCodeEnum: Status code
    """

    if pose_type == const.JOINT:
        # Get current joint pose
        motion_pose, ret_code = arm.motion.get_current_pose(const.JOINT)

        if ret_code != StatusCodeEnum.OK:
            return None, ret_code

        # Create joint state message
        joint_states = JointState()
        # Set timestamp
        joint_states.header.stamp = stamp
        # Set joint names
        joint_states.name = [f"joint{i+1}" for i in range(6)]

        # Convert degrees to radians
        joint_states.position = degrees_to_radians(
            [
                motion_pose.joint.j1,
                motion_pose.joint.j2,
                motion_pose.joint.j3,
                motion_pose.joint.j4,
                motion_pose.joint.j5,
                motion_pose.joint.j6,
            ]
        )

        return joint_states, ret_code

    elif pose_type == const.CART:
        # Get current Cartesian pose
        motion_pose, ret_code = arm.motion.get_current_pose(
            PoseType.CART, user_coordinate_id, tool_coordinate_id
        )

        if ret_code != StatusCodeEnum.OK:
            return None, ret_code

        # Extract Euler angles (in degrees)
        angles = [
            motion_pose.cartData.position.a,
            motion_pose.cartData.position.b,
            motion_pose.cartData.position.c,
        ]
        # Convert degrees to radians
        angles = degrees_to_radians(angles)
        # Convert radians to quaternion
        quaternion = euler_to_quaternion(*angles)

        # Create stamped pose message
        pose_stamp = PoseStamped()
        # Set timestamp
        pose_stamp.header.stamp = stamp
        # Set position (convert to Python float for SDK 2.0+ compatibility)
        pose_stamp.pose.position.x = float(motion_pose.cartData.position.x)
        pose_stamp.pose.position.y = float(motion_pose.cartData.position.y)
        pose_stamp.pose.position.z = float(motion_pose.cartData.position.z)
        # Set orientation as quaternion (convert to Python float)
        pose_stamp.pose.orientation.x = float(quaternion[0])
        pose_stamp.pose.orientation.y = float(quaternion[1])
        pose_stamp.pose.orientation.z = float(quaternion[2])
        pose_stamp.pose.orientation.w = float(quaternion[3])

        return pose_stamp, ret_code

    else:
        return None, StatusCodeEnum.UNSUPPORTED_TRATYPE


def pose_to_xyz_rpy_deg(pose: Pose) -> dict:
    """
    Convert geometry_msgs.msg.Pose to a dictionary containing xyz position and Euler angles in degrees.

    Args:
        pose (Pose): ROS Pose message containing position and orientation.

    Returns:
        dict: {
            'x': float, 'y': float, 'z': float,
            'roll': float, 'pitch': float, 'yaw': float
        }
    """
    # Extract translation
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    # Extract quaternion and convert to list
    q = pose.orientation
    quat = [q.x, q.y, q.z, q.w]

    #  Adjust quaternion order: [w, x, y, z] (scipy requires this order)
    quat_scipy = [quat[3], quat[0], quat[1], quat[2]]

    # Create Rotation object
    rotation = R.from_quat(quat_scipy)

    # Convert to Euler angles (in radians), rotation order is XYZ (Roll-Pitch-Yaw)
    roll_rad, pitch_rad, yaw_rad = rotation.as_euler("xyz", degrees=False)

    # Convert radians to degrees
    rad2deg = 180.0 / math.pi
    roll = roll_rad * rad2deg
    pitch = pitch_rad * rad2deg
    yaw = yaw_rad * rad2deg

    return {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}
