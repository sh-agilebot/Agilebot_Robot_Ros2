"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
math tool file, provides some commonly used math operation functions.
"""

import math
from typing import List


def radians_to_degrees(radians_list: List[float]) -> List[float]:
    """
    Convert a list of radians to degrees.

    :param radians_list: List of radians
    :return: Corresponding list of degrees
    """
    return [radian * (180 / math.pi) for radian in radians_list]


def degrees_to_radians(degrees_list: List[float]) -> List[float]:
    """
    Convert a list of degrees to radians.

    :param degrees_list: List of degrees
    :return: Corresponding list of radians
    """
    return [degree * (math.pi / 180) for degree in degrees_list]


def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion to Euler angles.

    :param x: X component of the quaternion
    :param y: Y component of the quaternion
    :param z: Z component of the quaternion
    :param w: W component of the quaternion
    :return: Euler angles [roll, pitch, yaw] (unit: radians)
    """
    # Normalize the quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    # Calculate Euler angles
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)  # Clamp t2 to [-1, 1] range
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return [roll, pitch, yaw]


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to a quaternion.

    :param roll: Roll angle (unit: radians)
    :param pitch: Pitch angle (unit: radians)
    :param yaw: Yaw angle (unit: radians)
    :return: Quaternion [x, y, z, w]
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    # Normalize the quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return [x, y, z, w]
