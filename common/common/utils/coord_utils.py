"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
coordinate tool file, provides some commonly used coordinate operation functions.

Adapted for SDK v2.0.1.0 - Uses Coordinate and Position classes from SDK
instead of non-existent GeometryPose, Translation, Rotation, CoordinateInfo.
"""

from typing import List, Union
from Agilebot.IR.A.sdk_classes import Coordinate, Position
import numpy as np
from scipy.spatial.transform import Rotation as R


class GeometryPose:
    """
    Adapter class for backward compatibility.
    Wraps SDK Coordinate object to provide geometry pose interface.
    """

    def __init__(self, coord: Coordinate):
        """
        Initialize GeometryPose from SDK Coordinate object.

        Args:
            coord: SDK Coordinate object containing id, name, comment, and data (Position)
        """
        self._coord = coord

    @property
    def position(self) -> Position:
        """Get position (x, y, z, a, b, c)."""
        return self._coord.data

    @property
    def orientation(self):
        """Get orientation as a simple object with y, p, r attributes."""
        return _OrientationWrapper(self._coord.data)

    @property
    def coord_info(self):
        """Get coordinate info (id, name, comment)."""
        return _CoordinateInfoWrapper(self._coord)

    def to_coordinate(self) -> Coordinate:
        """Convert back to SDK Coordinate object."""
        return self._coord


class _OrientationWrapper:
    """Wrapper for orientation data from Position (a, b, c are yaw, pitch, roll in degrees)."""

    def __init__(self, position: Position):
        # Position.a, .b, .c correspond to yaw, pitch, roll in degrees
        # In robot controller, the sequence is Z-Y-X (yaw-pitch-roll)
        self.y = position.a  # yaw (rotation around Z)
        self.p = position.b  # pitch (rotation around Y)
        self.r = position.c  # roll (rotation around X)


class _CoordinateInfoWrapper:
    """Wrapper for coordinate info."""

    def __init__(self, coord: Coordinate):
        self.id = coord.id
        self.name = coord.name
        self.comment = coord.comment


def three_points_to_pose(
    p0: List[float],
    p1: List[float],
    p2: List[float],
    coord_id=0,
    name="",
    comment="",
    group_id=1,
) -> Coordinate:
    """
    Define a coordinate system using three points and calculate its pose (Euler angles).

    Args:
        p0 (array-like): First point (origin) with shape (3,)
        p1 (array-like): Second point defining X-axis direction with shape (3,)
        p2 (array-like): Third point defining plane orientation with shape (3,)
        coord_id (int): Coordinate system ID
        name (str): Name of the coordinate system
        comment (str): Additional description
        group_id (int): Group identifier (unused, kept for API compatibility)

    Returns:
        Coordinate: SDK Coordinate object containing id, name, comment, and data (Position)
                    The Position contains x, y, z, a (yaw), b (pitch), c (roll) in degrees
    """
    # 1. Convert and validate dimensions
    p0 = np.asarray(p0, float).ravel()
    p1 = np.asarray(p1, float).ravel()
    p2 = np.asarray(p2, float).ravel()

    # If input is 6D, use first 3 elements as position
    if p0.shape[0] == 6:
        p0 = p0[:3]
    if p1.shape[0] == 6:
        p1 = p1[:3]
    if p2.shape[0] == 6:
        p2 = p2[:3]

    for idx, P in enumerate((p0, p1, p2), start=0):
        if P.shape != (3,):
            raise ValueError(f"P{idx} has shape {P.shape}, but (3,) is required")

    # 2. Calculate X-axis
    x_axis = p1 - p0
    norm_x = np.linalg.norm(x_axis)
    if norm_x == 0:
        raise ValueError("P0 and P1 coincide, cannot define X-axis")
    x_axis /= norm_x

    # 3. Calculate Z-axis (normal vector)
    v = p2 - p0
    z_axis = np.cross(x_axis, v)
    norm_z = np.linalg.norm(z_axis)
    if norm_z == 0:
        raise ValueError(
            "Three points are collinear, cannot define normal vector Z-axis"
        )
    z_axis /= norm_z

    # 4. Calculate Y-axis
    y_axis = np.cross(z_axis, x_axis)

    # 5. Build rotation matrix
    rot_matrix = np.column_stack((x_axis, y_axis, z_axis))

    # 6. Extract Z-Y-X Euler angles (yaw, pitch, roll) in radians
    yaw_rad = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
    pitch_rad = np.arcsin(-rot_matrix[2, 0])
    roll_rad = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])

    # 7. Convert to degrees (robot uses degrees)
    yaw_deg = np.degrees(yaw_rad)
    pitch_deg = np.degrees(pitch_rad)
    roll_deg = np.degrees(roll_rad)

    # 8. Create Position with x, y, z, a (yaw), b (pitch), c (roll)
    position = Position(
        x=float(p0[0]),
        y=float(p0[1]),
        z=float(p0[2]),
        a=float(yaw_deg),   # yaw (rotation around Z)
        b=float(pitch_deg), # pitch (rotation around Y)
        c=float(roll_deg)   # roll (rotation around X)
    )

    # 9. Create SDK Coordinate object
    coord = Coordinate(id=coord_id, name=name, comment=comment, data=position)
    return coord


def pose_to_matrix(position: Position, orientation=None) -> np.ndarray:
    """
    Convert position and Euler angles to homogeneous transformation matrix.

    Args:
        position: Position object with x, y, z, a, b, c attributes
                  a, b, c are yaw, pitch, roll in degrees
        orientation: Not used, kept for backward compatibility. Orientation data is
                     now contained in the Position object (a, b, c fields).

    Returns:
        np.ndarray: 4x4 homogeneous transformation matrix
    """
    # Note: In Agilebot robot controller, the sequence of rotation is Z-Y-X.
    # Position.a, .b, .c are yaw, pitch, roll in degrees
    r = R.from_euler("zyx", [position.a, position.b, position.c], degrees=True)
    T = np.eye(4)
    T[:3, :3] = r.as_matrix()
    T[:3, 3] = [position.x, position.y, position.z]
    return T


def coordinate_info_to_matrix(coord: Union[Coordinate, GeometryPose]) -> np.ndarray:
    """
    Convert Coordinate or GeometryPose object to homogeneous transformation matrix.

    Args:
        coord: SDK Coordinate object or GeometryPose wrapper

    Returns:
        np.ndarray: 4x4 homogeneous transformation matrix
    """
    # Handle both SDK Coordinate and GeometryPose wrapper
    if isinstance(coord, Coordinate):
        position = coord.data
    elif isinstance(coord, GeometryPose):
        position = coord.position
    else:
        raise TypeError(f"Expected Coordinate or GeometryPose, got {type(coord)}")

    return pose_to_matrix(position)


def transform_point_to_custom_frame(
    p_base: List[float], coord: Union[Coordinate, GeometryPose]
) -> np.ndarray:
    """
    Transform a point from base coordinate system to custom coordinate system.

    Args:
        p_base (array-like): Point in base coordinate system (x, y, z)
        coord: SDK Coordinate object or GeometryPose wrapper defining custom frame

    Returns:
        np.ndarray: Transformed point in custom coordinate system
    """
    T_custom = coordinate_info_to_matrix(coord)
    T_inv = np.linalg.inv(T_custom)
    p_base_h = np.array([*p_base, 1.0])
    p_custom_h = T_inv @ p_base_h
    return p_custom_h[:3]


def transform_point_to_base_frame(
    p_custom: List[float], coord: Union[Coordinate, GeometryPose]
) -> np.ndarray:
    """
    Transform a point from custom coordinate system to base coordinate system.

    Args:
        p_custom (array-like): Point in custom coordinate system (x, y, z)
        coord: SDK Coordinate object or GeometryPose wrapper defining custom frame

    Returns:
        np.ndarray: Transformed point in base coordinate system
    """
    T_custom = coordinate_info_to_matrix(coord)
    p_custom_h = np.array([*p_custom, 1.0])  # Homogeneous coordinates
    p_base_h = T_custom @ p_custom_h
    return p_base_h[:3]


def coord_id_exists(msg, target_id: int) -> bool:
    """
    Check if a coordinate system with specified ID exists in the message

    Args:
        msg: List of coordinate system entries (each with 'id' key/attribute)
             Can be either a list or an object with 'user_coord_summary' attribute
             List items can be either dicts with 'id' key or objects with 'id' attribute
        target_id (int): ID to search for

    Returns:
        bool: True if ID exists, False otherwise
    """
    # Handle both list input and object with user_coord_summary attribute
    if isinstance(msg, list):
        items = msg
    elif hasattr(msg, 'user_coord_summary'):
        items = msg.user_coord_summary
    else:
        return False

    # Check each item - items can be dicts or objects
    for item in items:
        # Handle dict items
        if isinstance(item, dict):
            item_id = item.get('id')
            # Convert to string for comparison since SDK returns strings
            if str(item_id) == str(target_id):
                return True
        # Handle object items with 'id' attribute
        elif hasattr(item, 'id'):
            if str(item.id) == str(target_id):
                return True

    return False
