"""Geometry utilities: pose to transformation matrix (roll-pitch-yaw)."""
import numpy as np

from robot_manager.core import Pose


def transformation_matrix(pose: Pose) -> np.ndarray:
    """Build 4x4 transformation matrix from pose (position + RPY orientation).

    Parameters
    ----------
    pose : Pose
        Dataclass with position (x, y, z) and orientation (roll, pitch, yaw) in radians.

    Returns
    -------
    np.ndarray
        4x4 homogeneous transformation matrix (rotation Rz*Ry*Rx then translation).
    """
    x, y, z = pose.position
    roll, pitch, yaw = pose.orientation

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1],
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)],
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)],
    ])
    R = Rz @ Ry @ Rx
    p = np.array([x, y, z]).reshape(3, 1)
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]