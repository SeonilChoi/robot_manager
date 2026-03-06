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


def quintic_time_scaling(t: float) -> float:
    """Smooth time scaling in [0, 1] with zero velocity at start and end.

    Parameters
    ----------
    t : float
        Progress in [0, 1]; clamped if out of range.

    Returns
    -------
    float
        Scaled progress (quintic polynomial).
    """
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def distance(a: np.ndarray, b: np.ndarray) -> float:
    """Euclidean distance between two vectors (any shape; flattened)."""
    return float(np.linalg.norm(np.asarray(a, dtype=np.float64) - np.asarray(b, dtype=np.float64)))


def steer(
    from_vec: np.ndarray,
    toward_vec: np.ndarray,
    step_size: float,
) -> np.ndarray:
    """Steer from one vector toward another by at most step_size."""
    from_vec = np.asarray(from_vec, dtype=np.float64)
    toward_vec = np.asarray(toward_vec, dtype=np.float64)
    d = distance(from_vec, toward_vec)
    if d <= step_size or d < 1e-9:
        return toward_vec.copy()
    scale = step_size / d
    return from_vec + scale * (toward_vec - from_vec)


def interpolate(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    """Linear interpolation between a and b at parameter t in [0, 1]."""
    t = max(0.0, min(1.0, t))
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    return a + t * (b - a)