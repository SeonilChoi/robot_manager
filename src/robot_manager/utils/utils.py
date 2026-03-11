"""Geometry and time-scaling utilities for planning and interpolation."""
import numpy as np


def quintic_time_scaling(t: float) -> float:
    """
    Smooth time scaling in [0, 1] with zero velocity at start and end.

    Args:
        t: Progress in [0, 1]; clamped if out of range.

    Returns:
        Scaled progress (quintic polynomial).
    """
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def distance(a: np.ndarray, b: np.ndarray) -> float:
    """
    Euclidean distance between two vectors.

    Args:
        a: First vector (any shape; flattened).
        b: Second vector (any shape; flattened).

    Returns:
        Scalar distance.
    """
    return float(
        np.linalg.norm(
            np.asarray(a, dtype=np.float64) - np.asarray(b, dtype=np.float64)
        )
    )


def steer(
    from_vec: np.ndarray,
    toward_vec: np.ndarray,
    step_size: float,
) -> np.ndarray:
    """
    Steer from one vector toward another by at most step_size.

    Args:
        from_vec: Start vector.
        toward_vec: Target vector.
        step_size: Maximum step length.

    Returns:
        New vector at most step_size away from from_vec toward toward_vec.
    """
    from_vec = np.asarray(from_vec, dtype=np.float64)
    toward_vec = np.asarray(toward_vec, dtype=np.float64)
    d = distance(from_vec, toward_vec)
    if d <= step_size or d < 1e-9:
        return toward_vec.copy()
    scale = step_size / d
    return from_vec + scale * (toward_vec - from_vec)


def interpolate(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    """
    Linear interpolation between a and b at parameter t.

    Args:
        a: Start vector.
        b: End vector.
        t: Interpolation parameter in [0, 1]; clamped if out of range.

    Returns:
        a + t * (b - a).
    """
    t = max(0.0, min(1.0, t))
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    return a + t * (b - a)


