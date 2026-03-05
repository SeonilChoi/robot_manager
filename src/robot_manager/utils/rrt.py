"""RRT algorithm with collision avoidance: pure logic, no threading.

Generalized to work in any Euclidean configuration space represented as np.ndarray
(e.g. joint position, pose.position, pose.orientation, velocity.linear, torque, or any concatenation).
JointState-based helpers are provided for backward compatibility and joint-space use.
"""
from __future__ import annotations

import copy
import random
from typing import Any, Callable

import numpy as np

from robot_manager.core import JointState, ObstacleState

PI = 3.14159265358979323846
MAX_ITERATIONS = 5000
GOAL_BIAS = 0.1
STEP_SIZE = 0.1
GOAL_THRESHOLD = 0.05
INTERP_STEPS = 10


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


# ----- Generic config-space (np.ndarray) API -----


def config_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Euclidean distance between two configuration vectors.

    Parameters
    ----------
    a, b : np.ndarray
        Config vectors (any shape; flattened).

    Returns
    -------
    float
        ||a - b||_2.
    """
    return float(np.linalg.norm(np.asarray(a, dtype=np.float64) - np.asarray(b, dtype=np.float64)))


def config_steer(
    from_config: np.ndarray,
    toward_config: np.ndarray,
    step_size: float,
) -> np.ndarray:
    """Steer from one config toward another by at most step_size.

    Parameters
    ----------
    from_config : np.ndarray
        Start configuration.
    toward_config : np.ndarray
        Target configuration.
    step_size : float
        Maximum step length.

    Returns
    -------
    np.ndarray
        New configuration (toward_config if within step_size, else interpolated).
    """
    from_config = np.asarray(from_config, dtype=np.float64)
    toward_config = np.asarray(toward_config, dtype=np.float64)
    d = config_distance(from_config, toward_config)
    if d <= step_size or d < 1e-9:
        return toward_config.copy()
    scale = step_size / d
    return from_config + scale * (toward_config - from_config)


def config_interpolate(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    """Linear interpolation between configs a and b at parameter t.

    Parameters
    ----------
    a, b : np.ndarray
        Start and end configurations.
    t : float
        Parameter in [0, 1]; clamped if out of range.

    Returns
    -------
    np.ndarray
        (1 - t) * a + t * b.
    """
    t = max(0.0, min(1.0, t))
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    return a + t * (b - a)


# ----- JointState helpers (backward compatibility / joint-space) -----


def joint_distance(a: JointState, b: JointState) -> float:
    """Euclidean distance between joint positions of two JointStates."""
    return config_distance(a.position, b.position)


def copy_metadata(from_state: JointState, position: np.ndarray) -> JointState:
    """Build a new JointState with same id/velocity/torque shape and given position.

    Parameters
    ----------
    from_state : JointState
        Template for id and array shapes.
    position : np.ndarray
        New position array (same size as from_state.position).

    Returns
    -------
    JointState
        New state with velocity and torque zeroed.
    """
    return JointState(
        id=copy.deepcopy(from_state.id),
        position=np.asarray(position, dtype=np.float64),
        velocity=np.zeros_like(from_state.position),
        torque=np.zeros_like(from_state.position),
    )


def steer(
    from_state: JointState,
    toward_state: JointState,
    step_size: float,
) -> JointState:
    """Steer from_state toward toward_state by at most step_size in position space.

    Parameters
    ----------
    from_state, toward_state : JointState
        Start and target joint states.
    step_size : float
        Maximum step length in position space.

    Returns
    -------
    JointState
        New state with position updated; velocity/torque from copy_metadata.
    """
    new_pos = config_steer(from_state.position, toward_state.position, step_size)
    return copy_metadata(from_state, new_pos)


def interpolate(a: JointState, b: JointState, t: float) -> JointState:
    """Linear interpolation between two JointStates at parameter t in [0, 1] (position space).

    Parameters
    ----------
    a, b : JointState
        Start and end joint states.
    t : float
        Parameter in [0, 1].

    Returns
    -------
    JointState
        New state with interpolated position.
    """
    pos = config_interpolate(a.position, b.position, t)
    return copy_metadata(a, pos)


# ----- RRT algorithm (generic config = np.ndarray) -----

ConfigCollisionFn = Callable[[np.ndarray, Any], bool]
SegmentCollisionFn = Callable[[np.ndarray, np.ndarray, Any], bool]


class RrtAlgorithm:
    """RRT in an arbitrary Euclidean config space (np.ndarray). No threading."""

    def __init__(
        self,
        *,
        step_size: float = STEP_SIZE,
        goal_bias: float = GOAL_BIAS,
        goal_threshold: float = GOAL_THRESHOLD,
        max_iterations: int = MAX_ITERATIONS,
        interp_steps: int = INTERP_STEPS,
        seed: int | None = None,
    ) -> None:
        self._step_size = step_size
        self._goal_bias = goal_bias
        self._goal_threshold = goal_threshold
        self._max_iterations = max_iterations
        self._interp_steps = interp_steps
        self._seed = seed
        self._bounds_min: np.ndarray | None = None
        self._bounds_max: np.ndarray | None = None
        self._config_collision: ConfigCollisionFn | None = None
        self._segment_collision: SegmentCollisionFn | None = None

    def set_bounds(
        self,
        min_config: np.ndarray,
        max_config: np.ndarray,
    ) -> None:
        """Set sampling bounds for the config space (any dimension)."""
        self._bounds_min = np.asarray(min_config, dtype=np.float64)
        self._bounds_max = np.asarray(max_config, dtype=np.float64)

    def set_joint_limits(
        self,
        min_positions: np.ndarray,
        max_positions: np.ndarray,
    ) -> None:
        """Convenience: set bounds for joint position space (same as set_bounds)."""
        self.set_bounds(min_positions, max_positions)

    def set_collision_checker(
        self,
        config_fn: ConfigCollisionFn | None = None,
        segment_fn: SegmentCollisionFn | None = None,
    ) -> None:
        """Set optional collision checkers: config_fn(config, obstacle_state), segment_fn(config_a, config_b, obstacle_state)."""
        self._config_collision = config_fn
        self._segment_collision = segment_fn

    def run(
        self,
        start: np.ndarray | JointState,
        goal: np.ndarray | JointState,
        obstacle_state: Any = None,
    ) -> tuple[bool, list[tuple[float, np.ndarray]]] | tuple[bool, list[tuple[float, JointState]]]:
        """
        Run RRT in config space.
        - If start/goal are np.ndarray: returns (success, list of (t, np.ndarray)).
        - If start/goal are JointState: delegates to run_joint_space and returns (success, list of (t, JointState)).
        """
        if hasattr(start, "position") and hasattr(goal, "position"):
            return self.run_joint_space(start, goal, obstacle_state)
        start = np.asarray(start, dtype=np.float64).ravel()
        goal = np.asarray(goal, dtype=np.float64).ravel()
        n = start.size
        if n == 0 or goal.size != n:
            return False, []

        nodes: list[np.ndarray] = [start.copy()]
        parent: list[int] = [0]

        if self._bounds_min is not None and self._bounds_max is not None and self._bounds_min.size >= n:
            min_c = self._bounds_min[:n].copy()
            max_c = self._bounds_max[:n].copy()
        else:
            min_c = np.full(n, -PI)
            max_c = np.full(n, PI)

        def in_collision(q: np.ndarray) -> bool:
            if self._config_collision is not None:
                return self._config_collision(q, obstacle_state)
            return False

        def segment_collision(a: np.ndarray, b: np.ndarray) -> bool:
            if self._segment_collision is not None:
                return self._segment_collision(a, b, obstacle_state)
            for s in range(1, self._interp_steps):
                t = s / self._interp_steps
                q = config_interpolate(a, b, t)
                if in_collision(q):
                    return True
            return False

        rng = random.Random(self._seed)

        for _ in range(self._max_iterations):
            if rng.random() < self._goal_bias:
                sample = goal.copy()
            else:
                sample = min_c + (max_c - min_c) * np.array([rng.random() for _ in range(n)])

            nearest_idx = 0
            nearest_dist = float("inf")
            for i, node in enumerate(nodes):
                d = config_distance(node, sample)
                if d < nearest_dist:
                    nearest_dist = d
                    nearest_idx = i

            new_node = config_steer(nodes[nearest_idx], sample, self._step_size)

            if in_collision(new_node):
                continue
            if segment_collision(nodes[nearest_idx], new_node):
                continue

            nodes.append(new_node)
            parent.append(nearest_idx)

            if config_distance(new_node, goal) <= self._goal_threshold:
                path: list[int] = []
                idx = len(nodes) - 1
                while True:
                    path.append(idx)
                    if idx == 0:
                        break
                    idx = parent[idx]
                path.reverse()

                total_len = 0.0
                for i in range(1, len(path)):
                    total_len += config_distance(nodes[path[i - 1]], nodes[path[i]])
                if total_len < 1e-9:
                    total_len = 1.0

                traj: list[tuple[float, np.ndarray]] = []
                cum = 0.0
                for i in range(len(path)):
                    t = (cum / total_len) if i > 0 else 0.0
                    if i > 0:
                        cum += config_distance(nodes[path[i - 1]], nodes[path[i]])
                    traj.append((t, nodes[path[i]].copy()))
                traj[0] = (0.0, traj[0][1])
                traj[-1] = (1.0, traj[-1][1])
                return True, traj

        return False, []

    def run_joint_space(
        self,
        start: JointState,
        goal: JointState,
        obstacle_state: Any = None,
    ) -> tuple[bool, list[tuple[float, JointState]]]:
        """
        Run RRT in joint position space. Uses start.position / goal.position as config.
        Returns (success, trajectory) where trajectory is list of (t, JointState).
        """
        success, traj = self.run(start.position, goal.position, obstacle_state)
        if not success:
            return False, []
        out: list[tuple[float, JointState]] = []
        for t, config in traj:
            out.append((t, copy_metadata(start, config)))
        return True, out
