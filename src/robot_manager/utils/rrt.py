"""RRT algorithm with collision avoidance: pure logic, no threading."""
from __future__ import annotations

import copy
import random
from typing import Callable

import numpy as np

from robot_manager.core import JointState, ObstacleState

PI = 3.14159265358979323846
MAX_ITERATIONS = 5000
GOAL_BIAS = 0.1
STEP_SIZE = 0.1
GOAL_THRESHOLD = 0.05
INTERP_STEPS = 10


def quintic_time_scaling(t: float) -> float:
    """Smooth time scaling so velocity is zero at start and end."""
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def joint_distance(a: JointState, b: JointState) -> float:
    """Euclidean distance between joint positions."""
    return float(np.linalg.norm(a.position - b.position))


def copy_metadata(from_state: JointState, position: np.ndarray) -> JointState:
    """New JointState with same id/velocity/torque shape, given position."""
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
    """Steer from_state toward toward_state by at most step_size."""
    d = joint_distance(from_state, toward_state)
    if d <= step_size or d < 1e-9:
        return copy_metadata(from_state, toward_state.position)
    scale = step_size / d
    pos = from_state.position + scale * (toward_state.position - from_state.position)
    return copy_metadata(from_state, pos)


def interpolate(a: JointState, b: JointState, t: float) -> JointState:
    """Linear interpolation between a and b at parameter t in [0,1]."""
    t = max(0.0, min(1.0, t))
    pos = a.position + t * (b.position - a.position)
    return copy_metadata(a, pos)


class RrtAlgorithm:
    """RRT algorithm with collision avoidance. No threading; call run() with start, goal, obstacle_state."""

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
        self._joint_min: np.ndarray | None = None
        self._joint_max: np.ndarray | None = None
        self._config_collision: Callable[[JointState, ObstacleState], bool] | None = None
        self._segment_collision: Callable[[JointState, JointState, ObstacleState], bool] | None = None

    def set_joint_limits(
        self,
        min_positions: np.ndarray,
        max_positions: np.ndarray,
    ) -> None:
        self._joint_min = np.asarray(min_positions, dtype=np.float64)
        self._joint_max = np.asarray(max_positions, dtype=np.float64)

    def set_collision_checker(
        self,
        config_fn: Callable[[JointState, ObstacleState], bool] | None = None,
        segment_fn: Callable[[JointState, JointState, ObstacleState], bool] | None = None,
    ) -> None:
        self._config_collision = config_fn
        self._segment_collision = segment_fn

    def run(
        self,
        start: JointState,
        goal: JointState,
        obstacle_state: ObstacleState,
    ) -> tuple[bool, list[tuple[float, JointState]]]:
        """
        Run RRT from start to goal. Returns (success, trajectory).
        Trajectory is list of (t, JointState) with t in [0, 1].
        """
        n = start.position.size
        if n == 0 or goal.position.size != n:
            return False, []

        nodes: list[JointState] = [start]
        parent: list[int] = [0]

        if self._joint_min is not None and self._joint_max is not None and self._joint_min.size >= n:
            min_pos = self._joint_min[:n].copy()
            max_pos = self._joint_max[:n].copy()
        else:
            min_pos = np.full(n, -PI)
            max_pos = np.full(n, PI)

        def in_collision(q: JointState) -> bool:
            if self._config_collision is not None:
                return self._config_collision(q, obstacle_state)
            return False

        def segment_collision(a: JointState, b: JointState) -> bool:
            if self._segment_collision is not None:
                return self._segment_collision(a, b, obstacle_state)
            for s in range(1, self._interp_steps):
                t = s / self._interp_steps
                interp_q = interpolate(a, b, t)
                if in_collision(interp_q):
                    return True
            return False

        rng = random.Random(self._seed)

        for _ in range(self._max_iterations):
            if rng.random() < self._goal_bias:
                sample = copy_metadata(start, goal.position)
            else:
                pos = min_pos + (max_pos - min_pos) * np.array([rng.random() for _ in range(n)])
                sample = copy_metadata(start, pos)

            nearest_idx = 0
            nearest_dist = float("inf")
            for i, node in enumerate(nodes):
                d = joint_distance(node, sample)
                if d < nearest_dist:
                    nearest_dist = d
                    nearest_idx = i

            new_node = steer(nodes[nearest_idx], sample, self._step_size)

            if in_collision(new_node):
                continue
            if segment_collision(nodes[nearest_idx], new_node):
                continue

            nodes.append(new_node)
            parent.append(nearest_idx)

            if joint_distance(new_node, goal) <= self._goal_threshold:
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
                    total_len += joint_distance(nodes[path[i - 1]], nodes[path[i]])
                if total_len < 1e-9:
                    total_len = 1.0

                traj: list[tuple[float, JointState]] = []
                cum = 0.0
                for i in range(len(path)):
                    t = (cum / total_len) if i > 0 else 0.0
                    if i > 0:
                        cum += joint_distance(nodes[path[i - 1]], nodes[path[i]])
                    js = nodes[path[i]]
                    traj.append((t, js))
                traj[0] = (0.0, traj[0][1])
                traj[-1] = (1.0, traj[-1][1])
                return True, traj

        return False, []
