"""RRT algorithm with collision avoidance: pure logic, no threading.

Generalized to work in any Euclidean configuration space represented as np.ndarray
(e.g. joint position, pose.position, pose.orientation, velocity.linear, torque, or any concatenation).
JointState-based helpers are provided for backward compatibility and joint-space use.
"""
from __future__ import annotations

import copy
import random
from typing import List, Callable

import numpy as np

from robot_manager.types import JointState, SphereObstacleState, CircleObstacleState, SelfObstacleState
from robot_manager.utils.utils import distance, interpolate, steer

PI = 3.14159265358979323846
MAX_ITERATIONS = 5000
GOAL_BIAS = 0.3
STEP_SIZE = 0.08
GOAL_THRESHOLD = 0.005
INTERP_STEPS = 10

CollisionFn = Callable[[np.ndarray, List[SphereObstacleState | CircleObstacleState | SelfObstacleState]], bool]
SegmentCollisionFn = Callable[[np.ndarray, np.ndarray, List[SphereObstacleState | CircleObstacleState | SelfObstacleState]], bool]


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
        self._collision_fn: CollisionFn | None = None
        self._segment_collision_fn: SegmentCollisionFn | None = None

    def set_bounds(
        self,
        min_bounds: np.ndarray,
        max_bounds: np.ndarray,
    ) -> None:
        """Set sampling bounds (any dimension)."""
        self._bounds_min = np.asarray(min_bounds, dtype=np.float64)
        self._bounds_max = np.asarray(max_bounds, dtype=np.float64)

    def set_joint_limits(
        self,
        min_positions: np.ndarray,
        max_positions: np.ndarray,
    ) -> None:
        """Convenience: set bounds for joint position space (same as set_bounds)."""
        self.set_bounds(min_positions, max_positions)

    def set_collision_checker(
        self,
        collision_fn: CollisionFn | None = None,
        segment_fn: SegmentCollisionFn | None = None,
    ) -> None:
        """Set optional collision checkers: collision_fn(q, obstacle_state), segment_fn(a, b, obstacle_state)."""
        self._collision_fn = collision_fn
        self._segment_collision_fn = segment_fn

    def run(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacle_state: List[SphereObstacleState | CircleObstacleState | SelfObstacleState] | None = None,
    ) -> tuple[bool, list[tuple[float, np.ndarray]]]:
        """
        Run RRT in config space.
        - If start/goal are np.ndarray: returns (success, list of (t, np.ndarray)).
        - If start/goal are JointState: delegates to run_joint_space and returns (success, list of (t, JointState)).
        """
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
            if self._collision_fn is not None:
                return self._collision_fn(q, obstacle_state)
            return False

        def segment_collision(a: np.ndarray, b: np.ndarray) -> bool:
            if self._segment_collision_fn is not None:
                return self._segment_collision_fn(a, b, obstacle_state)
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
                d = distance(node, sample)
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

            if distance(new_node, goal) <= self._goal_threshold:
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
                    total_len += distance(nodes[path[i - 1]], nodes[path[i]])
                if total_len < 1e-9:
                    total_len = 1.0

                traj: list[tuple[float, np.ndarray]] = []
                cum = 0.0
                for i in range(len(path)):
                    if i == 0:
                        t = 0.0
                    else:
                        cum += distance(nodes[path[i - 1]], nodes[path[i]])
                        t = cum / total_len
                    traj.append((t, nodes[path[i]].copy()))
                traj[-1] = (1.0, traj[-1][1])
                return True, traj

        return False, []
