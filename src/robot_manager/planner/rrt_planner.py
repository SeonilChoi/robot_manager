"""RRT planner: Planner (threading) + RrtAlgorithm; uses np.ndarray only."""
from __future__ import annotations

import threading
from typing import Any, Callable

import numpy as np

from robot_manager.core import Planner
from robot_manager.utils import (
    interpolate,
    quintic_time_scaling,
    RrtAlgorithm,
)

CollisionFn = Callable[[np.ndarray, Any], bool]
SegmentCollisionFn = Callable[[np.ndarray, np.ndarray, Any], bool]


class RrtPlanner(Planner):
    """Planner that runs RrtAlgorithm in a worker thread; trajectory is np.ndarray only."""

    def __init__(self, dt: float = 0.01, seed: int | None = None) -> None:
        super().__init__()
        self._dt = dt
        self._rrt = RrtAlgorithm(seed=seed)
        self._trajectory: list[tuple[float, np.ndarray]] = []
        self._trajectory_mutex = threading.Lock()
        self._collision_fn: CollisionFn | None = None
        self._segment_collision: SegmentCollisionFn | None = None

    def set_collision_checker(
        self,
        collision_fn: CollisionFn | None = None,
        segment_fn: SegmentCollisionFn | None = None,
    ) -> None:
        """Set collision checkers: collision_fn(q, obstacle_state), segment_fn(a, b, obstacle_state)."""
        self._collision_fn = collision_fn
        self._segment_collision = segment_fn

    def set_bounds(
        self,
        min_bounds: np.ndarray,
        max_bounds: np.ndarray,
    ) -> None:
        """Set sampling bounds (any dimension)."""
        self._rrt.set_bounds(min_bounds, max_bounds)

    def set_joint_limits(
        self,
        min_positions: np.ndarray,
        max_positions: np.ndarray,
    ) -> None:
        """Convenience: set bounds for joint position space."""
        self._rrt.set_joint_limits(min_positions, max_positions)

    def reset(self) -> None:
        """Reset planner and clear trajectory so is_planned is False and path is gone."""
        super().reset()
        with self._trajectory_mutex:
            self._trajectory.clear()

    def generate_trajectory(
        self,
        current_state: np.ndarray,
        target_state: np.ndarray,
        obstacle_state: Any = None,
    ) -> bool:
        """Generate trajectory; current_state and target_state must be np.ndarray."""
        start = np.asarray(current_state, dtype=np.float64).ravel().copy()
        goal = np.asarray(target_state, dtype=np.float64).ravel().copy()

        if self._collision_fn is not None or self._segment_collision is not None:
            self._rrt.set_collision_checker(
                collision_fn=self._collision_fn,
                segment_fn=self._segment_collision,
            )
        else:
            self._rrt.set_collision_checker(collision_fn=None, segment_fn=None)

        success, traj = self._rrt.run(start, goal, obstacle_state)
        with self._trajectory_mutex:
            self._trajectory = [(t, np.asarray(c).copy() if not isinstance(c, np.ndarray) else c.copy()) for t, c in traj]
        self._is_planned = success
        return success

    def get_trajectory(self) -> list[tuple[float, np.ndarray]]:
        """Return a copy of the planned trajectory [(t, q), ...]. Empty if not planned."""
        if not self._is_planned:
            return []
        with self._trajectory_mutex:
            return [(t, c.copy()) for t, c in self._trajectory]

    def _interpolate(self, progress: float) -> np.ndarray | None:
        """Return interpolated state at progress in [0, 1]. None if no trajectory or not planned."""
        if not self._is_planned:
            return None
        with self._trajectory_mutex:
            traj = self._trajectory
        if not traj:
            return None
        progress = quintic_time_scaling(progress)
        if len(traj) == 1:
            return traj[0][1].copy()
        i = 0
        while i + 1 < len(traj) and traj[i + 1][0] <= progress:
            i += 1
        if i + 1 >= len(traj):
            return traj[-1][1].copy()
        t0, t1 = traj[i][0], traj[i + 1][0]
        t = (progress - t0) / (t1 - t0) if (t1 - t0) > 1e-9 else 0.0
        t = max(0.0, min(1.0, t))
        return interpolate(traj[i][1], traj[i + 1][1], t).copy()

    def eval(self, progress: float) -> np.ndarray | None:
        """Evaluate at progress. Returns state (np.ndarray) or None."""
        return self._interpolate(progress)
