"""RRT planner: Planner (threading) + RrtAlgorithm; trajectory is np.ndarray only."""
from __future__ import annotations

import threading
from typing import Callable, List

import numpy as np

from robot_manager.core import Planner
from robot_manager.types import (
    CircleObstacleState,
    SelfObstacleState,
    SphereObstacleState,
)
from robot_manager.utils.rrt import RrtAlgorithm
from robot_manager.utils.utils import interpolate, quintic_time_scaling

CollisionFn = Callable[
    [np.ndarray, List[SphereObstacleState | CircleObstacleState | SelfObstacleState]],
    bool,
]
SegmentCollisionFn = Callable[
    [np.ndarray, np.ndarray, List[SphereObstacleState | CircleObstacleState | SelfObstacleState]],
    bool,
]


class RrtPlanner(Planner):
    """
    Planner that runs RrtAlgorithm in the worker thread.

    Trajectory is stored as list of (progress, config). Use set_bounds and
    set_collision_checker before planning.
    """

    def __init__(self, seed: int | None = None) -> None:
        """
        Initialize planner and RRT algorithm.

        Args:
            seed: Optional random seed for reproducible sampling.
        """
        super().__init__()
        self._rrt = RrtAlgorithm(seed=seed)
        self._trajectory: List[tuple[float, np.ndarray]] = []
        self._trajectory_mutex = threading.Lock()
        self._collision_fn: CollisionFn | None = None
        self._segment_collision: SegmentCollisionFn | None = None

    def set_collision_checker(
        self,
        collision_fn: CollisionFn | None = None,
        segment_fn: SegmentCollisionFn | None = None,
    ) -> None:
        """
        Set collision checkers for the RRT.

        Args:
            collision_fn: (config, obstacle_list) -> True if in collision.
            segment_fn: (config_a, config_b, obstacle_list) -> True if segment
                intersects an obstacle.
        """
        self._collision_fn = collision_fn
        self._segment_collision = segment_fn

    def set_bounds(
        self,
        min_bounds: np.ndarray,
        max_bounds: np.ndarray,
    ) -> None:
        """
        Set sampling bounds for the configuration space.

        Args:
            min_bounds: Minimum value per dimension.
            max_bounds: Maximum value per dimension.
        """
        self._rrt.set_bounds(min_bounds, max_bounds)

    def reset(self) -> None:
        """Reset planner and clear the stored trajectory."""
        super().reset()
        with self._trajectory_mutex:
            self._trajectory.clear()

    def _generate_trajectory(
        self,
        current_state: np.ndarray,
        target_state: np.ndarray,
        obstacle_state: List[SphereObstacleState | CircleObstacleState | SelfObstacleState] | None = None,
    ) -> bool:
        """
        Generate trajectory from current to target using RRT.

        Args:
            current_state: Start configuration.
            target_state: Goal configuration.
            obstacle_state: Optional obstacles for collision checking.

        Returns:
            True if a path was found and stored.
        """
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

    def get_trajectory(self) -> List[tuple[float, np.ndarray]]:
        """
        Return a copy of the planned trajectory.

        Returns:
            List of (progress, config); empty if not planned.
        """
        if not self._is_planned:
            return []
        with self._trajectory_mutex:
            return [(t, c.copy()) for t, c in self._trajectory]

    def _interpolate(self, progress: float) -> np.ndarray | None:
        """
        Return interpolated configuration at progress in [0, 1].

        Uses quintic time scaling and linear interpolation between waypoints.

        Args:
            progress: Progress along the path in [0, 1].

        Returns:
            Configuration at that progress, or None if no trajectory.
        """
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
        """
        Evaluate the planned trajectory at a given progress.

        Args:
            progress: Progress in [0, 1].

        Returns:
            Configuration at that progress, or None if no plan.
        """
        return self._interpolate(progress)
