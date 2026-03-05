"""RRT planner: Planner (threading) + RrtAlgorithm (joint-space path with collision avoidance)."""
from __future__ import annotations

import threading
from typing import Callable

import numpy as np

from robot_manager.core import JointState, ObstacleState, Planner
from robot_manager.utils import interpolate, quintic_time_scaling, RrtAlgorithm

class RrtPlanner(Planner):
    """Planner that runs RrtAlgorithm in the worker thread (plan → _run → generate_trajectory)."""

    def __init__(self, dt: float = 0.01, seed: int | None = None) -> None:
        super().__init__()
        self._dt = dt
        self._rrt = RrtAlgorithm(seed=seed)
        self._trajectory: list[tuple[float, JointState]] = []
        self._trajectory_mutex = threading.Lock()

    def set_collision_checker(
        self,
        config_fn: Callable[[JointState, ObstacleState], bool] | None = None,
        segment_fn: Callable[[JointState, JointState, ObstacleState], bool] | None = None,
    ) -> None:
        """Set optional collision checkers: config_fn(q, obstacle_state), segment_fn(a, b, obstacle_state)."""
        self._rrt.set_collision_checker(config_fn=config_fn, segment_fn=segment_fn)

    def set_joint_limits(
        self,
        min_positions: np.ndarray,
        max_positions: np.ndarray,
    ) -> None:
        """Set joint limits for sampling (same length as joint count)."""
        self._rrt.set_joint_limits(min_positions, max_positions)

    def generate_trajectory(
        self,
        current_state: JointState,
        target_state: JointState,
        obstacle_state: ObstacleState,
    ) -> bool:
        success, traj = self._rrt.run(current_state, target_state, obstacle_state)
        with self._trajectory_mutex:
            self._trajectory = traj
        return success

    def eval(self, progress: float, joint_command: JointState) -> None:
        if not self._is_planned:
            return
        with self._trajectory_mutex:
            traj = self._trajectory
        if not traj:
            return
        if len(traj) == 1:
            js = traj[0][1]
            joint_command.position[:] = js.position
            joint_command.velocity[:] = js.velocity
            joint_command.torque[:] = js.torque
            return

        progress = quintic_time_scaling(progress)
        i = 0
        while i + 1 < len(traj) and traj[i + 1][0] <= progress:
            i += 1

        if i + 1 >= len(traj):
            js = traj[-1][1]
            joint_command.position[:] = js.position
            joint_command.velocity[:] = js.velocity
            joint_command.torque[:] = js.torque
            return

        t0, t1 = traj[i][0], traj[i + 1][0]
        t = (progress - t0) / (t1 - t0) if (t1 - t0) > 1e-9 else 0.0
        t = max(0.0, min(1.0, t))
        interp = interpolate(traj[i][1], traj[i + 1][1], t)
        joint_command.position[:] = interp.position
        joint_command.velocity[:] = interp.velocity
        joint_command.torque[:] = interp.torque
