"""RRT planner: Planner (threading) + RrtAlgorithm in arbitrary config space (np.ndarray).

Supports joint_state.position, pose.position, velocity.linear, or any np.ndarray config space.
"""
from __future__ import annotations

import threading
from typing import Any, Callable

import numpy as np

from robot_manager.core import JointState, ObstacleState, Planner
from robot_manager.utils import (
    config_interpolate,
    copy_metadata,
    quintic_time_scaling,
    RrtAlgorithm,
)

ConfigCollisionFn = Callable[[np.ndarray, Any], bool]
SegmentCollisionFn = Callable[[np.ndarray, np.ndarray, Any], bool]


class RrtPlanner(Planner):
    """Planner that runs RrtAlgorithm in a worker thread. Works in any config space (np.ndarray)."""

    def __init__(self, dt: float = 0.01, seed: int | None = None) -> None:
        super().__init__()
        self._dt = dt
        self._rrt = RrtAlgorithm(seed=seed)
        self._trajectory: list[tuple[float, np.ndarray]] = []
        self._trajectory_mutex = threading.Lock()
        # Joint-space collision (JointState) for backward compat
        self._joint_space_config_fn: Callable[[JointState, Any], bool] | None = None
        self._joint_space_segment_fn: Callable[[JointState, JointState, Any], bool] | None = None
        # Generic config-space collision (np.ndarray)
        self._config_collision: ConfigCollisionFn | None = None
        self._segment_collision: SegmentCollisionFn | None = None

    def set_collision_checker(
        self,
        config_fn: Callable[[JointState, Any], bool] | None = None,
        segment_fn: Callable[[JointState, JointState, Any], bool] | None = None,
    ) -> None:
        """Set collision checkers in joint space (backward compat): config_fn(q, obstacle_state), segment_fn(a, b, obstacle_state)."""
        self._joint_space_config_fn = config_fn
        self._joint_space_segment_fn = segment_fn
        self._config_collision = None
        self._segment_collision = None

    def set_collision_checker_config(
        self,
        config_fn: ConfigCollisionFn | None = None,
        segment_fn: SegmentCollisionFn | None = None,
    ) -> None:
        """Set collision checkers in config space: config_fn(config, obstacle_state), segment_fn(config_a, config_b, obstacle_state)."""
        self._config_collision = config_fn
        self._segment_collision = segment_fn
        self._joint_space_config_fn = None
        self._joint_space_segment_fn = None

    def set_bounds(
        self,
        min_config: np.ndarray,
        max_config: np.ndarray,
    ) -> None:
        """Set sampling bounds for config space (any dimension)."""
        self._rrt.set_bounds(min_config, max_config)

    def set_joint_limits(
        self,
        min_positions: np.ndarray,
        max_positions: np.ndarray,
    ) -> None:
        """Convenience: set bounds for joint position space."""
        self._rrt.set_joint_limits(min_positions, max_positions)

    def generate_trajectory(
        self,
        current_state: JointState | np.ndarray,
        target_state: JointState | np.ndarray,
        obstacle_state: ObstacleState | Any = None,
    ) -> bool:
        """Generate trajectory. start/goal can be np.ndarray (any config) or JointState (uses .position)."""
        start_config: np.ndarray
        goal_config: np.ndarray
        template_joint_state: JointState | None = None

        if hasattr(current_state, "position") and hasattr(target_state, "position"):
            # JointState
            start_config = np.asarray(current_state.position, dtype=np.float64).ravel()
            goal_config = np.asarray(target_state.position, dtype=np.float64).ravel()
            template_joint_state = current_state
        else:
            start_config = np.asarray(current_state, dtype=np.float64).ravel()
            goal_config = np.asarray(target_state, dtype=np.float64).ravel()

        # Apply collision checkers to RRT
        if self._config_collision is not None or self._segment_collision is not None:
            self._rrt.set_collision_checker(
                config_fn=self._config_collision,
                segment_fn=self._segment_collision,
            )
        elif self._joint_space_config_fn is not None or self._joint_space_segment_fn is not None:
            if template_joint_state is None:
                raise TypeError("Joint-space collision checkers require JointState start/goal.")
            def config_collision(config: np.ndarray, obs: Any) -> bool:
                if self._joint_space_config_fn is None:
                    return False
                return self._joint_space_config_fn(copy_metadata(template_joint_state, config), obs)

            def segment_collision(a: np.ndarray, b: np.ndarray, obs: Any) -> bool:
                if self._joint_space_segment_fn is None:
                    return False
                return self._joint_space_segment_fn(
                    copy_metadata(template_joint_state, a),
                    copy_metadata(template_joint_state, b),
                    obs,
                )
            self._rrt.set_collision_checker(config_fn=config_collision, segment_fn=segment_collision)
        else:
            self._rrt.set_collision_checker(config_fn=None, segment_fn=None)

        success, traj = self._rrt.run(start_config, goal_config, obstacle_state)
        with self._trajectory_mutex:
            self._trajectory = [(t, c.copy()) for t, c in traj]
        self._is_planned = success
        return success

    def eval_config(self, progress: float) -> np.ndarray | None:
        """Return interpolated config at progress in [0, 1]. None if no trajectory or not planned."""
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
        return config_interpolate(traj[i][1], traj[i + 1][1], t)

    def eval(self, progress: float, joint_command: JointState) -> None:
        """Evaluate at progress and write into joint_command.position (when config dim matches)."""
        config = self.eval_config(progress)
        if config is None:
            return
        config = np.asarray(config).ravel()
        n = min(config.size, joint_command.position.size)
        if n > 0:
            joint_command.position[:n] = config[:n]
            joint_command.velocity[:n] = 0.0
            joint_command.torque[:n] = 0.0
        if config.size < joint_command.position.size:
            joint_command.velocity[config.size:] = 0.0
            joint_command.torque[config.size:] = 0.0
