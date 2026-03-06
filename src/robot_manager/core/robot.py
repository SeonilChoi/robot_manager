"""Abstract robot interface: config, initialize, control, update, forward/inverse kinematics."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

import numpy as np

from robot_manager.types import JointState, SphereObstacleState, CircleObstacleState, RobotConfig


class Robot(ABC):
    """Abstract base for robot models: joint control and kinematics."""

    def __init__(self, config: RobotConfig) -> None:
        """Store config and initialize internal state (home/goal, world frame, etc.)."""
        self._id = config.id
        self._number_of_joints = config.number_of_joints
        self._controller_indexes = config.controller_indexes
        self._scheduler_type = config.scheduler_type
        self._planner_type = config.planner_type

        self._current_joint_state: JointState | None = None
        self._current_obstacles: List[SphereObstacleState | CircleObstacleState] | None = None

        self._is_homing = False
        self._is_moving = False
        self._is_auto   = False
        self._is_stop   = True
        
    @abstractmethod
    def initialize(self) -> None:
        """Initialize scheduler, planner, and any hardware or simulation."""
        ...

    @abstractmethod
    def control(self, status: JointState) -> JointState | None:
        """Compute next joint command from current state. Returns command or None."""
        ...

    @abstractmethod
    def update(self, status: JointState, obstacles: List[SphereObstacleState | CircleObstacleState] | None = None) -> None:
        """Update internal state from joint feedback and optional obstacle state.

        Parameters
        ----------
        status : JointState
            Current joint position, velocity, torque.
        obstacles : List[SphereObstacleState | CircleObstacleState] | None
            Optional obstacle state for planning.
        """
        ...

    @abstractmethod
    def _home(self) -> None:
        """Set the robot to home mode."""
        ...

    @abstractmethod
    def _move(self) -> None:
        """Set the robot to moving mode."""
        ...

    @abstractmethod
    def _stop(self) -> None:
        """Set the robot to stopped mode."""
        ...

    @abstractmethod
    def _auto(self) -> None:
        """Set the robot to auto mode."""
        ...