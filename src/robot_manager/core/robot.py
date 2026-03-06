"""Abstract robot interface: config, initialize, control, update, forward/inverse kinematics."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

import numpy as np

from robot_manager.core import (
    JointState,
    ObstacleState,
    Pose,
    RobotConfig,
    RobotState,
)


class Robot(ABC):
    """Abstract base for robot models: joint control and kinematics."""

    def __init__(self, config: RobotConfig) -> None:
        """Store config and initialize internal state (home/goal, world frame, etc.)."""
        self._id = config.id
        self._number_of_joints = config.number_of_joints
        self._controller_indexes = config.controller_indexes
        self._scheduler_type = config.scheduler_type
        self._planner_type = config.planner_type
        
        self._world_frame: Pose | None = None
        
        self._current_progress = 0.0
        self._current_joint_state: JointState | None = None 
        self._current_joint_coordinates: np.ndarray | None = None
        self._current_robot_state: RobotState | None = None
        self._current_obstacles: List[ObstacleState] | None = None

        self._is_homing    = False
        self._is_moving    = False
        self._is_operating = False
        self._is_stop      = True
        
    @abstractmethod
    def initialize(self) -> None:
        """Initialize scheduler, planner, and any hardware or simulation."""
        ...

    @abstractmethod
    def control(self, status: JointState) -> JointState | None:
        """Compute next joint command from current state. Returns command or None."""
        ...

    @abstractmethod
    def update(self, status: JointState, obstacles: List[ObstacleState] | None = None) -> None:
        """Update internal state from joint feedback and optional obstacle state.

        Parameters
        ----------
        status : JointState
            Current joint position, velocity, torque.
        obstacle : ObstacleState | None
            Optional obstacle state for planning.
        """
        ...

    @abstractmethod
    def home(self) -> None:
        """Set the robot to home mode."""
        ...

    @abstractmethod
    def move(self) -> None:
        """Set the robot to moving mode."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Set the robot to stopped mode."""
        ...

    @abstractmethod
    def operating(self) -> None:
        """Set the robot to operating mode."""
        ...