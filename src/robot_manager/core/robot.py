"""Abstract robot interface: config, initialize, control, update, forward/inverse kinematics."""
from __future__ import annotations

from abc import ABC, abstractmethod

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
        self._home_state: JointState | None = None
        self._goal_state: JointState | None = None
        self._world_frame: Pose | None = None
        self._current_robot_state: RobotState | None = None
        self._current_obstacle_state: ObstacleState | None = None

    @abstractmethod
    def initialize(self) -> None:
        """Initialize scheduler, planner, and any hardware or simulation."""
        ...

    @abstractmethod
    def control(self) -> JointState | None:
        """Compute next joint command from current state. Returns command or None."""
        ...

    @abstractmethod
    def update(self, status: JointState, obstacle: ObstacleState | None = None) -> None:
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
    def forward_kinematics(self, joint_state: JointState) -> RobotState:
        """Compute Cartesian robot state from joint state."""
        ...

    @abstractmethod
    def inverse_kinematics(self, robot_state: RobotState) -> JointState:
        """Compute joint state from desired Cartesian robot state."""
        ...
