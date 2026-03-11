"""Abstract robot interface: config, initialize, control, update, and mode hooks."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, List

import numpy as np

from robot_manager.types import (
    CircleObstacleState,
    JointState,
    RobotConfig,
    SelfObstacleState,
    SphereObstacleState,
)

ObstacleList = List[SphereObstacleState | CircleObstacleState | SelfObstacleState]


class Robot(ABC):
    """
    Abstract base for robot models.

    Subclasses must implement initialize, control, update, and the mode hooks
    (_home, _move, _stop, _auto).
    """

    def __init__(self, config: RobotConfig) -> None:
        """
        Store config and initialize internal state.

        Args:
            config: Robot configuration (id, joint count, scheduler/planner types).
        """
        self._id = config.id
        self._number_of_joints = config.number_of_joints
        self._controller_indexes = config.controller_indexes
        self._scheduler_type = config.scheduler_type
        self._planner_type = config.planner_type

        self._current_joint_state: JointState | None = None
        self._current_joint_coordinates: np.ndarray | None = None
        self._current_obstacles: ObstacleList | None = None
        self._current_progress = 0.0
        self._configuration_space_bounds: Dict[str, np.ndarray] = {}

        self._is_homing = False
        self._is_moving = False
        self._is_auto = False
        self._is_stop = True

    @abstractmethod
    def initialize(self) -> None:
        """
        Initialize scheduler, planner, and any hardware or simulation.

        Called once after construction. Must create scheduler and planner
        and set collision checker if needed.
        """
        ...

    @abstractmethod
    def control(self, status: JointState) -> JointState | None:
        """
        Compute the next joint command from current state.

        Args:
            status: Current joint feedback (position, velocity, torque).

        Returns:
            Next joint command, or None if no command is available.
        """
        ...

    @abstractmethod
    def update(
        self,
        status: JointState,
        obstacles: List[SphereObstacleState | CircleObstacleState] | None = None,
    ) -> None:
        """
        Update internal state from joint feedback and optional obstacles.

        Args:
            status: Current joint position, velocity, torque.
            obstacles: Optional list of obstacles for planning; may be merged
                into internal obstacle list (e.g. by id).
        """
        ...

    @abstractmethod
    def _home(self) -> None:
        """Run one step of homing (e.g. tick scheduler, request plan)."""
        ...

    @abstractmethod
    def _move(self) -> None:
        """Run one step of move mode."""
        ...

    @abstractmethod
    def _stop(self) -> None:
        """Run one step of stop (e.g. reset planner and scheduler)."""
        ...

    @abstractmethod
    def _auto(self) -> None:
        """Run one step of auto/operating mode."""
        ...