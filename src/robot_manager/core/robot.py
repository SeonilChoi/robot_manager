from __future__ import annotations
from abc import ABC, abstractmethod

from robot_manager.core import JointState, ObstacleState, Pose, RobotConfig, RobotState
    
class Robot(ABC):
    def __init__(self, config: RobotConfig) -> None:
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
        ...

    @abstractmethod
    def control(self) -> JointState | None:
        ...

    @abstractmethod
    def update(self, status: JointState) -> None:
        ...

    @abstractmethod
    def forward_kinematics(self, joint_state: JointState) -> RobotState:
        ...

    @abstractmethod
    def inverse_kinematics(self, robot_state: RobotState) -> JointState:
        ...