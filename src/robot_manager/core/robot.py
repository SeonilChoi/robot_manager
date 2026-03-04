from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import List

import numpy as np

from robot_manager.core.scheduler import Scheduler
from robot_manager.core.planner import Planner, JointState

class SchedulerType(Enum):
    FSM = 0
    GAIT = 1

class PlannerType(Enum):
    RRT = 0
    PRM = 1

def to_scheduler_type(scheduler_type: str) -> SchedulerType:
    if scheduler_type == 'fsm':
        return SchedulerType.FSM
    elif scheduler_type == 'gait':
        return SchedulerType.GAIT
    else:
        raise ValueError(f"Invalid scheduler type: {scheduler_type}")

def to_planner_type(planner_type: str) -> PlannerType:
    if planner_type == 'rrt':
        return PlannerType.RRT
    elif planner_type == 'prm':
        return PlannerType.PRM
    else:
        raise ValueError(f"Invalid planner type: {planner_type}")

@dataclass
class Pose:
    position: np.ndarray
    orientation: np.ndarray

@dataclass
class Twist:
    linear: np.ndarray
    angular: np.ndarray

@dataclass
class Wrench:
    force: np.ndarray
    torque: np.ndarray

@dataclass
class RobotState:
    id: int
    number_of_joints: int
    pose: Pose
    twist: Twist
    wrench: Wrench
    joint_state: JointState

@dataclass
class RobotConfig:
    id: int
    number_of_joints: int
    controller_indexes: List[int]
    scheduler_type: SchedulerType
    planner_type: PlannerType
    
class Robot(ABC):
    def __init__(self, config: RobotConfig) -> None:
        self._id = config.id
        self._number_of_joints = config.number_of_joints
        self._controller_indexes = config.controller_indexes
        self._scheduler_type = config.scheduler_type
        self._planner_type = config.planner_type

        self._home_state: JointState | None = None
        self._goal_state: JointState | None = None

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