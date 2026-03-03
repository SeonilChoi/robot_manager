from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import List

import numpy as np

from robot_manager.core.scheduler import Scheduler
from robot_manager.core.planner import Planner

@dataclass
class JointState:
    id: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    torque: np.ndarray

class SchedulerType(Enum):
    FSM = 0
    GAIT = 1

class PlannerType(Enum):
    RRT = 0
    PRM = 1

def toSchedulerType(scheduler_type: str) -> SchedulerType:
    if scheduler_type == 'fsm':
        return SchedulerType.FSM
    elif scheduler_type == 'gait':
        return SchedulerType.GAIT
    else:
        raise ValueError(f"Invalid scheduler type: {scheduler_type}")

def toPlannerType(planner_type: str) -> PlannerType:
    if planner_type == 'rrt':
        return PlannerType.RRT
    elif planner_type == 'prm':
        return PlannerType.PRM
    else:
        raise ValueError(f"Invalid planner type: {planner_type}")

@dataclass
class RobotConfig:
    id: int
    number_of_joints: int
    controller_indexes: List[int]
    scheduler_type: SchedulerType
    planner_type: PlannerType
    
class Robot(ABC):
    def __init__(self, config: RobotConfig) -> None:
        self.id_                 = config.id
        self.number_of_joints_   = config.number_of_joints
        self.controller_indexes_ = config.controller_indexes
        self.scheduler_type_     = config.scheduler_type
        self.planner_type_       = config.planner_type

    @abstractmethod
    def initialize(self) -> None:
        ...

    @abstractmethod
    def control(self) -> JointState:
        ...

    @abstractmethod
    def update(self, status: JointState) -> None:
        ...