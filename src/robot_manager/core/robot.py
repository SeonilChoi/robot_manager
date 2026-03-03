from __future__ import annotations
from dataclasses import dataclass
from typing import List
from enum import Enum

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
    
class Robot:
    def __init__(self, config: RobotConfig) -> None:
        self.id_ = config.id
        self.number_of_joints_ = config.number_of_joints