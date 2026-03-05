from __future__ import annotations
from dataclasses import dataclass
from typing import List
from enum import Enum

import numpy as np

class SchedulerType(Enum):
    FSM = 0
    GAIT = 1

class PlannerType(Enum):
    RRT = 0
    PRM = 1

class RobotType(Enum):
    LITTLE_READER = 0

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

def to_robot_type(robot_type: str) -> RobotType:
    if robot_type == 'little_reader':
        return RobotType.LITTLE_READER
    else:
        raise ValueError(f"Invalid robot type: {robot_type}")

@dataclass
class FsmState:
    state: int
    progress: float

@dataclass
class FsmAction:
    action: int
    duration: float

@dataclass
class JointState:
    id: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    torque: np.ndarray

@dataclass
class ObstacleState:
    position: np.ndarray
    radius: float

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
    robot_type: RobotType