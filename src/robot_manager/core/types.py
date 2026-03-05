"""Core dataclasses and enums: scheduler/planner/robot types, FSM state/action, joint/pose/wrench."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List

import numpy as np


class SchedulerType(Enum):
    """Supported scheduler backends."""
    FSM = 0
    GAIT = 1


class PlannerType(Enum):
    """Supported planner backends."""
    RRT = 0
    PRM = 1


class RobotType(Enum):
    """Supported robot model types."""
    LITTLE_READER = 0


def to_scheduler_type(scheduler_type: str) -> SchedulerType:
    """Convert string to SchedulerType. Accepts 'fsm', 'gait'."""
    if scheduler_type == "fsm":
        return SchedulerType.FSM
    if scheduler_type == "gait":
        return SchedulerType.GAIT
    raise ValueError(f"Invalid scheduler type: {scheduler_type}")


def to_planner_type(planner_type: str) -> PlannerType:
    """Convert string to PlannerType. Accepts 'rrt', 'prm'."""
    if planner_type == "rrt":
        return PlannerType.RRT
    if planner_type == "prm":
        return PlannerType.PRM
    raise ValueError(f"Invalid planner type: {planner_type}")


def to_robot_type(robot_type: str) -> RobotType:
    """Convert string to RobotType. Accepts 'little_reader'."""
    if robot_type == "little_reader":
        return RobotType.LITTLE_READER
    raise ValueError(f"Invalid robot type: {robot_type}")


@dataclass
class FsmState:
    """FSM state snapshot: state id and progress in [0, 1]."""
    state: int
    progress: float


@dataclass
class FsmAction:
    """FSM action: action id and duration (e.g. seconds)."""
    action: int
    duration: float


@dataclass
class JointState:
    """Joint-space state: id, position, velocity, torque (each 1d array)."""
    id: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    torque: np.ndarray


@dataclass
class ObstacleState:
    """Obstacle representation: center position and radius (sphere/circle)."""
    position: np.ndarray
    radius: float


@dataclass
class Pose:
    """Rigid pose: position (3,) and orientation (e.g. RPY or quat)."""
    position: np.ndarray
    orientation: np.ndarray


@dataclass
class Twist:
    """Velocity twist: linear and angular (each 3d)."""
    linear: np.ndarray
    angular: np.ndarray


@dataclass
class Wrench:
    """Wrench: force and torque (each 3d)."""
    force: np.ndarray
    torque: np.ndarray


@dataclass
class RobotState:
    """Full robot state: id, joint count, pose, twist, wrench, joint_state."""
    id: int
    number_of_joints: int
    pose: Pose
    twist: Twist
    wrench: Wrench
    joint_state: JointState


@dataclass
class RobotConfig:
    """Robot configuration: id, joint count, controller indexes, scheduler/planner/robot types."""
    id: int
    number_of_joints: int
    controller_indexes: List[int]
    scheduler_type: SchedulerType
    planner_type: PlannerType
    robot_type: RobotType
