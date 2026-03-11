"""Core dataclasses: FSM state/action, joint/pose/wrench, robot config."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

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
class SphereObstacleState:
    """Obstacle representation: center position and radius (sphere/circle)."""
    id: int
    position: np.ndarray
    radius: float

@dataclass
class CircleObstacleState:
    """Obstacle representation: center position and radius (circle)."""
    id: int
    position: np.ndarray
    radius: float
    axis: int # if axis == 2 then it is a circle on the xy plane

@dataclass
class SelfObstacleState:
    """Self obstacle representation: center position and radius (self)."""
    id: int
    position: np.ndarray
    radius: float
    neighbor_id: List[int]

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
    scheduler_type: str
    planner_type: str
    robot_type: str
