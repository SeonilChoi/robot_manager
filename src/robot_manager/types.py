"""Core dataclasses: FSM state/action, joint/pose/wrench, robot config."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass
class FsmState:
    """
    FSM state snapshot.

    Attributes:
        state: State identifier (e.g. enum value).
        progress: Progress along the current action in [0, 1].
    """
    state: int
    progress: float


@dataclass
class FsmAction:
    """
    FSM action descriptor.

    Attributes:
        action: Action identifier (e.g. enum value).
        duration: Duration of the action (e.g. in seconds).
    """
    action: int
    duration: float


@dataclass
class JointState:
    """
    Joint-space state (positions, velocities, torques).

    Attributes:
        id: Joint identifiers (1D array).
        position: Joint positions (1D array).
        velocity: Joint velocities (1D array).
        torque: Joint torques (1D array).
    """
    id: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    torque: np.ndarray


@dataclass
class SphereObstacleState:
    """
    Spherical obstacle in 3D space.

    Attributes:
        id: Unique obstacle identifier.
        position: Center position (3D array).
        radius: Sphere radius.
    """
    id: int
    position: np.ndarray
    radius: float


@dataclass
class CircleObstacleState:
    """
    Circular obstacle in a plane (axis-aligned disk).

    Attributes:
        id: Unique obstacle identifier.
        position: Center position (3D array).
        radius: Circle radius.
        axis: Plane normal axis (e.g. 2 for xy-plane).
    """
    id: int
    position: np.ndarray
    radius: float
    axis: int


@dataclass
class SelfObstacleState:
    """
    Self-collision obstacle (e.g. link sphere).

    Attributes:
        id: Obstacle identifier.
        position: Center position (3D array).
        radius: Sphere radius.
        neighbor_id: Neighbor link ids to exclude from collision check.
    """
    id: int
    position: np.ndarray
    radius: float
    neighbor_id: List[int]


@dataclass
class Pose:
    """
    Rigid body pose (position + orientation).

    Attributes:
        position: Position (3,) in space.
        orientation: Orientation (e.g. RPY or quaternion).
    """
    position: np.ndarray
    orientation: np.ndarray


@dataclass
class Twist:
    """
    Velocity twist (linear and angular).

    Attributes:
        linear: Linear velocity (3D).
        angular: Angular velocity (3D).
    """
    linear: np.ndarray
    angular: np.ndarray


@dataclass
class Wrench:
    """
    Wrench (force and torque).

    Attributes:
        force: Force (3D).
        torque: Torque (3D).
    """
    force: np.ndarray
    torque: np.ndarray


@dataclass
class RobotState:
    """
    Full robot state (pose, twist, wrench, joint state).

    Attributes:
        id: Robot identifier.
        number_of_joints: Number of joints.
        pose: End-effector pose.
        twist: End-effector twist.
        wrench: End-effector wrench.
        joint_state: Joint-space state.
    """
    id: int
    number_of_joints: int
    pose: Pose
    twist: Twist
    wrench: Wrench
    joint_state: JointState


@dataclass
class RobotConfig:
    """
    Robot configuration (loaded from YAML or code).

    Attributes:
        id: Robot identifier.
        number_of_joints: Number of joints.
        controller_indexes: Optional controller indices.
        scheduler_type: Scheduler type string (e.g. "fsm").
        planner_type: Planner type string (e.g. "rrt").
        robot_type: Robot model type string (e.g. "little_reader").
    """
    id: int
    number_of_joints: int
    controller_indexes: List[int]
    scheduler_type: str
    planner_type: str
    robot_type: str
