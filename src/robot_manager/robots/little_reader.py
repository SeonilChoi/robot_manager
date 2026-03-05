from __future__ import annotations

import numpy as np

from robot_manager.core import (
    JointState,
    Pose,
    PlannerType,
    Robot,
    RobotConfig,
    RobotState,
    SchedulerType,
)
from robot_manager.scheduler.fsm_scheduler import FsmScheduler
from robot_manager.planner.rrt_planner import RrtPlanner

class LittleReader(Robot):
    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config)
        
        self._world_frame = Pose(
            position=np.array([0, 0, 0.05]),
            orientation=np.array([0, 0, 0]),
        )

        self._l1 = 0.1
        self._l2 = 0.1
        self._l3 = 0.4

        self.M_left = np.array([[1, 0, 0,  self._l2 + self._l3],
                                [0, 1, 0, -self._l1],
                                [0, 0, 1,  0],
                                [0, 0, 0,  1]])

        self.M_right = np.array([[1, 0, 0, self._l2 + self._l3],
                                 [0, 1, 0, self._l1],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])

        self.S_left = np.array([[1,  0, 0, 0, 0,  self._l1],
                                [0, -1, 0, 0, 0, -self._l2]])

        self.S_right = np.array([[1, 0, 0, 0, 0, -self._l1],
                                 [0, 1, 0, 0, 0,  self._l2]])

    def initialize(self) -> None:
        if self._scheduler_type == SchedulerType.FSM:
            self._scheduler = FsmScheduler(0.01)
        else:
            raise ValueError("Invalid scheduler type.")

        if self._planner_type == PlannerType.RRT:
            self._planner = RrtPlanner(0.01)
        else:
            raise ValueError("Invalid planner type.")

    def control(self) -> JointState | None:
        return JointState(
            id=np.arange(self._number_of_joints),
            position=np.random.rand(self._number_of_joints),
            velocity=np.random.rand(self._number_of_joints),
            torque=np.random.rand(self._number_of_joints),
        )

    def update(self, status: JointState) -> None:
        pass

    def forward_kinematics(self, joint_state: JointState) -> RobotState:
        pass

    def inverse_kinematics(self, robot_state: RobotState) -> JointState:
        pass

    def _fk(self, M: np.ndarray, Slist: np.ndarray, thetalist: np.ndarray) -> np.ndarray:
        pass