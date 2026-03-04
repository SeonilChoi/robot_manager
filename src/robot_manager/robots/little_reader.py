from __future__ import annotations

from robot_manager.core.robot import Robot, RobotConfig, JointState
from robot_manager.core.robot import SchedulerType, PlannerType
from robot_manager.scheduler.fsm_scheduler import FsmScheduler
from robot_manager.planner.rrt_planner import RrtPlanner

class LittleReader(Robot):
    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config)

    def initialize(self) -> None:
        if self._scheduler_type == SchedulerType.FSM:
            self._scheduler = FsmScheduler(0.01)
        else:
            raise ValueError("Invalid scheduler type.")

        if self._planner_type == PlannerType.RRT:
            self._planner = RrtPlanner(0.01)
        else:
            raise ValueError("Invalid planner type.")

    def control(self) -> JointState:
        pass

    def update(self, status: JointState) -> None:
        pass
