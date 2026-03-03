from __future__ import annotations

import yaml

from robot_manager.core.robot import (
    JointState,
    RobotConfig,
    toSchedulerType,
    toPlannerType,
)
from robot_manager.robots.little_reader import LittleReader
from robot_manager.scheduler.fsm_scheduler import FsmScheduler

class RobotManager:
    def __init__(self, config_file: str) -> None:
        self.loadConfigurations(config_file)
        self.initialize()

    def loadConfigurations(self, config_file: str) -> None:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        robot = config.get('robot')
        if robot is None or not isinstance(robot, dict):
            raise ValueError("Config must contain a 'robot' section (dict)")

        for key in ('id', 'number_of_joints', 'scheduler_type', 'planner_type'):
            if robot.get(key) is None:
                raise ValueError(f"Config 'robot' must specify '{key}'")
        st = robot.get('scheduler_type')
        pt = robot.get('planner_type')
        r_cfg = RobotConfig(
            id=robot['id'],
            number_of_joints=robot['number_of_joints'],
            controller_indexes=robot.get('controller_indexes') or [],
            scheduler_type=toSchedulerType(st),
            planner_type=toPlannerType(pt),
        )

        if robot.get('type') == 'little_reader':
            self.robot_ = LittleReader(r_cfg)
        else:
            raise ValueError(f"Invalid robot type: {robot.get('type')}")

    def initialize(self) -> None:
        self.robot_.initialize()

    def control(self) -> JointState:
        return self.robot_.control()

    def update(self, status: JointState) -> None:
        self.robot_.update(status)
    