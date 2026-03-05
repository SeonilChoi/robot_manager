from __future__ import annotations

import yaml

from robot_manager.core import (
    JointState,
    ObstacleState,
    RobotConfig,
    to_planner_type,
    to_robot_type,
    to_scheduler_type,
)
from robot_manager.robots.little_reader import LittleReader
from robot_manager.scheduler.fsm_scheduler import FsmScheduler

import numpy as np

class RobotManager:
    def __init__(self, config_file: str) -> None:
        self._load_configurations(config_file)
        self.initialize()

    def _load_configurations(self, config_file: str) -> None:
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
            scheduler_type=to_scheduler_type(st),
            planner_type=to_planner_type(pt),
            robot_type=to_robot_type(robot.get('type') or 'little_reader'),
        )

        if robot.get('type') == 'little_reader':
            self._robot = LittleReader(r_cfg)
        else:
            raise ValueError(f"Invalid robot type: {robot.get('type')}")

    def initialize(self) -> None:
        self._robot.initialize()

    def control(self) -> JointState | None:
        return self._robot.control()

    def update(self, status: JointState, obstacle: ObstacleState) -> None:
        self._robot.update(status, obstacle)

    def home(self) -> JointState | None:
        pass

    def stop(self) -> None:
        pass

    def move(self) -> None:
        pass
    