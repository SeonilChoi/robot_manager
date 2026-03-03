from __future__ import annotations
from dataclasses import dataclass

import yaml
import numpy as np

from robot_manager.core.robot import RobotConfig, toSchedulerType, toPlannerType

@dataclass
class JointState:
    id: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    torque: np.ndarray

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

        r_cfg = RobotConfig()
        r_cfg.id = robot.get('id')
        r_cfg.number_of_joints = robot.get('number_of_joints')
        r_cfg.controller_indexes = robot.get('controller_indexes')
        r_cfg.scheduler_type = toSchedulerType(robot.get('scheduler_type'))
        r_cfg.planner_type = toPlannerType(robot.get('planner_type'))

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
    