from __future__ import annotations

from robot_manager.core.robot import Robot, RobotConfig, JointState

class LittleReader(Robot):
    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config)

    def initialize(self) -> None:
        pass

    def control(self) -> JointState:
        raise NotImplementedError("LittleReader.control")

    def update(self, status: JointState) -> None:
        pass
