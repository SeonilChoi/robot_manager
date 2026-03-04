"""RRT planner stub; implement eval and generate_trajectory for joint-space planning."""
from __future__ import annotations
from typing import Any

from robot_manager.core.planner import JointState, Planner


class RrtPlanner(Planner):
    def __init__(self, dt: float) -> None:
        super().__init__()
        self._dt = dt

    def eval(self, progress: float, joint_command: JointState) -> None:
        pass

    def generate_trajectory(
        self,
        current_state: JointState,
        target_state: JointState,
        obstacle_state: Any,
    ) -> bool:
        return False
