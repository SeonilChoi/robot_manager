"""LittleReader robot model: FSM scheduler, RRT planner, placeholder kinematics."""
from __future__ import annotations

from typing import List

import numpy as np

from robot_manager.core.robot import Robot
from robot_manager.types import JointState, RobotConfig, SphereObstacleState, CircleObstacleState
from robot_manager.scheduler.fsm_scheduler import FsmScheduler, Action, FsmAction
from robot_manager.planner.rrt_planner import RrtPlanner


class LittleReader(Robot):
    """Robot implementation with FSM scheduler and RRT planner; FK/IK stubs."""

    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config)
        
        self._wz = 0.05
        self._l1 = 0.1
        self._l2 = 0.1
        self._l3 = 0.4

        self._current_joint_state = JointState(
            id=np.arange(self._number_of_joints),
            position=np.zeros(self._number_of_joints),
            velocity=np.zeros(self._number_of_joints),
            torque=np.zeros(self._number_of_joints),
        )
        self._current_obstacles = []

        self._current_joint_coordinates = np.zeros((8, 3), dtype=np.float64)
        self._home_joint_positions = np.array([0.2, 0.5, 0, 0], dtype=np.float64)
        self._current_progress = 0.0

    def initialize(self) -> None:
        """Create FSM scheduler and RRT planner from config types."""
        if self._scheduler_type == "fsm":
            self._scheduler = FsmScheduler(0.1)
        else:
            raise ValueError("Invalid scheduler type.")

        if self._planner_type == "rrt":
            self._planner = RrtPlanner()
        else:
            raise ValueError("Invalid planner type.")

    def control(self, status) -> JointState | None:
        """Return next joint command from scheduler/planner or current state."""
        if self._is_homing:
            self._home()
        elif self._is_moving:
            self._move()
        elif self._is_auto:
            self._auto()
        elif self._is_stop:
            self._stop()

        if self._planner.is_planned():
            config = self._planner.eval(self._current_progress)
            if config is not None:
                config = np.asarray(config, dtype=np.float64).ravel().copy()
                n = min(config.size, self._number_of_joints)
                joint_command = JointState(
                    id=np.arange(self._number_of_joints),
                    position=np.zeros(self._number_of_joints),
                    velocity=np.zeros(self._number_of_joints),
                    torque=np.zeros(self._number_of_joints),
                )
                joint_command.position[:n] = config[:n]
                self._scheduler.step()
                return joint_command
        
        return self._current_joint_state
        
    def update(self, status: JointState, obstacles: List[SphereObstacleState | CircleObstacleState] | None = None) -> None:
        """Update internal state from joint feedback. Optional obstacle for planning.

        During a planned motion we do NOT overwrite _current_joint_state from status:
        it is set in control() from the command we send. So status (which may be
        from a previous cycle or a stale goal) cannot overwrite and jump to goal.
        """
        self._current_joint_state.position = status.position.copy()
        self._update_current_joint_coordinates(self._current_joint_state.position)
        if obstacles is not None:
            existing_ids = {getattr(o, "id", None) for o in self._current_obstacles if getattr(o, "id", None) is not None}
            for obs in obstacles:
                oid = getattr(obs, "id", None)
                if oid is not None and oid in existing_ids:
                    continue
                self._current_obstacles.append(obs)
                if oid is not None:
                    existing_ids.add(oid)

    def _update_current_joint_coordinates(self, joint_positions: np.ndarray) -> None:
        q1, q2, h1, h2 = joint_positions
        self._current_joint_coordinates[0] = np.array([0, 0, 0])
        self._current_joint_coordinates[1] = np.array([0, 0, self._wz])
        self._current_joint_coordinates[2] = np.array([       0, -self._l1, self._wz])
        self._current_joint_coordinates[3] = np.array([self._l2, -self._l1, self._wz])
        self._current_joint_coordinates[4] = np.array([ self._l2 + self._l3 * np.cos(q2),
                                                       -self._l1 - self._l3 * np.sin(q1) * np.sin(q2),
                                                        self._wz + self._l3 * np.cos(q1) * np.sin(q2)])
        self._current_joint_coordinates[5] = np.array([       0, self._l1, self._wz])
        self._current_joint_coordinates[6] = np.array([self._l2, self._l1, self._wz])
        self._current_joint_coordinates[7] = np.array([self._l2 + self._l3 * np.cos(h2),
                                                       self._l1 + self._l3 * np.sin(h1) * np.sin(h2),
                                                       self._wz - self._l3 * np.cos(h1) * np.sin(h2)])

    def _home(self) -> None:
        is_event, current_fsm_state = self._scheduler.tick(FsmAction(Action.HOME.value, duration=10.0))
        self._current_progress = current_fsm_state.progress

        if is_event:
            if current_fsm_state.progress >= 1.0:
                self._is_homing = False
                self._planner.reset()
                
            elif current_fsm_state.progress > 0.0 and self._current_joint_state is not None:
                self._planner.plan(
                    self._current_joint_state.position[:2].copy(),
                    self._home_joint_positions[:2].copy(),
                    self._current_obstacles
                )

    def _move(self) -> None:
        pass

    def _stop(self) -> None:
        self._scheduler.tick(FsmAction(Action.STOP.value, duration=0.0))
        self._planner.reset()
        self._scheduler.reset()
        self._current_progress = 0.0

    def _auto(self) -> None:
        pass

    def forward_kinematics(self, joint_positions: np.ndarray) -> np.ndarray:
        q1, q2, h1, h2 = joint_positions

        current_joint_coordinates = np.zeros((8, 3), dtype=np.float64)
        current_joint_coordinates[0] = np.array([0, 0, 0])
        current_joint_coordinates[1] = np.array([0, 0, self._wz])
        current_joint_coordinates[2] = np.array([       0, -self._l1, self._wz])
        current_joint_coordinates[3] = np.array([self._l2, -self._l1, self._wz])
        current_joint_coordinates[4] = np.array([ self._l2 + self._l3 * np.cos(q2),
                                                       -self._l1 - self._l3 * np.sin(q1) * np.sin(q2),
                                                        self._wz + self._l3 * np.cos(q1) * np.sin(q2)])
        current_joint_coordinates[5] = np.array([       0, self._l1, self._wz])
        current_joint_coordinates[6] = np.array([self._l2, self._l1, self._wz])
        current_joint_coordinates[7] = np.array([self._l2 + self._l3 * np.cos(h2),
                                                 self._l1 + self._l3 * np.sin(h1) * np.sin(h2),
                                                 self._wz - self._l3 * np.cos(h1) * np.sin(h2)])
        return current_joint_coordinates