"""LittleReader robot model: FSM scheduler, RRT planner, placeholder kinematics."""
from __future__ import annotations

from typing import List

import numpy as np

from robot_manager.types import FsmAction, JointState, ObstacleState, Pose, PlannerType, Robot, RobotConfig, SchedulerType
from robot_manager.core.scheduler import FsmScheduler
from robot_manager.planner.rrt_planner import RrtPlanner
from robot_manager.utils.utils import transformation_matrix

class LittleReader(Robot):
    """Robot implementation with FSM scheduler and RRT planner; FK/IK stubs."""

    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config)
        
        self._world_frame = Pose(
            position=np.array([0, 0, 0.05]),
            orientation=np.array([0, 0, -np.pi/2]),
        )
        self._Tws = transformation_matrix(self._world_frame)

        self._l1 = 0.1
        self._l2 = 0.1
        self._l3 = 0.4

        self._current_joint_state = JointState(
            id=np.arange(self._number_of_joints),
            position=np.zeros(self._number_of_joints),
            velocity=np.zeros(self._number_of_joints),
            torque=np.zeros(self._number_of_joints),
        )

    def initialize(self) -> None:
        """Create FSM scheduler and RRT planner from config types."""
        if self._scheduler_type == SchedulerType.FSM:
            self._scheduler = FsmScheduler(0.01)
        else:
            raise ValueError("Invalid scheduler type.")

        if self._planner_type == PlannerType.RRT:
            self._planner = RrtPlanner(0.01)
        else:
            raise ValueError("Invalid planner type.")

    def control(self, status) -> JointState | None:
        """Return next joint command from scheduler/planner or current state."""
        np.copyto(self._current_joint_state.position, status.position)
        
        if self._is_homing:
            self.home()
        elif self._is_moving:
            self.move()
        elif self._is_operating:
            self.operating()
        elif self._is_stop:
            self.stop()

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
                np.copyto(self._current_joint_state.position, joint_command.position)
                self._scheduler.step()
                return joint_command
        return self._current_joint_state
        
    def update(self, status: JointState, obstacles: List[ObstacleState] | None = None) -> None:
        """Update internal state from joint feedback. Optional obstacle for planning.

        During a planned motion we do NOT overwrite _current_joint_state from status:
        it is set in control() from the command we send. So status (which may be
        from a previous cycle or a stale goal) cannot overwrite and jump to goal.
        """
        np.copyto(self._current_joint_state.position, status.position)
        self.forward_kinematics(self._current_joint_state.position)

        if obstacles is not None:
            self._current_obstacles = obstacles

    def forward_kinematics(self, joint_positions: np.ndarray) -> None:
        """Compute Cartesian state from joint state. Not implemented."""
        q1, q2, h1, h2 = joint_positions

        p1 = np.dot(self._Tws, np.array([0, 0, 0, 1]).T)[:3]
        p2 = np.dot(self._Tws, np.array([self._l1, 0, 0, 1]).T)[:3]
        p3 = np.dot(self._Tws, np.array([self._l1, self._l2, 0, 1]).T)[:3]
        p4 = np.dot(self._Tws, np.array([self._l3 * np.sin(q1) * np.sin(q2) + self._l1,
                                         self._l3 * np.cos(q2) + self._l2,
                                         self._l3 * np.cos(q1) * np.sin(q2), 1]).T)[:3]
        p5 = np.dot(self._Tws, np.array([-self._l1, 0, 0, 1]).T)[:3]
        p6 = np.dot(self._Tws, np.array([-self._l1, self._l2, 0, 1]).T)[:3]
        p7 = np.dot(self._Tws, np.array([-self._l3 * np.sin(h1) * np.sin(h2) - self._l1,
                                          self._l3 * np.cos(h2) + self._l2,
                                         -self._l3 * np.cos(h1) * np.sin(h2), 1]).T)[:3]

        self._current_joint_coordinates = np.array([Pose, Pose, Pose, Pose, Pose, Pose, Pose, Pose, Pose])
        self._current_joint_coordinates[0] = Pose(
            position=np.zeros(3),
            orientation=np.zeros(3),
        )
        self._current_joint_coordinates[1] = Pose(
            position=p1,
            orientation=np.zeros(3),
        )
        self._current_joint_coordinates[2] = Pose(
            position=p2,
            orientation=np.zeros(3),
        )
        self._current_joint_coordinates[3] = Pose(
            position=p3,
            orientation=np.zeros(3),
        )
        self._current_joint_coordinates[4] = Pose(
            position=p4,
            orientation=np.zeros(3),
        )
        self._current_joint_coordinates[5] = Pose(
            position=p1,
            orientation=np.zeros(3),
        )
        self._current_joint_coordinates[6] = Pose(
            position=p5,
            orientation=np.zeros(3),
        )
        self._current_joint_coordinates[7] = Pose(
            position=p6,
            orientation=np.zeros(3),
        )
        self._current_joint_coordinates[8] = Pose(
            position=p7,
            orientation=np.zeros(3),
        )

    def inverse_kinematics(self, robot_state: RobotState) -> JointState:
        """Compute joint state from Cartesian state. Not implemented."""
        pass

    def home(self) -> None:
        is_event, current_fsm_state = self._scheduler.tick(FsmAction(Action.HOME.value, duration=10.0))
        self._current_progress = current_fsm_state.progress

        if is_event:
            if current_fsm_state.progress >= 1.0:
                self._is_homing = False
                self._planner.reset()
                
            elif current_fsm_state.progress > 0.0 and self._current_joint_state is not None:
                target_joint_state = JointState(
                    id=self._current_joint_state.id,
                    position=self._current_joint_state.position.copy(),
                    velocity=self._current_joint_state.velocity.copy(),
                    torque=self._current_joint_state.torque.copy(),
                )
                target_joint_state.position[0] += 0.2
                target_joint_state.position[1] += 0.5
                
                obstacle = self._current_obstacles[0] if self._current_obstacles else None
                self._planner.plan(self._current_joint_state.position[:2].copy(), target_joint_state.position[:2].copy(), obstacle)

    def move(self) -> None:
        pass

    def stop(self) -> None:
        self._planner.reset()
        is_event, current_fsm_state = self._scheduler.tick(FsmAction(Action.STOP.value, duration=0.0))
        self._current_progress = current_fsm_state.progress
        if is_event:
            self._scheduler.reset()

    def operating(self) -> None:
        pass