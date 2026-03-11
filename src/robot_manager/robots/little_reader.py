"""LittleReader robot model: FSM scheduler, RRT planner, and DH-based kinematics."""
from __future__ import annotations

from typing import List, Tuple

import numpy as np

from robot_manager.core.robot import Robot
from robot_manager.planner.rrt_planner import RrtPlanner
from robot_manager.scheduler.fsm_scheduler import Action, FsmAction, FsmScheduler
from robot_manager.types import (
    CircleObstacleState,
    JointState,
    RobotConfig,
    SelfObstacleState,
    SphereObstacleState,
)
from robot_manager.utils.kinematics import forward_kinematics
from robot_manager.utils.utils import interpolate


class LittleReader(Robot):
    """
    Robot implementation with FSM scheduler and RRT planner.

    Uses Denavit–Hartenberg forward kinematics and a simple 2D IK for left/right
    arms. Supports circle and self-collision checking for planning.
    """

    def __init__(self, config: RobotConfig) -> None:
        """Initialize link lengths, joint state, obstacles, and configuration bounds."""
        super().__init__(config)

        self._wz = 0.05
        self._l1 = 0.15
        self._l2 = 0.15
        self._l3 = 0.4

        self._current_joint_state = JointState(
            id=np.arange(self._number_of_joints),
            position=np.zeros(self._number_of_joints),
            velocity=np.zeros(self._number_of_joints),
            torque=np.zeros(self._number_of_joints),
        )
        self._current_joint_coordinates = np.zeros((8, 3), dtype=np.float64)
        self._current_obstacles = [
            SelfObstacleState(
                id=i,
                position=np.array([0.0, 0.0, 0.0]),
                radius=0.05,
                neighbor_id=[1-i if i <= 1 else i-2, None if i >= 15 else i+2],
            ) for i in range(0, 17)
        ]
        self._current_obstacles.append(
            CircleObstacleState(
                id=17,
                position=np.array([0.0, 0.0, 0.0]),
                radius=0.5,
                axis=2
            )
        )
        self._current_progress = 0.0

        self._configuration_space_bounds = {
            "min": np.array([-1.0, 0.0, -1.0, -1.5]),
            "max": np.array([ 1.0, 1.5,  1.0,  0.0]),
        }

        self._current_configuration = np.array([0.0, 0.0, 0.0, 0.0])
        self._home_configuration = np.array([0.5, 0.5, -0.5, -0.5])
        self._home_count = 0
        self._control_indexes = [np.array([0, 1]), np.array([2, 3])]
        self._meaningful_joint_indexes = np.array([0, 2, 5, 9, 12, 15, 19])
        self._self_collision_joint_indexes = np.array(
            [0, 1, 11, 2, 12, 4, 14, 5, 15, 6, 16, 7, 17, 8, 18, 9, 19]
        )

    def initialize(self) -> None:
        """
        Create FSM scheduler and RRT planner from config and set collision checker.

        Raises:
            ValueError: If scheduler_type or planner_type is not supported.
        """
        if self._scheduler_type == "fsm":
            self._scheduler = FsmScheduler(0.05)
        else:
            raise ValueError("Invalid scheduler type.")

        if self._planner_type == "rrt":
            self._planner = RrtPlanner()
        else:
            raise ValueError("Invalid planner type.")

        self._update_current_joint_coordinates_and_obstacles(np.zeros(self._number_of_joints))

        self._planner.set_collision_checker(
            collision_fn=self._collision_checker,
            segment_fn=self._segment_collision_checker,
        )

    def control(self, status: JointState) -> JointState | None:
        """
        Compute next joint command from scheduler/planner or return current state.

        Args:
            status: Current joint feedback (unused for command; state is internal).

        Returns:
            JointState with commanded positions, or current state if no plan active.
        """
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
                joint_command = JointState(
                    id=np.arange(self._number_of_joints),
                    position=np.zeros(self._number_of_joints),
                    velocity=np.zeros(self._number_of_joints),
                    torque=np.zeros(self._number_of_joints),
                )
                if self._home_count == 0:
                    joint_command.position = np.concatenate([config, self._current_configuration[2:]])
                else:
                    joint_command.position = np.concatenate([self._current_configuration[:2], config])
                self._scheduler.step()
                return joint_command

        return self._current_joint_state

    def update(
        self,
        status: JointState,
        obstacles: List[SphereObstacleState | CircleObstacleState] | None = None,
    ) -> None:
        """
        Update internal state from joint feedback and optionally merge obstacles.

        Args:
            status: Current joint feedback; updates position and FK/obstacles.
            obstacles: Optional list; obstacles with new ids are appended to
                _current_obstacles (existing ids are not replaced).
        """
        self._current_joint_state.position = status.position.copy()
        self._update_current_joint_coordinates_and_obstacles(self._current_joint_state.position)

        self._current_configuration = self._current_joint_state.position.copy()

        if obstacles is None:
            return
        existing_ids = {o.id for o in self._current_obstacles}
        for obs in obstacles:
            if obs.id not in existing_ids:
                self._current_obstacles.append(obs)

    def inverse_kinematics(self, position: np.ndarray) -> np.ndarray:
        """
        Compute joint positions for left and right end-effector positions.

        Args:
            position: (6,) array: [x_l, y_l, z_l, x_r, y_r, z_r] in task space.

        Returns:
            (number_of_joints,) joint position array.
        """
        joint_positions = np.zeros(self._number_of_joints)
        joint_positions[0], joint_positions[1] = self._inverse_kinematics(position[0], position[1], position[2], 1)
        joint_positions[2], joint_positions[3] = self._inverse_kinematics(position[3], position[4], position[5], -1)
        return joint_positions

    def get_circle_obstacles(
        self, joint_positions: np.ndarray
    ) -> List[CircleObstacleState]:
        """
        Return circle obstacles from current obstacle list.

        Args:
            joint_positions: Not used; kept for API consistency.

        Returns:
            List of CircleObstacleState from _current_obstacles.
        """
        obstacles = []
        for obs in self._current_obstacles:
            if isinstance(obs, CircleObstacleState):
                obstacles.append(obs)
        return obstacles

    def get_self_obstacles(
        self, joint_positions: np.ndarray
    ) -> List[SelfObstacleState]:
        """
        Compute self-obstacle spheres from FK at given joint positions.

        Args:
            joint_positions: Joint positions for FK.

        Returns:
            List of SelfObstacleState (link spheres with neighbor exclusions).
        """
        left_dh_params, right_dh_params = self._dh_parameters(joint_positions)
        left_points = forward_kinematics(left_dh_params)
        right_points = forward_kinematics(right_dh_params)
        points = np.concatenate([left_points, right_points], axis=0)

        obstacles = []
        for i in range(0, 17):
            obstacles.append(
                SelfObstacleState(
                    id=i,
                    position=points[self._self_collision_joint_indexes[i]],
                    radius=0.05,
                    neighbor_id=[1-i if i <= 1 else i-2, None if i >= 15 else i+2]))
        return obstacles

    def get_joint_coordinates(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Compute 3D coordinates of selected link frames from joint positions.

        Args:
            joint_positions: Joint positions for FK.

        Returns:
            (8, 3) array of xyz positions for meaningful joint indexes.
        """
        left_dh_params, right_dh_params = self._dh_parameters(joint_positions)
        left_points = forward_kinematics(left_dh_params)
        right_points = forward_kinematics(right_dh_params)
        points = np.concatenate([left_points, right_points], axis=0)

        joint_coordinates = np.zeros((8, 3), dtype=np.float64)
        joint_coordinates[1:] = points[self._meaningful_joint_indexes]
        return joint_coordinates

    def _point_in_circle(
        self,
        point: np.ndarray,
        center: np.ndarray,
        radius: float,
        axis: int,
    ) -> bool:
        """Return True if point lies inside the circle in the plane normal to axis."""
        p = point[axis]
        if p != center[axis]:
            return False
        if axis == 0:
            return np.linalg.norm(point[:2] - center[:2]) < radius
        elif axis == 1:
            return np.linalg.norm(point[np.array([0, 2])] - center[np.array([0, 2])]) < radius
        elif axis == 2:
            return np.linalg.norm(point[1:] - center[1:]) < radius
        else:
            raise ValueError("Invalid axis.")

    def _segment_intersects_circle(
        self,
        a: np.ndarray,
        b: np.ndarray,
        center: np.ndarray,
        radius: float,
        axis: int,
    ) -> bool:
        """Return True if segment a-b intersects the circle (plane normal to axis)."""
        denom = b[axis] - a[axis]
        if abs(denom) < 1e-12:
            return False
        t = (center[axis] - a[axis]) / denom
        p = a + t * (b - a)
        return float(np.linalg.norm(p - center)) < radius

    def _config_collision_circles(
        self,
        config: np.ndarray,
        obstacle_state: List[CircleObstacleState],
    ) -> bool:
        """Return True if the config places the end-effector inside any circle obstacle."""
        if obstacle_state is None:
            return False
        if self._home_count == 0:
            joint_positions = np.concatenate([config, self._current_configuration[2:]])
        else:
            joint_positions = np.concatenate([self._current_configuration[:2], config])
        position = self.get_joint_coordinates(joint_positions)[4 if self._home_count == 0 else 7]
        for obs in obstacle_state:
            c, r, axis = np.asarray(obs.position).ravel(), obs.radius, obs.axis
            if self._point_in_circle(position, c, r, axis):
                return True
        return False

    def _segment_collision_circles(
        self,
        config_a: np.ndarray,
        config_b: np.ndarray,
        obstacle_state: List[CircleObstacleState],
    ) -> bool:
        """Return True if the segment between config_a and config_b hits a circle obstacle."""
        if obstacle_state is None:
            return False
        if self._home_count == 0:
            joint_positions_a = np.concatenate([config_a, self._current_configuration[2:]])
            joint_positions_b = np.concatenate([config_b, self._current_configuration[2:]])
        else:
            joint_positions_a = np.concatenate([self._current_configuration[:2], config_a])
            joint_positions_b = np.concatenate([self._current_configuration[:2], config_b])
        position_a = self.get_joint_coordinates(joint_positions_a)[4 if self._home_count == 0 else 7]
        position_b = self.get_joint_coordinates(joint_positions_b)[4 if self._home_count == 0 else 7]
        for obs in obstacle_state:
            c, r, axis = np.asarray(obs.position).ravel(), obs.radius, obs.axis
            if self._segment_intersects_circle(position_a, position_b, c, r, axis):
                return True
        return False

    def _point_in_self_obstacle(
        self, obstacle_states: List[SelfObstacleState]
    ) -> bool:
        """Return True if any non-neighbor pair of self-obstacles overlaps."""
        for obs in obstacle_states:
            for i in range(0, 17):
                if i == obs.id:
                    continue
                if i == obs.neighbor_id[0] or i == obs.neighbor_id[1]:
                    continue
                distance = np.linalg.norm(obs.position - obstacle_states[i].position)
                if distance < obs.radius + obstacle_states[i].radius:
                    return True
        return False

    def _segment_intersects_self(
        self,
        a: np.ndarray,
        b: np.ndarray,
        obstacle_states: List[SelfObstacleState],
    ) -> bool:
        """Return True if interpolating joint positions from a to b causes self-collision."""
        for s in range(1, 10):
            t = s / 10
            q = interpolate(a, b, t)
            
            if self._home_count == 0:
                joint_positions = np.concatenate([q, self._current_configuration[2:]])
            else:
                joint_positions = np.concatenate([self._current_configuration[:2], q])
            obstacle_states = self.get_self_obstacles(joint_positions)
            if self._point_in_self_obstacle(obstacle_states):
                return True
        return False

    def _config_collision_self(
        self,
        config: np.ndarray,
        obstacle_state: List[SelfObstacleState],
    ) -> bool:
        """Return True if config yields self-collision (non-neighbor link overlap)."""
        if obstacle_state is None:
            return False
        if self._home_count == 0:
            joint_positions = np.concatenate([config, self._current_configuration[2:]])
        else:
            joint_positions = np.concatenate([self._current_configuration[:2], config])
        obstacle_state = self.get_self_obstacles(joint_positions)
        if self._point_in_self_obstacle(obstacle_state):
            return True
        return False

    def _segment_collision_self(
        self,
        config_a: np.ndarray,
        config_b: np.ndarray,
        obstacle_state: List[SelfObstacleState],
    ) -> bool:
        """Return True if the segment between config_a and config_b has self-collision."""
        if obstacle_state is None:
            return False
        if self._home_count == 0:
            joint_positions_a = np.concatenate([config_a, self._current_configuration[2:]])
            joint_positions_b = np.concatenate([config_b, self._current_configuration[2:]])
        else:
            joint_positions_a = np.concatenate([self._current_configuration[:2], config_a])
            joint_positions_b = np.concatenate([self._current_configuration[:2], config_b])
        if self._segment_intersects_self(joint_positions_a, joint_positions_b, obstacle_state):
            return True
        return False

    def _collision_checker(
        self,
        config: np.ndarray,
        obstacle_state: List[
            SphereObstacleState | CircleObstacleState | SelfObstacleState
        ],
    ) -> bool:
        """Return True if config is in collision with any circle or self obstacle."""
        circle_obstacles = []
        self_obstacles = []
        for obs in obstacle_state:
            if isinstance(obs, CircleObstacleState):
                circle_obstacles.append(obs)
            elif isinstance(obs, SelfObstacleState):
                self_obstacles.append(obs)
        if self._config_collision_circles(config, circle_obstacles):
            return True
        if self._config_collision_self(config, self_obstacles):
            return True
        return False

    def _segment_collision_checker(
        self,
        config_a: np.ndarray,
        config_b: np.ndarray,
        obstacle_state: List[
            SphereObstacleState | CircleObstacleState | SelfObstacleState
        ],
    ) -> bool:
        """Return True if segment config_a–config_b collides with circle or self obstacles."""
        circle_obstacles = []
        self_obstacles = []
        for obs in obstacle_state:
            if isinstance(obs, CircleObstacleState):
                circle_obstacles.append(obs)
            elif isinstance(obs, SelfObstacleState):
                self_obstacles.append(obs)
        if self._segment_collision_circles(config_a, config_b, circle_obstacles):
            return True
        if self._segment_collision_self(config_a, config_b, self_obstacles):
            return True
        return False

    def _inverse_kinematics(
        self, x: float, y: float, z: float, side: int
    ) -> Tuple[float, float]:
        """
        Solve 2D IK for one arm: (x, y, z) -> (joint0, joint1).

        Args:
            x, y, z: End-effector position in task space.
            side: 1 for one arm, -1 for the other.

        Returns:
            (th1, th2) joint angles for that arm.
        """
        py = y + self._l1 * side
        pz = z - self._wz
        q1 = np.arctan2(py, pz)

        sx = x - self._l2
        sz = np.sin(q1) * py + np.cos(q1) * pz
        q2 = np.arctan2(sz, sx)

        th1 = q1 * side
        th2 = q2 * side
        return th1, th2

    def _dh_parameters(
        self, joint_positions: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Build left and right arm DH parameter matrices from joint positions.

        Args:
            joint_positions: (4,) joint positions (left then right arm).

        Returns:
            (left_dh_params, right_dh_params) each (n_links, 4).
        """
        left_dh_params = np.array([[         0,        0, self._wz,           -np.pi/2],
                                   [self._l1/2,        0,        0,                  0],
                                   [self._l1/2, -np.pi/2,        0, joint_positions[0]],
                                   [         0,  np.pi/2,        0,            np.pi/2],
                                   [self._l2/2,        0,        0,                  0],
                                   [self._l2/2,  np.pi/2,        0, joint_positions[1]],
                                   [self._l3/4,        0,        0,                  0],
                                   [self._l3/4,        0,        0,                  0],
                                   [self._l3/4,        0,        0,                  0],
                                   [self._l3/4,        0,        0,                  0]])
        
        right_dh_params = np.array([[          0,        0, self._wz,           -np.pi/2],
                                    [-self._l1/2,        0,        0,                  0],
                                    [-self._l1/2, -np.pi/2,        0, joint_positions[2]],
                                    [          0,  np.pi/2,        0,            np.pi/2],
                                    [ self._l2/2,        0,        0,                  0],
                                    [ self._l2/2, -np.pi/2,        0, joint_positions[3]],
                                    [ self._l3/4,        0,        0,                  0],
                                    [ self._l3/4,        0,        0,                  0],
                                    [ self._l3/4,        0,        0,                  0],
                                    [ self._l3/4,        0,        0,                  0]])
        
        return left_dh_params, right_dh_params

    def _update_current_joint_coordinates_and_obstacles(
        self, joint_positions: np.ndarray
    ) -> None:
        """Update _current_joint_coordinates and self-obstacle positions from FK."""
        left_dh_params, right_dh_params = self._dh_parameters(joint_positions)
        left_points = forward_kinematics(left_dh_params)
        right_points = forward_kinematics(right_dh_params)
        points = np.concatenate([left_points, right_points], axis=0)

        self._current_joint_coordinates[1:] = points[self._meaningful_joint_indexes]
        for i in range(0, 17):
            self._current_obstacles[i].position = points[self._self_collision_joint_indexes[i]]

    def _home(self) -> None:
        """Advance homing FSM: tick HOME, update progress, plan to home config when needed."""
        is_event, current_fsm_state = self._scheduler.tick(
            FsmAction(Action.HOME.value, duration=10.0)
        )
        self._current_progress = current_fsm_state.progress

        if is_event:
            if current_fsm_state.progress >= 1.0:
                self._home_count += 1
                self._planner.reset()
                
            elif current_fsm_state.progress > 0.0 and self._current_joint_state is not None:
                print(self._home_count)
                self._planner.set_bounds(
                    min_bounds=self._configuration_space_bounds["min"][self._control_indexes[self._home_count]].flatten(),
                    max_bounds=self._configuration_space_bounds["max"][self._control_indexes[self._home_count]].flatten(),
                )
                
                self._planner.plan(
                    self._current_configuration[self._control_indexes[self._home_count]].flatten(),
                    self._home_configuration[self._control_indexes[self._home_count]].flatten(),
                    self._current_obstacles
                )

        if self._home_count == 2:
            self._home_count = 0
            self._is_homing = False

    def _move(self) -> None:
        """Move mode: placeholder (no extra behavior beyond planned motion)."""
        pass

    def _stop(self) -> None:
        """Stop motion: send STOP, reset planner and scheduler, clear progress."""
        self._scheduler.tick(FsmAction(Action.STOP.value, duration=0.0))
        self._planner.reset()
        self._scheduler.reset()
        self._current_progress = 0.0

    def _auto(self) -> None:
        """Auto mode: placeholder (no extra behavior)."""
        pass


if __name__ == "__main__":
    config = RobotConfig(
        id=1,
        number_of_joints=4,
        scheduler_type="fsm",
        planner_type="rrt",
        robot_type="little_reader",
        controller_indexes=[],
    )
    robot = LittleReader(config)