"""Robot manager: loads YAML config, owns robot instance, exposes control/update/home/stop/move."""
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


class RobotManager:
    """High-level robot interface: load config, initialize robot, run control loop and commands."""

    def __init__(self, config_file: str) -> None:
        """Load robot config from YAML and initialize the robot.

        Parameters
        ----------
        config_file : str
            Path to YAML file with a 'robot' section (id, number_of_joints,
            scheduler_type, planner_type, type, optional controller_indexes).
        """
        self._load_configurations(config_file)
        self.initialize()

    def _load_configurations(self, config_file: str) -> None:
        """Parse YAML and instantiate the configured robot (e.g. LittleReader)."""
        with open(config_file, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        robot = config.get("robot")
        if robot is None or not isinstance(robot, dict):
            raise ValueError("Config must contain a 'robot' section (dict)")

        for key in ("id", "number_of_joints", "scheduler_type", "planner_type"):
            if robot.get(key) is None:
                raise ValueError(f"Config 'robot' must specify '{key}'")

        st = robot.get("scheduler_type")
        pt = robot.get("planner_type")
        r_cfg = RobotConfig(
            id=robot["id"],
            number_of_joints=robot["number_of_joints"],
            controller_indexes=robot.get("controller_indexes") or [],
            scheduler_type=to_scheduler_type(st),
            planner_type=to_planner_type(pt),
            robot_type=to_robot_type(robot.get("type") or "little_reader"),
        )

        if robot.get("type") == "little_reader":
            self._robot = LittleReader(r_cfg)
        else:
            raise ValueError(f"Invalid robot type: {robot.get('type')}")

    def initialize(self) -> None:
        """Initialize the robot (scheduler, planner, etc.)."""
        self._robot.initialize()

    def control(self, status: JointState) -> JointState | None:
        """Compute and return the next joint command (control output).

        Returns
        -------
        JointState | None
            Command state, or None if not available.
        """
        return self._robot.control(status)

    def update(self, status: JointState, obstacle: ObstacleState | None = None) -> None:
        """Update robot with current joint status and optional obstacle state.

        Parameters
        ----------
        status : JointState
            Current joint feedback (position, velocity, torque).
        obstacle : ObstacleState | None
            Optional obstacle state for planning/collision.
        """
        obstacles = [obstacle] if obstacle is not None else None
        self._robot.update(status, obstacles)

    def home(self) -> JointState | None:
        """Send home command. Returns home state if defined, else None."""
        self._robot._is_homing = True
        self._robot._is_moving = False
        self._robot._is_operating = False
        self._robot.home()

    def stop(self) -> None:
        """Stop motion."""
        self._robot._is_homing = False
        self._robot._is_moving = False
        self._robot._is_operating = False
        
    def move(self) -> None:
        """Start or continue move."""
        self._robot._is_homing = False
        self._robot._is_moving = True
        self._robot._is_operating = False

    def operating(self) -> None:
        """Start operating."""
        self._robot._is_homing = False
        self._robot._is_moving = False
        self._robot._is_operating = True
