"""Robot manager: loads YAML config, owns robot instance, exposes control/update/home/stop/move."""
from __future__ import annotations
from typing import List

import yaml

from robot_manager.types import JointState, SphereObstacleState, CircleObstacleState, RobotConfig
from robot_manager.robots.little_reader import LittleReader

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
        self._initialize()

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

        r_cfg = RobotConfig(
            id=robot["id"],
            number_of_joints=robot["number_of_joints"],
            controller_indexes=robot.get("controller_indexes") or [],
            scheduler_type=robot.get("scheduler_type"),
            planner_type=robot.get("planner_type"),
            robot_type=robot.get("type"),
        )

        if r_cfg.robot_type == "little_reader":
            self._robot = LittleReader(r_cfg)
        else:
            raise ValueError(f"Invalid robot type: {r_cfg.robot_type}")

    def _initialize(self) -> None:
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

    def update(self, status: JointState, obstacles: List[SphereObstacleState | CircleObstacleState] | None = None) -> None:
        """Update robot with current joint status and optional obstacle state.

        Parameters
        ----------
        status : JointState
            Current joint feedback (position, velocity, torque).
        obstacles : List[SphereObstacleState | CircleObstacleState] | None
            Optional obstacle state for planning/collision.
        """
        self._robot.update(status, obstacles)

    def home(self) -> None:
        """Start home motion."""
        self._robot._is_homing = True
        self._robot._is_moving = False
        self._robot._is_auto   = False

    def stop(self) -> None:
        """Stop motion."""
        self._robot._is_homing = False
        self._robot._is_moving = False
        self._robot._is_auto   = False
        
    def move(self) -> None:
        """Start or continue move."""
        self._robot._is_homing = False
        self._robot._is_moving = True
        self._robot._is_auto   = False

    def auto(self) -> None:
        """Start operating."""
        self._robot._is_homing = False
        self._robot._is_moving = False
        self._robot._is_auto   = True
