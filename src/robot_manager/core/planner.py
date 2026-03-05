"""Abstract planner: background thread, plan(), eval(), generate_trajectory()."""
from __future__ import annotations

from abc import ABC, abstractmethod
import threading

from robot_manager.core import JointState, ObstacleState


class Planner(ABC):
    """Abstract planner: runs in a worker thread, plans from current to target with optional obstacles."""

    def __init__(self) -> None:
        """Create lock, condition, and start the planning thread (daemon)."""
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._is_planned = False
        self._is_running = False
        self._stop_requested = False
        self._current_state: JointState | None = None
        self._target_state: JointState | None = None
        self._obstacle_state: ObstacleState | None = None
        self._planner_thread = threading.Thread(target=self._run, daemon=True)
        self._planner_thread.start()

    @abstractmethod
    def eval(self, progress: float, joint_command: JointState) -> None:
        """Evaluate planned trajectory at progress and write result into joint_command.

        Parameters
        ----------
        progress : float
            Progress in [0, 1] along the trajectory.
        joint_command : JointState
            Output; position/velocity/torque are written in place.
        """
        ...

    def plan(
        self,
        current_state: JointState,
        target_state: JointState,
        obstacle_state: ObstacleState,
    ) -> None:
        """Request a new plan from current to target. Non-blocking; runs in worker thread.

        Parameters
        ----------
        current_state : JointState
            Start configuration.
        target_state : JointState
            Goal configuration.
        obstacle_state : ObstacleState
            Obstacle state for collision checking.
        """
        with self._cv:
            if self._is_running:
                return
            self._current_state = current_state
            self._target_state = target_state
            self._obstacle_state = obstacle_state
            self._is_planned = False
            self._is_running = True
            self._cv.notify()

    def request_stop(self) -> None:
        """Signal the worker thread to exit (e.g. before join)."""
        with self._cv:
            self._stop_requested = True
            self._cv.notify()

    def is_planned(self) -> bool:
        """Return True if the last plan request completed successfully."""
        return self._is_planned

    @abstractmethod
    def generate_trajectory(
        self,
        current_state: JointState,
        target_state: JointState,
        obstacle_state: ObstacleState,
    ) -> bool:
        """Generate trajectory from current to target. Returns True if successful."""
        ...

    def _run(self) -> None:
        """Worker loop: wait for plan request, then call generate_trajectory."""
        while True:
            with self._cv:
                self._cv.wait_for(lambda: self._stop_requested or self._is_running)
            if self._stop_requested:
                break
            current = self._current_state
            target = self._target_state
            obstacle = self._obstacle_state
            with self._cv:
                self._is_running = False
            if current is None or target is None:
                self._is_planned = False
                continue
            success = self.generate_trajectory(current, target, obstacle)
            self._is_planned = success
