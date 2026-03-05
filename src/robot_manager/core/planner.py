from __future__ import annotations
from abc import ABC, abstractmethod

import threading

from robot_manager.core import JointState, ObstacleState

class Planner(ABC):
    def __init__(self) -> None:
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
        """Evaluate planned joint-space trajectory at progress; write into joint_command."""
        ...

    def plan(
        self,
        current_state: JointState,
        target_state: JointState,
        obstacle_state: ObstacleState,
    ) -> None:
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
        """Call before joining the planner thread (e.g. in derived destructor)."""
        with self._cv:
            self._stop_requested = True
            self._cv.notify()

    def is_planned(self) -> bool:
        return self._is_planned

    @abstractmethod
    def generate_trajectory(
        self,
        current_state: JointState,
        target_state: JointState,
        obstacle_state: ObstacleState,
    ) -> bool:
        """Generate trajectory in joint space. Returns True if successful."""
        ...

    def _run(self) -> None:
        while True:
            with self._cv:
                self._cv.wait_for(
                    lambda: self._stop_requested or self._is_running
                )
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
