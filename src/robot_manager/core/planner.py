"""Abstract planner: background thread, plan(), eval(), and _generate_trajectory()."""
from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from typing import List

import numpy as np

from robot_manager.types import (
    CircleObstacleState,
    SelfObstacleState,
    SphereObstacleState,
)

ObstacleList = List[SphereObstacleState | CircleObstacleState | SelfObstacleState]


class Planner(ABC):
    """
    Abstract planner: runs in a worker thread, plans from current to target.

    Config space is generic (np.ndarray). Subclasses implement eval and
    _generate_trajectory.
    """

    def __init__(self) -> None:
        """Create lock, condition variable, and start the daemon planning thread."""
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._is_planned = False
        self._is_running = False
        self._stop_requested = False
        self._current_state: np.ndarray | None = None
        self._target_state: np.ndarray | None = None
        self._obstacle_state: ObstacleList | None = None
        self._planner_thread = threading.Thread(target=self._run, daemon=True)
        self._planner_thread.start()

    @abstractmethod
    def eval(self, progress: float) -> np.ndarray | None:
        """
        Evaluate the planned trajectory at a given progress.

        Args:
            progress: Progress along the trajectory in [0, 1].

        Returns:
            Configuration at that progress, or None if no plan is available.
        """
        ...

    def plan(
        self,
        current_state: np.ndarray,
        target_state: np.ndarray,
        obstacle_state: ObstacleList | None = None,
    ) -> None:
        """
        Request a new plan from current to target (non-blocking).

        The request is processed in the worker thread. If a plan is already
        running, this call is ignored.

        Args:
            current_state: Start configuration.
            target_state: Goal configuration.
            obstacle_state: Optional obstacles for collision checking.
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
    def _generate_trajectory(
        self,
        current_state: np.ndarray,
        target_state: np.ndarray,
        obstacle_state: ObstacleList | None = None,
    ) -> bool:
        """
        Generate trajectory from current to target (called in worker thread).

        Args:
            current_state: Start configuration.
            target_state: Goal configuration.
            obstacle_state: Optional obstacles for collision checking.

        Returns:
            True if a trajectory was generated successfully.
        """
        ...

    def _run(self) -> None:
        """Worker loop: wait for plan request, then call _generate_trajectory."""
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
            success = self._generate_trajectory(current, target, obstacle)
            self._is_planned = success

    def reset(self) -> None:
        """Reset the planner (clear state and any pending plan)."""
        with self._cv:
            self._is_planned = False
            self._is_running = False
            self._stop_requested = False
            self._current_state = None
            self._target_state = None
            self._obstacle_state = None
