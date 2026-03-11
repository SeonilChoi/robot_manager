"""Abstract planner: background thread, plan(), eval(), generate_trajectory(). Generic over config space (np.ndarray)."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List
import threading

import numpy as np

from robot_manager.types import SphereObstacleState, CircleObstacleState, SelfObstacleState


class Planner(ABC):
    """Abstract planner: runs in a worker thread, plans from current to target. Config space is generic (np.ndarray)."""

    def __init__(self) -> None:
        """Create lock, condition, and start the planning thread (daemon)."""
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        
        self._is_planned = False
        self._is_running = False
        self._stop_requested = False

        self._current_state: np.ndarray | None = None
        self._target_state: np.ndarray | None = None
        self._obstacle_state: List[SphereObstacleState | CircleObstacleState | SelfObstacleState] | None = None
        
        self._planner_thread = threading.Thread(target=self._run, daemon=True)
        self._planner_thread.start()

    @abstractmethod
    def eval(self, progress: float) -> np.ndarray | None:
        """Evaluate planned trajectory at progress. Returns config (np.ndarray) or None if no plan.

        Parameters
        ----------
        progress : float
            Progress in [0, 1] along the trajectory.

        Returns
        -------
        np.ndarray | None
            Configuration at that progress, or None.
        """
        ...

    def plan(
        self,
        current_state: np.ndarray,
        target_state: np.ndarray,
        obstacle_state: List[SphereObstacleState | CircleObstacleState | SelfObstacleState] | None = None,
    ) -> None:
        """Request a new plan from current to target. Non-blocking; runs in worker thread.

        Parameters
        ----------
        current_state : np.ndarray
            Start configuration.
        target_state : np.ndarray
            Goal configuration.
        obstacle_state : List[SphereObstacleState | CircleObstacleState | SelfObstacleState] | None
            Optional obstacle state for collision checking.
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
        obstacle_state: List[SphereObstacleState | CircleObstacleState | SelfObstacleState] | None = None,
    ) -> bool:
        """Generate trajectory from current to target. Config must be np.ndarray. Returns True if successful."""
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
            success = self._generate_trajectory(current, target, obstacle)
            self._is_planned = success

    def reset(self) -> None:
        """Reset the planner to initial state."""
        with self._cv:
            self._is_planned = False
            self._is_running = False
            self._stop_requested = False
            self._current_state = None
            self._target_state = None
            self._obstacle_state = None
