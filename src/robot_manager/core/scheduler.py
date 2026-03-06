"""Abstract scheduler: time step, reset, step, tick(action) -> (changed, FsmState)."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Tuple

from robot_manager.types import FsmAction, FsmState


class Scheduler(ABC):
    """Abstract scheduler: advances time and returns FSM state for a given action."""

    def __init__(self, dt: float) -> None:
        """Set time step and initialize internal time and duration.

        Parameters
        ----------
        dt : float
            Time step (e.g. seconds) for step().
        """
        self._dt = dt
        self._T = 0.0
        self._t = 0.0

    @abstractmethod
    def reset(self) -> None:
        """Reset scheduler to initial state and zero time."""
        ...

    @abstractmethod
    def step(self) -> None:
        """Advance internal time by _dt."""
        ...

    @abstractmethod
    def tick(self, action: FsmAction) -> Tuple[bool, FsmState]:
        """Apply action and return whether state changed and the new FSM state.

        Parameters
        ----------
        action : FsmAction
            Action id and duration.

        Returns
        -------
        Tuple[bool, FsmState]
            (changed, fsm_state) where changed is True if state or progress changed.
        """
        ...

    def _progress_raw(self, t: float) -> float:
        """Return progress in [0, 1] for current duration; 0 if duration is 0."""
        if self._T == 0.0:
            return 0.0
        return round(t / self._T, 3)
