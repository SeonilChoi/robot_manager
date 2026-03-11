"""Abstract scheduler: time step, reset, step, and tick(action) -> (changed, FsmState)."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Tuple

from robot_manager.types import FsmAction, FsmState


class Scheduler(ABC):
    """
    Abstract scheduler: advances time and returns FSM state for a given action.

    Subclasses implement reset, step, and tick to drive state transitions
    and progress along an action duration.
    """

    def __init__(self, dt: float) -> None:
        """
        Set time step and initialize internal time and duration.

        Args:
            dt: Time step (e.g. seconds) used in step().
        """
        self._dt = dt
        self._T = 0.0
        self._t = 0.0

    @abstractmethod
    def reset(self) -> None:
        """Reset scheduler to initial state and set internal time to zero."""
        ...

    @abstractmethod
    def step(self) -> None:
        """Advance internal time by _dt."""
        ...

    @abstractmethod
    def tick(self, action: FsmAction) -> Tuple[bool, FsmState]:
        """
        Apply the given action and return whether state changed and the new FSM state.

        Args:
            action: Action identifier and duration.

        Returns:
            A pair (changed, fsm_state). changed is True if the state or
            progress changed (e.g. transition or progress reached 1.0).
        """
        ...

    def _progress_raw(self, t: float) -> float:
        """
        Return progress in [0, 1] for the current action duration.

        Args:
            t: Current time within the action.

        Returns:
            Progress in [0, 1], or 0.0 if duration is zero.
        """
        if self._T == 0.0:
            return 0.0
        return round(t / self._T, 3)
