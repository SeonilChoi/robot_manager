"""FSM scheduler: states STOPPED, OPERATING, HOMING; transition table and tick()."""
from __future__ import annotations

from enum import Enum
from typing import Tuple

from robot_manager.core.scheduler import Scheduler
from robot_manager.types import FsmAction, FsmState


class State(Enum):
    """FSM state identifiers."""
    STOPPED = 0
    OPERATING = 1
    HOMING = 2
    INVALID = 3


class Action(Enum):
    """FSM actions."""
    STOP = 0
    MOVE = 1
    HOME = 2


TRANSITION_TABLE: dict[tuple[State, Action], State] = {
    (State.STOPPED, Action.STOP): State.STOPPED,
    (State.STOPPED, Action.MOVE): State.OPERATING,
    (State.STOPPED, Action.HOME): State.HOMING,
    (State.OPERATING, Action.STOP): State.STOPPED,
    (State.OPERATING, Action.MOVE): State.OPERATING,
    (State.OPERATING, Action.HOME): State.INVALID,
    (State.HOMING, Action.STOP): State.STOPPED,
    (State.HOMING, Action.MOVE): State.INVALID,
    (State.HOMING, Action.HOME): State.HOMING,
}


def _get_next_state(current: State, action: Action) -> State:
    """
    Return the next FSM state from the transition table.

    Args:
        current: Current FSM state.
        action: Incoming action.

    Returns:
        Next state; State.INVALID if the (current, action) pair has no entry.
    """
    return TRANSITION_TABLE.get((current, action), State.INVALID)


def get_next_state(current: State, action: Action) -> State:
    """Public alias for _get_next_state."""
    return _get_next_state(current, action)


def _to_action(a: int | Action) -> Action:
    """Convert an int to Action enum if needed."""
    return a if isinstance(a, Action) else Action(a)


class FsmScheduler(Scheduler):
    """
    Scheduler that advances FSM state and progress based on actions and time.

    When progress reaches 1.0, state resets to STOPPED. Invalid transitions
    raise ValueError.
    """

    def __init__(self, dt: float) -> None:
        """
        Initialize with time step and initial state STOPPED.

        Args:
            dt: Time step used in step().
        """
        super().__init__(dt)
        self._state = State.STOPPED

    def reset(self) -> None:
        """Set state to STOPPED and internal time to zero."""
        self._state = State.STOPPED
        self._t = 0.0

    def step(self) -> None:
        """Advance internal time by _dt."""
        self._t += self._dt

    def tick(self, action: FsmAction) -> Tuple[bool, FsmState]:
        """
        Apply the action and return whether state changed and the new FSM state.

        Args:
            action: Action identifier and duration.

        Returns:
            (changed, fsm_state). changed is True if the state transitioned or
            progress reached 1.0.

        Raises:
            ValueError: If the transition (current state, action) is INVALID.
        """
        self._T = action.duration
        t = self._t + self._dt if self._T != 0.0 else 0.0
        next_enum = _get_next_state(self._state, _to_action(action.action))
        next_state = FsmState(
            state=next_enum.value,
            progress=self._progress_raw(t),
        )
        if next_enum == State.INVALID:
            raise ValueError("Invalid state transition.")
        if next_enum != self._state:
            self._state = next_enum
            return True, next_state
        if next_state.progress == 1.0:
            self._t = 0.0
            self._state = State.STOPPED
            return True, next_state
        return False, next_state
