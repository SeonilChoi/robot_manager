from __future__ import annotations
from enum import Enum
from typing import Tuple

from robot_manager.core import FsmAction, FsmState, Scheduler

class State(Enum):
    STOPPED = 0
    OPERATING = 1
    HOMING = 2
    INVALID = 3

class Action(Enum):
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

def get_next_state(current: State, action: Action) -> State:
    return TRANSITION_TABLE.get((current, action), State.INVALID)

def _to_action(a: int | Action) -> Action:
    return a if isinstance(a, Action) else Action(a)


class FsmScheduler(Scheduler):
    def __init__(self, dt: float) -> None:
        super().__init__(dt)
        self._state = State.STOPPED

    def reset(self) -> None:
        self._state = State.STOPPED
        self._t = 0.0

    def step(self) -> None:
        self._t += self._dt

    def tick(self, action: FsmAction) -> Tuple[bool, FsmState]:
        self._T = action.duration
        t = self._t + self._dt if self._T != 0.0 else 0.0

        next_enum = get_next_state(self._state, _to_action(action.action))
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
            if next_enum == State.HOMING:
                self._state = State.STOPPED
            return True, next_state

        return False, next_state