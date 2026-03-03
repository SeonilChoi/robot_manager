from __future__ import annotations
from enum import Enum
from dataclasses import dataclass
from typing import Tuple

from robot_manager.core.scheduler import Scheduler

class State(Enum):
    STOPPED = 0
    OPERATING = 1
    HOMING = 2
    INVALID = 3

class Action(Enum):
    STOP = 0
    MOVE = 1
    HOME = 2

@dataclass
class FsmState:
    state: State
    progress: float

@dataclass
class FsmAction:
    action: Action
    duration: float

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

class FsmScheduler(Scheduler):
    def __init__(self, dt: float) -> None:
        super().__init__(dt)
        self.state_ = State.STOPPED
        
    def reset(self) -> None:
        self.state_ = State.STOPPED
        self.t_ = 0.0

    def step(self) -> None:
        self.t_ += self.dt_

    def tick(self, action: FsmAction) -> Tuple[bool, FsmState]:
        self.T_ = action.duration
        t = self.t_ + self.dt_ if self.T_ != 0.0 else 0.0

        next_state = FsmState(
            state=get_next_state(self.state_, action.action),
            progress=self._progress_raw(t)
        )

        if next_state.state == State.INVALID:
            raise ValueError(f"Invalid state transition.")

        if next_state.state != self.state_:
            self.state_ = next_state.state
            return True, next_state
        
        if next_state.progress == 1.0:
            self.t_ = 0.0
            if next_state.state == State.HOMING:
                self.state_ = State.STOPPED
            return True, next_state

        return False, next_state