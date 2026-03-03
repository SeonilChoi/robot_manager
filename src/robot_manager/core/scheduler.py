from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

class Scheduler(ABC):
    def __init__(self, dt: float) -> None:
        self.dt_ = dt
        self.T_ = 0.0
        self.t_ = 0.0

    @abstractmethod
    def reset(self) -> None:
        ...

    @abstractmethod
    def step(self) -> None:
        ...

    @abstractmethod
    def tick(self, action):
        ...

    def _progress_raw(self, t: float) -> float:
        if self.T_ == 0.0:
            return 0.0
        return round(t / self.T_, 2)