from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

class Scheduler(ABC):
    def __init__(self, dt: float) -> None:
        self._dt = dt
        self._T = 0.0
        self._t = 0.0

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
        if self._T == 0.0:
            return 0.0
        return round(t / self._T, 2)