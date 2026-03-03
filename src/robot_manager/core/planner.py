from __future__ import annotations
from abc import ABC, abstractmethod


class Planner(ABC):
    @abstractmethod
    def plan(self) -> None:
        ...
