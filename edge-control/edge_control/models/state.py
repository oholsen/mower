from __future__ import annotations

import numpy as np
from dataclasses import dataclass


@dataclass(frozen=True)
class State:
    """State required to update state by system model"""

    x: float
    y: float
    theta: float

    def to_array(self):
        return np.array([[self.x], [self.y], [self.theta]])

    def position(self) -> complex:
        return complex(self.x, self.y)

    def distance(self, other: State) -> float:
        return abs(self.position() - other.position())

    @staticmethod
    def from_array(s) -> State:
        s = s.flatten()
        return State(*s)


@dataclass(frozen=True)
class ModeState:
    mode: str
