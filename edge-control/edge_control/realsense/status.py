from typing import Optional

from dataclasses import dataclass

from edge_control.models.state import State
from edge_control.util.status import Status


@dataclass
class TagStatus:
    id: int
    pos: Optional[float]  # error in position
    yaw: Optional[float]  # error in yaw (heading)


@dataclass(frozen=True)
class Vector:
    x: float
    y: float
    z: float

    # def to_array(self):
    #     return numpy.array([self.x, self.y, self.z])


@dataclass(frozen=True)
class Rotation:
    # radians
    pitch: float
    roll: float
    yaw: float


class RealsenseStatus:
    tag_error = Status[TagStatus]()
    tags = Status[list]()  # list of TagStatus, typing module does not support that!
    state = Status[State]()
    confidence = Status[int]()
    heading_correction = Status[float]()
    position = Status[Vector]()
    rotation = Status[Rotation]()
    stream_error = Status[str]()
    override: float = 0  # time of override, revert to DIY tracking on expiration

    @staticmethod
    def fault(t: float):
        pass
