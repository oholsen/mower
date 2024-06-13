from abc import ABC
from typing import Optional

from dataclasses import dataclass


@dataclass
class SitePosition:
    x: float  # m
    y: float  # m
    hdop: float  # m


@dataclass(frozen=True)
class ObstacleDetection:
    time: float  # seconds
    # Flags obstacles detected in zones ahead of the robot.
    # Should also say something about angle and distance to obstacle.
    left: bool
    center_left: bool
    center_right: bool
    right: bool

    def center(self):
        return self.center_left or self.center_right


@dataclass
class ToRobot(ABC):
    # robot/motor controller clock time (absolute time, seconds), disabled if timeout <= 0.
    timeout: float


@dataclass
class FromRobot(ABC):
    time: float  # seconds


@dataclass
class Time(FromRobot):
    # timestamp: float  # real-time, in main controller
    robot_time: float  # robot time, in motor controller


@dataclass
class Status(FromRobot):
    # To be defined per robot type
    def ok(self) -> bool:
        raise NotImplemented


@dataclass
class Battery(FromRobot):
    voltage: float


@dataclass
class Odometry(FromRobot):
    speed: float
    omega: float


@dataclass
class MoveCommand(ToRobot):
    speed: float  # m/s, positive forwards
    omega: float  # rad/s, positive turning left


@dataclass
class StopCommand(MoveCommand):
    # Unconditional stop of motors
    # str() outputs speed, omega (from MoveCommand) - add dummy constructor arguments. Not a subclass of MoveCommand!?
    def __init__(self, timeout=0.0, speed=0.0, omega=0.0):
        assert speed == 0.0
        assert omega == 0.0
        self.timeout = timeout
        self.speed = 0
        self.omega = 0


@dataclass
class ResetCommand(ToRobot):
    pass


@dataclass
class DockCommand(ToRobot):
    pass


@dataclass
class CutCommand(ToRobot):
    # Will be merged into MoveCommand to motors and subject to StopCommand.
    # It is a state - will applied on every MoveCommand, also after StopCommand.
    power: float  # [-1, 1]


@dataclass
class LightsCommand:
    head: Optional[bool]
    strobe: Optional[bool]


class MissionCommand(ABC):
    pass


@dataclass
class MissionStart(MissionCommand):
    name: str


@dataclass
class MissionAbort(MissionCommand):
    pass
