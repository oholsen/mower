from abc import ABC
from typing import Dict, List, Optional, Type

from . import RobotModel


def to_float(s: str) -> float:
    return float(s)


def to_int(s: str) -> Optional[int]:
    try:
        return int(s)
    except ValueError:
        return None


class FromRobot(ABC):
    def __init__(self, segments: List[str]):
        raise NotImplemented


class Time(FromRobot):
    def __init__(self, segments: List[str]):
        self.time = to_float(segments[1])

    def __str__(self):
        return f"Time({self.time})"


class Ticks(FromRobot):
    def __init__(self, segments: List[str]):
        self.right = to_float(segments[1]) * RobotModel.DIST_PER_TICK
        self.left = to_float(segments[2]) * RobotModel.DIST_PER_TICK


class Speed(FromRobot):
    def __init__(self, segments: List[str]):
        self.left = to_float(segments[1]) * RobotModel.DIST_PER_TICK
        self.right = to_float(segments[2]) * RobotModel.DIST_PER_TICK
        self.left_set_lp = to_float(segments[3]) * RobotModel.DIST_PER_TICK
        self.right_set_lp = to_float(segments[4]) * RobotModel.DIST_PER_TICK

    def speed(self):
        return (self.left + self.right) / 2

    def omega(self):
        return (self.right - self.left) / RobotModel.WHEEL_BASE

    def __str__(self):
        return f"Speed({self.left}, {self.right})"


class Power(FromRobot):
    def __init__(self, segments: List[str]):
        self.left = to_float(segments[1])
        self.right = to_float(segments[2])

    def max(self) -> float:
        return max(abs(self.left), abs(self.right))


class Battery(FromRobot):
    def __init__(self, segments: List[str]):
        self.voltage = float(segments[1])

    def __str__(self):
        return f"Battery({self.voltage})"


class ControlAck(FromRobot):
    """Ack as response from starting processing of control line"""

    pass


class HeartbeatAck(ControlAck):
    def __init__(self, segments: List[str]):
        pass

    def __str__(self):
        return "Heartbeat()"


class AckFromRobot(FromRobot):
    pass


class MoveAck(AckFromRobot):
    def __init__(self, segments: List[str]):
        self.speed = to_float(segments[1]) * RobotModel.DIST_PER_TICK
        self.omega = to_float(segments[2]) * RobotModel.DIST_PER_TICK / (RobotModel.WHEEL_BASE / 2)
        self.timeout = to_float(segments[3])

    def __str__(self):
        return f"Move({self.speed},{self.omega},{self.timeout})"


class StopAck(AckFromRobot):
    def __init__(self, segments: List[str]):
        pass

    def __str__(self):
        return "Stop()"


class ResetAck(AckFromRobot):
    def __init__(self, segments: List[str]):
        pass

    def __str__(self):
        return "Reset()"


class Status(FromRobot):
    STATUS_MAX_POWER = 1

    def __init__(self, segments: List[str]):
        self.status = int(segments[1])

    def max_power(self) -> bool:
        return self.status & self.STATUS_MAX_POWER != 0

    def ok(self) -> bool:
        return self.status == 0

    def __str__(self):
        return f"Status({self.status})"


class Ack(FromRobot):
    # Ack <time> <command as interpreted by robot>
    def __init__(self, time: float, command: FromRobot):
        self.time = time
        self.command = command

    def __str__(self):
        return f"Ack({self.time},{self.command})"


class Timeout(FromRobot):
    # Timeout <time> <command as interpreted by robot>
    def __init__(self, time: float, command: FromRobot):
        self.time = time
        self.command = command

    def __str__(self):
        return f"Timeout({self.time},{self.command})"


_acks = {
    "m": MoveAck,
    ".": StopAck,
    "!": ResetAck,
}  # type: Dict[str, Type[AckFromRobot]]


def control(segments: List[str]) -> Optional[AckFromRobot]:
    assert segments[0] == "Control"
    segments = segments[1:]
    ack = _acks.get(segments[0])
    if ack is not None:
        return ack(segments)
    return None


def ack(clz: Type, segments: List[str]):  # -> Optional[]:
    time = float(segments[1])
    segments = segments[2:]
    cmd = _acks.get(segments[0])
    if cmd is not None:
        return clz(time, cmd(segments))
    return None


class Ignore(FromRobot):
    def __init__(self, segments: List[str]):
        self.segments = segments

    def __str__(self):
        return f"Ignore({self.segments})"


_sentences = {
    "Time": Time,
    "Ticks": Ticks,
    "Speed": Speed,
    "Power": Power,
    "Battery": Battery,
    "Status": Status,
    "Control": control,
    "Ack": lambda segments: ack(Ack, segments),
    "Timeout": lambda segments: ack(Timeout, segments),
}


def process(line):
    line = line.strip()
    segments = line.split()
    cmd = segments[0]
    sentence = _sentences.get(cmd, Ignore)
    if sentence:
        return sentence(segments)


class RobotCommand(ABC):
    pass


class MoveCommand(RobotCommand):
    def __init__(self, speed: float, omega: float, timeout: float):
        self.speed = speed  # m/s
        self.omega = omega  # rad/s
        self.timeout = timeout

    def __str__(self):
        t = self.speed / RobotModel.DIST_PER_TICK
        r = self.omega * (RobotModel.WHEEL_BASE / 2) / RobotModel.DIST_PER_TICK
        return f"m {t:.2f} {r:.2f} {self.timeout:.3f}"


class StopCommand(RobotCommand):
    def __str__(self):
        return "."


class ResetCommand(StopCommand):
    def __str__(self):
        return "!"


class CutCommand(RobotCommand):
    def __init__(self, speed: float):
        self.speed = speed

    def __str__(self):
        return f"CUT {self.speed:.2f}"
