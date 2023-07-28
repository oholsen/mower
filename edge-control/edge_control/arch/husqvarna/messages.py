from __future__ import annotations

import struct
from enum import Enum
from typing import Dict, List, Optional, Type

from dataclasses import dataclass

_RESP_HEARTBEAT = 0x88
_RESP_ERROR = 0xF0

_REQ_MOTOR_STOP = 0
_REQ_MOTOR_SPEED = 1


@dataclass
class FromMotorController(object):
    timestamp: float  # time.time()

    @staticmethod
    def parse(timestamp: float, message: bytes) -> FromMotorController:
        raise NotImplemented


class ToMotorController(object):
    def message(self) -> bytes:
        raise NotImplemented


@dataclass
class Heartbeat(FromMotorController):
    time: float  # seconds since motor controller started
    # LP filtered counts at 20Hz
    count0: int
    count1: int

    @staticmethod
    def parse(timestamp: float, message: bytes) -> Heartbeat:
        _, millis, count0, count1 = struct.unpack("<BLhh", message)
        time = millis / 1000.0  # seconds
        return Heartbeat(timestamp, time, count0, count1)


class ErrorCode(Enum):
    message = 0  # followed by string
    invalid_frame = 1
    invalid_request = 2
    motor_speed_end_time = 8


@dataclass
class Error(FromMotorController):
    code: int
    message: str

    @staticmethod
    def parse(timestamp: float, data: bytes) -> Error:
        code = data[1]
        message = data[2:].decode("ascii")
        return Error(timestamp, code, message)


@dataclass
class MotorStop(ToMotorController):
    def message(self) -> bytes:
        return bytes([_REQ_MOTOR_STOP])


@dataclass
class MotorReset(ToMotorController):
    def message(self) -> bytes:
        return bytes([_REQ_MOTOR_SPEED, 0xF2, 0, 0, 0, 0])


@dataclass
class MotorSpeed(ToMotorController):
    # end_time_millis allows a client to observe heartbeat time and use that as reference
    # for a end time - to make sure the controller does not executed delayed commands, e.g. buffered in
    # serial port queues.
    speeds: List[int]  # three channels int8_t
    end_time_millis: Optional[int] = None

    def message(self) -> bytes:
        assert len(self.speeds) == 3
        data = struct.pack("Bbbb", _REQ_MOTOR_SPEED, *self.speeds)
        if self.end_time_millis is not None:
            data += struct.pack("<L", self.end_time_millis)
        return data


_responses = {_RESP_HEARTBEAT: Heartbeat, _RESP_ERROR: Error}  # type: Dict[int, Type[FromMotorController]]


def parse(timestamp: float, message: bytes):
    clz = _responses.get(message[0])
    if clz is None:
        raise Exception(f"Unknown motor control message {message[0]}")
    return clz.parse(timestamp, message)
