import time

from pytest import approx, raises

from edge_control.arch.husqvarna.frame import frame
from edge_control.arch.husqvarna.messages import Error, ErrorCode, Heartbeat, MotorSpeed, MotorStop, parse
from edge_control.arch.husqvarna.motor import from_move


def test_motor_move():
    from edge_control.models.messages import MoveCommand

    mc = MoveCommand(0, 0.1, 0)
    ms = from_move(mc, 0.3)
    assert ms.message().hex() == "0105fb01"


def test_motor_stop():
    assert MotorStop().message() == bytes([0])


def test_motor_speed():
    assert MotorSpeed([1, -1, 0]).message() == b"\x01\x01\xff\x00"


def test_motor_speed_end_time():
    assert MotorSpeed([1, -1, 0], 0xAABBCCDD).message() == b"\x01\x01\xff\x00\xdd\xcc\xbb\xaa"


def test_unknown():
    message = b"\x40yayaya"
    t = time.time()
    with raises(Exception):
        parse(t, message)


def test_heartbeat():
    message = bytes([0x88, 0xD0, 0x1E, 0x8E, 0x00, 0x02, 0x01, 0x03, 0x01])
    t = time.time()
    m = parse(t, message)
    assert isinstance(m, Heartbeat)
    assert m.timestamp == t
    assert m.time == 9314.0
    assert m.count0 == 258
    assert m.count1 == 259


def test_error():
    message = b"\xf0\x00Hello world!"
    t = time.time()
    m = parse(t, message)
    assert isinstance(m, Error)
    assert m.timestamp == t
    assert m.code == 0
    assert m.message == "Hello world!"
    assert ErrorCode(m.code) == ErrorCode.message


def test_frame():
    message = bytes([0x88, 0xD0, 0x1E, 0x8E, 0x00])
    assert frame(message) == bytes([0xDE, 0x05, 0x88, 0xD0, 0x1E, 0x8E, 0x00, 0xAD])
