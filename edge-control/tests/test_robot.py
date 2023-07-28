import pytest

from edge_control import mission
from edge_control.robot import RobotState


def test_empty():
    keys = RobotState.as_dict(10.0).keys()
    assert "docked" in keys
    assert "mission" in keys


def test_docked():
    # mutates global state, fails if test is duplicated
    assert not RobotState.docked
    RobotState.docked = True
    assert RobotState.docked
    assert RobotState.as_dict(10.0)["docked"] is True


def test_status():
    t = 20.0
    from edge_control.arch.roomba.messages import Infrared

    RobotState.roomba.ir_omni.set(Infrared(162), t)
    # RobotState.roomba.foo = Infrared(22)
    print(RobotState.as_dict(t + 0.1))
    # print(RobotState.as_dict(t + 2.1))


@pytest.mark.asyncio
async def test_mission():
    await mission.start(2.0, "Test mission")
    await mission.complete(4.0)
    m = RobotState.as_dict(10.0)["mission"]
    assert {"name": "Test mission", "start_time": 2.0, "status": "Completed", "fault": "", "stop_time": 4.0} == m
