from typing import Optional

from dataclasses import dataclass

from . import topics
from .models import cdf

MISSION_DOCKED = "Docked"
MISSION_COMPLETED = "Completed"
MISSION_RUNNING = "Running"
MISSION_ABORTED = "Aborted"


@dataclass
class MissionStatus:
    # represents current mission status as well as last completed mission.
    # should maybe split in two separate entities.
    start_time: float
    name: str
    fault: str = ""
    stop_time: Optional[float] = None
    status: str = ""

    def mission_id(self) -> str:
        return str(int(self.start_time * 1000))

    def to_cdf(self) -> cdf.Mission:
        m = cdf.Mission(
            missionId=self.mission_id(),
            name=self.name,
            startTime=int(1000 * self.start_time),
            status=self.status,
            fault=self.fault,
        )
        if self.stop_time is not None:
            m.endTime = int(1000 * self.stop_time)
        return m

    async def publish(self):
        await topics.missions.publish(self.to_cdf())


_mission: Optional[MissionStatus] = None


def get() -> Optional[MissionStatus]:
    return _mission


def mission_id() -> str:
    return _mission.mission_id() if _mission else ""


async def start(t: float, name: str):
    global _mission
    _mission = MissionStatus(start_time=t, name=name, status=MISSION_RUNNING)
    await _mission.publish()


async def abort(t: float, fault: str):
    assert _mission is not None
    if _mission.status == MISSION_RUNNING:
        _mission.stop_time = t
        _mission.fault = fault
        _mission.status = MISSION_ABORTED
        await _mission.publish()


async def complete(t: float):
    assert _mission is not None
    if _mission.status == MISSION_RUNNING:
        _mission.stop_time = t
        _mission.status = MISSION_COMPLETED
        await _mission.publish()
