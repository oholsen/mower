import asyncio
import logging

from dataclasses import dataclass

from .. import mission, topics
from ..arch.roomba.ptz import camera
from ..config import robot_config
from ..models import cdf
from ..models.ptz import PanTiltZoomPosition, Snapshot
from ..models.state import State
from ..util.tasks import start_task
from ..util.time import now
from . import Control

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class AsyncSnapshot(Snapshot):
    done = asyncio.Event()

    async def completed(self):
        self.done.set()


async def ptz(pos: PanTiltZoomPosition, wait: float):
    await topics.ptz_command.publish(pos)
    # No feedback from servos - allow servos to settle
    await asyncio.sleep(wait)


def start_stream():
    camera.start_stream()


def stop_stream():
    camera.stop_stream()


async def snapshot(pos: PanTiltZoomPosition, grace1: float, filename: str, grace2: float):
    logger.info("snapshot: %s %s", pos, filename)
    await topics.ptz_command.publish(pos)
    await asyncio.sleep(grace1)
    camera.snapshot(filename)
    await asyncio.sleep(grace2)

    # snap = AsyncSnapshot(filename)
    # await topics.ptz_command.publish(snap)
    # await snap.done.wait()


async def capture_gauge(position: PanTiltZoomPosition, gauge_id: str):
    grace1 = 5
    grace2 = 1

    # stop_stream()
    logger.info("Gauge %s pos: %s", gauge_id, position)
    await topics.ptz_command.publish(position)
    # wait to get the PTZ in position AND stabilize PTZ and robot.
    # there is no feedback from servos on position
    await asyncio.sleep(grace1)

    timestamp = now()
    filename = f"data/image/{timestamp}.jpg"
    logger.info("Gauge %s file: %s", gauge_id, filename)
    camera.snapshot(filename)

    url = f"http://{robot_config.ip_address}/{filename}"

    capture = cdf.GaugeCapture(
        gaugeId=gauge_id,
        time=timestamp,
        missionId=mission.mission_id(),
        url=url,
        mimeType="image/jpeg",
    )

    logger.info("Gauge capture: %s", capture)
    await topics.gauge_captures.publish(capture)
    await asyncio.sleep(grace2)
    # start_stream()


class CoroutineControl(Control):
    def __init__(self, action):
        self.action = action
        self.task = None
        self.started = False

    def update(self, t: float, state: State):
        if not self.started:
            self.started = True
            self.task = start_task(self.action)

    def end(self, t: float, state) -> bool:
        return self.started and self.task.done()


class ImageCaptureControl(Control):
    # Stop streaming when capturing still image.
    # Blocks the event loop if action is blocking (e.g. camera streaming start/stop)

    def __init__(self, action):
        self.task = None
        self.started = False
        self.action = action
        self._end = False

    def update(self, t: float, state: State):
        if not self.started:
            stop_stream()
            self.started = True
            self.task = start_task(self.action)
        elif self.task.done() and not self._end:
            self._end = True
            start_stream()

    def end(self, t: float, state) -> bool:
        return self._end
