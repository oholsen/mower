import logging
from typing import Any, Dict, Optional

from . import mission, topics
from .arch.hagedag.status import HagedagStatus
from .arch.husqvarna.status import HusqvarnaStatus
from .arch.simulation.status import SimulationStatus
from .gps.status import GpsStatus
from .models.messages import Battery, ObstacleDetection, Time
from .models.state import State
# from .realsense.status import RealsenseStatus
from .util.status import Status
from .util.tasks import start_task

logger = logging.getLogger(__name__)


class RobotState:
    """Last recorded state - must check if it is current"""

    time: Optional[Time] = None
    battery: Optional[Battery] = None
    docked: bool = False
    state = Status[State]()
    obstacle_depth: Optional[ObstacleDetection] = None
    gps = GpsStatus
    # realsense = RealsenseStatus
    simulation = SimulationStatus
    hagedag = HagedagStatus
    husqvarna = HusqvarnaStatus

    @staticmethod
    def fault(t: float) -> Optional[str]:
        try:
            assert RobotState.time is not None, "No robot time"
            # RealsenseStatus.fault(t)
            # fault on all registered statuses, but must register per robot
            return None
        except AssertionError as e:
            return str(e)

    @staticmethod
    def as_dict(t: float) -> Dict[str, Any]:
        def _as_dict(o) -> Dict[str, Any]:
            res: Dict[str, Any] = {}
            if o is None:
                return res
            for name, value in vars(o).items():
                if name[0] == "_":
                    continue
                # check for Status at the top level only
                if isinstance(value, staticmethod):
                    continue
                if isinstance(value, Status):
                    value = value.get(t)
                    # logger.debug("STATUS %s %r", name, value)
                if hasattr(value, "__dict__") and value.__dict__:
                    # class wrappers around basic types (e.g. Infrared(int)) has empty __dict__
                    # also handles data classes
                    value = _as_dict(value)
                # elif is_dataclass(value): value = asdict(value)
                if value is not None and value != {}:
                    res[name] = value
            return res

        d = _as_dict(RobotState)
        d.update(mission=_as_dict(mission.get()))
        return d


async def _robot_state():
    async for message in topics.robot_state.stream():
        # logger.debug("robot_state message: %r", message)
        if isinstance(message, Time):
            RobotState.time = message


async def _tracking():
    async for state in topics.robot_tracking.stream():
        RobotState.state.set(state)


def start():
    logger.debug("Starting robot state...")
    start_task(_robot_state())
    start_task(_tracking())
