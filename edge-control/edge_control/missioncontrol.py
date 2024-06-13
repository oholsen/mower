import asyncio
import logging
import time
from datetime import timedelta
from typing import Optional

from . import mission, topics
from .config import mission_config, site_config
from .control import Control
from .models.messages import MissionAbort, MissionStart
from .models.state import State
from .robot import RobotState

logger = logging.getLogger(__name__)


def log(t: float, state: State, speed: float, omega: float):
    logger.debug(
        "state %.3f %.3f %.3f %.3f %.3f %.3f",
        t,
        state.x,
        state.y,
        state.theta,
        speed,
        omega,
    )


async def realtime_control(control: Control, name=None):
    from shapely.geometry import Point

    from .models.messages import MoveCommand, StopCommand

    move_command = StopCommand()  # type: MoveCommand
    start_time = time.time()
    on_site_time = 0.0
    on_site = True
    await mission.start(start_time, name)
    try:
        # use timeout to stop robot when there is no state - is (should be) embedded in arch driver
        async for state in topics.robot_tracking.stream_timeout(timeout=1.0):
            t = time.time()
            logger.debug("Control input %r %s", t, state)

            if state is None:
                logger.warning("Control paused without state")
                await topics.robot_command.publish(StopCommand())
                continue

            # TODO: embed hdop in state (in tracker) - add to buffer.
            # Missions are different: go from A to B without crashing (more slack) and mowing mission following
            # exactly a path to cover area.
            # [Stop mowing when hdop large - or - register mowed area only when hdop is low].
            if mission_config.on_site.enabled and t >= on_site_time + mission_config.on_site.interval:
                on_site_time = t
                on_site = site_config.on_site(state.x, state.y, mission_config.on_site.buffer)

            if not on_site:
                # May also mask out e.g. DockCommand!
                logger.warning("Control paused while outside site: %.3f %.3f", state.x, state.y)
                await topics.robot_command.publish(StopCommand())
                continue

            # Does not check on timeout. Could request current speed, omega from robot.
            log(t - start_time, state, move_command.speed, move_command.omega)

            # Check for any faults in the robot state - applying consistency rules
            fault = RobotState.fault(t)
            if fault:
                logger.warning(fault)
                continue

            if control.end(t, state):
                break

            # update control if everything is ok with the robot.
            # otherwise, the robot move commands will time out and the robot stops (within two seconds or so).
            command = control.update(t, state)
            if not command:
                continue
            if hasattr(command, "timeout"):
                # already asserted above in RobotState.fault(t)
                assert RobotState.time is not None, "No robot time"
                command.timeout = RobotState.time.time + 2
            logger.debug("Control command: %r", command)
            await topics.robot_command.publish(command)
            if isinstance(command, MoveCommand):
                move_command = command

        stop_time = time.time()
        mission_time = stop_time - start_time
        logging.info("Mission %s completed in %s", control, timedelta(seconds=mission_time))
        await mission.complete(stop_time)
        return
    except asyncio.CancelledError:
        stop_time = time.time()
        mission_time = stop_time - start_time
        logging.warning("Mission %s cancelled in %s", control, timedelta(seconds=mission_time))
        await mission.abort(stop_time, "cancelled")
        await topics.robot_command.publish(StopCommand())
    except Exception as e:
        logger.exception("Mission error")
        stop_time = time.time()
        mission_time = stop_time - start_time
        logging.warning(
            "Mission %s error %s in %s",
            control,
            str(e),
            timedelta(seconds=mission_time),
        )
        await mission.abort(stop_time, str(e))
        await topics.robot_command.publish(StopCommand())

    # Rely on move timeout in motor driver when mission fails badly or doesn't explicitly stop/dock.
    # Stopping while docking aborts the docking built-in mode on Roomba.
    # TODO: Implement end() in DockCommand to wait for the robot to dock. Then we can stop here always.


_mission_in_progress: Optional[asyncio.Task] = None


async def mission_start(name: str):
    from .missions import get_mission

    global _mission_in_progress

    logger.info("Mission start: %s", name)
    if _mission_in_progress is not None and not _mission_in_progress.done():
        logger.warning("Mission start %s ignored - mission already in progress", name)
        _mission_in_progress.print_stack()
        return

    controls = await get_mission(name)

    logger.info("Starting mission %s", name)
    _mission_in_progress = asyncio.create_task(realtime_control(controls, name=name))


async def mission_abort():
    global _mission_in_progress

    logger.warning("Abort mission: %r", _mission_in_progress)
    if _mission_in_progress and not _mission_in_progress.done():
        logger.info("Aborting mission")
        _mission_in_progress.print_stack()
        _mission_in_progress.cancel()


async def mission_control():
    async for command in topics.mission_command.stream():
        logger.info("Mission command: %s", command)
        if isinstance(command, MissionStart):
            await mission_start(command.name)
        elif isinstance(command, MissionAbort):
            await mission_abort()
