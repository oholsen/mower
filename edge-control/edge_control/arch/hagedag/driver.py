"""
Communication against the STM32 motor controllers.
Feeds motor_command -> STM32 -> motor_state.
"""

import asyncio
import logging
import time

import aioserial

from edge_control import topics
from edge_control.config import robot_config
from edge_control.models import messages as msgs
from edge_control.util.tasks import retry, start_tasks

from . import messages
from .messages import ResetCommand, RobotCommand, Time, process
from .status import HagedagStatus

logger = logging.getLogger(__name__)

if robot_config.mower is not None:
    from . import cutter


def from_move(message: msgs.MoveCommand, cut_power: float) -> messages.MoveCommand:
    # v_rot = message.omega * robot_config.wheel_base / 2
    # v_left = message.speed - v_rot
    # v_right = message.speed + v_rot
    assert message.timeout > 0, "No timeout in MoveCommand"
    return messages.MoveCommand(message.speed, message.omega, message.timeout)


async def _connect():
    logger.info("Connecting to STM32")
    device = robot_config.motor_control
    with aioserial.AioSerial(device.port, device.baud_rate) as serial:
        logger.info("Connected to STM32")

        async def write(command: RobotCommand):
            data = (str(command) + "\n").encode("ascii")
            logger.debug("To STM32: %s", data)
            await serial.write_async(data)

        # Reset all state and set desired state. Will also reset Time.
        await write(ResetCommand())

        async def commands():
            # initial cut power from config, can be modified with CutCommand, applied to every move command
            cut_power = robot_config.mower.cut_power if robot_config.mower else 0

            async for message in topics.robot_command.stream():
                logger.debug("robot_to_motor message: %r", message)
                try:
                    if isinstance(message, msgs.StopCommand):
                        await write(messages.StopCommand())
                        cutter.cutter.stop()
                        await topics.odometry.publish(msgs.Odometry(time.time(), 0, 0))
                    elif isinstance(message, msgs.MoveCommand):
                        await write(from_move(message, cut_power))
                        await topics.odometry.publish(msgs.Odometry(time.time(), message.speed, message.omega))
                        if robot_config.mower is not None:
                            cutter.cutter.power(cut_power)
                            cutter.reset_timeout()
                    elif isinstance(message, msgs.CutCommand):
                        # applied at every MoveCommand to refresh timeout in RPi
                        cut_power = message.power
                        if robot_config.mower is not None:
                            cutter.cutter.power(cut_power)
                            cutter.reset_timeout()
                except (KeyboardInterrupt, asyncio.CancelledError):
                    raise
                except Exception:
                    logger.exception("commands")

        async def status():
            while True:
                line = await serial.readline_async()
                timestamp = time.time()
                line = line.decode("ascii").strip()
                # logger.debug("From STM32: %s", line)
                try:
                    m = process(line)
                except (KeyboardInterrupt, asyncio.CancelledError):
                    raise
                except Exception:
                    logger.exception("Parsing message: " + line)
                    continue

                # logger.debug("From STM32: %r %s", line, m)
                if m is None or isinstance(m, messages.Ignore):
                    continue

                HagedagStatus.reports.inc(timestamp)
                if isinstance(m, messages.Time):
                    await topics.robot_state.publish(msgs.Time(timestamp, m.time))
                elif isinstance(m, messages.Status):
                    HagedagStatus.status.set(m, timestamp)
                elif isinstance(m, messages.Battery):
                    HagedagStatus.battery.set(m, timestamp)
                    await topics.robot_state.publish(msgs.Battery(timestamp, m.voltage))

                # up to 1s lag, should report it much more often from STM32
                # elif isinstance(m, messages.Speed):
                #    await topics.odometry.publish(msgs.Odometry(timestamp, m.speed(), m.omega()))

        await start_tasks({commands(), status()})


async def motor_driver():
    logger.info("Starting hagedag driver")
    loop = asyncio.get_event_loop()
    if robot_config.mower is not None:
        cutter.start_watch_dog(loop)
    await retry(_connect, 2)
