"""
Communication against the Arduino Nano and the motor controllers.
Feeds motor_command -> Arduino -> motor_state.
"""

import asyncio
import logging

import aioserial

from edge_control import topics
from edge_control.config import robot_config
from edge_control.models.messages import CutCommand, MoveCommand, Odometry, StopCommand, Time

from .frame import frame, read_frames
from .messages import Heartbeat, MotorStop, ToMotorController, parse
from .motor import from_move, to_move
from .status import HusqvarnaStatus

logger = logging.getLogger(__name__)


async def motor_driver():

    while True:
        logger.info("Connecting to Arduino")
        device = robot_config.motor_control
        arduino = aioserial.AioSerial(device.port, device.baud_rate)
        logger.info("Connected to Arduino")

        async def write(command: ToMotorController):
            logger.debug("Write command: %s", command)
            data = frame(command.message())
            logger.debug("To arduino: %s %s", command, data.hex())
            await arduino.write_async(data)

        # TODO: wait for Heartbeat before sending commands - Arduino resets on connection and takes a while to start
        # await write(MotorStop())
        # await asyncio.sleep(1)
        # await write(MotorStop())

        async def commands():
            # initial cut power from config, can be modified with CutCommand, applied to every move command
            cut_power = robot_config.mower.cut_power
            async for command in topics.robot_command.stream():
                logger.debug("Command %r", command)
                if isinstance(command, StopCommand):
                    await write(MotorStop())
                elif isinstance(command, MoveCommand):
                    await write(from_move(command, cut_power))
                elif isinstance(command, CutCommand):
                    # applied at next MoveCommand
                    cut_power = command.power
                else:
                    logger.warning("Ignoring %s", command)

        _commands = asyncio.create_task(commands())

        async for t, message in read_frames(arduino):
            try:
                logger.debug("From arduino: %s", message.hex())
                m = parse(t, message)
                logger.debug("From arduino: %s", m)
                if isinstance(m, Heartbeat):
                    speed, omega = to_move(m)
                    logger.debug("Speed %.3f omega %.3f", speed, omega)
                    HusqvarnaStatus.speed.set(speed, t)
                    HusqvarnaStatus.omega.set(omega, t)
                    # guard against haywire motion
                    assert abs(speed) < 1
                    assert abs(omega) < 1.6
                    await topics.odometry.publish(Odometry(m.timestamp, speed, omega))
                    await topics.robot_state.publish(Time(m.timestamp, m.time))

            except ValueError:
                logger.exception("Parsing message: " + message.hex())
