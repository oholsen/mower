import asyncio
import logging
import sys

from edge_control import topics
from edge_control.arch.hagedag.driver import motor_driver
from edge_control.models.messages import MoveCommand, StopCommand

logger = logging.getLogger(__name__)


async def run_robot_timeout():
    from edge_control.robot import RobotState

    speed = 0.2
    omega = 0
    while True:
        if RobotState.time is None:
            logger.warning("No robot time")
        else:
            logger.info("Move: %g", speed)
            m = MoveCommand(RobotState.time.time + 2.0, speed, omega)
            await topics.robot_command.publish(m)

        # should timeout and stop during this sleep
        await asyncio.sleep(5)


async def run_robot_start_stop():
    # cut speed from robot config!

    omega = 0
    speed = 0.2
    await asyncio.sleep(3)

    while True:
        m = MoveCommand(0, speed, omega)
        logger.info("Move: %s", m)
        await topics.robot_command.publish(m)
        await asyncio.sleep(3)

        m = MoveCommand(0, 0, omega)
        logger.info("Move: %s", m)
        await topics.robot_command.publish(m)
        await asyncio.sleep(3)


async def run_robot_speed():
    # cut speed from robot config!

    omega = 0
    await asyncio.sleep(1)
    for speed in scan(0, 0.34, 0.1):
        m = MoveCommand(0, speed, omega)
        logger.info("Move: %s", m)
        await topics.robot_command.publish(m)
        await asyncio.sleep(3)


async def run_robot_omega():
    # cut speed from robot config!

    speed = 0.1
    await asyncio.sleep(1)
    for omega in scan(0, 0.20, 0.02):
        m = MoveCommand(0, speed, omega)
        logger.info("Move: %s", m)
        await topics.robot_command.publish(m)
        await asyncio.sleep(1)


async def run_robot_move(speed, omega):
    # cut speed from robot config!

    while True:
        m = MoveCommand(0, speed, omega)
        logger.info("Move: %s", m)
        await topics.robot_command.publish(m)
        await asyncio.sleep(2)


async def run_robot_start_arc():
    # cut speed from robot config!

    await asyncio.sleep(1)
    if 0:
        m = MoveCommand(0, speed=0, omega=0)
        logger.info("Move: %s", m)
        await topics.robot_command.publish(m)
        await asyncio.sleep(2)

    while True:
        m = MoveCommand(0, speed=0.017499999999999998, omega=0.35)
        logger.info("Move: %s", m)
        await topics.robot_command.publish(m)
        await asyncio.sleep(5)


async def run():
    sys.argv.pop(0)
    asyncio.create_task(motor_driver())

    if sys.argv and sys.argv[0] == "stop":
        await topics.robot_command.publish(StopCommand())
        await asyncio.sleep(2)
        return

    if len(sys.argv) > 2 and sys.argv[0] == "move":
        speed = float(sys.argv[1])
        omega = float(sys.argv[2])
        await run_robot_move(speed, omega)
        return

    # test motor topics
    # await run_motor()
    # await run_motor_start_stop()

    # test robot topics
    # a wait run_robot_start_arc()
    # await run_robot_timeout()

    # asyncio.create_task(dump("Robot state", robot_state))
    # await run_robot_start_stop()
    # await run_robot_speed()
    # await run_robot_omega()

    while True:
        await asyncio.sleep(60)


def main():
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(run(), debug=False)


if __name__ == "__main__":
    main()
