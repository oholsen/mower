import asyncio
import logging
from json import JSONDecodeError
from typing import AsyncGenerator

from ..config import robot_config, site_config
from ..models.turtle import Turtle
from .T265 import Frame

logger = logging.getLogger(__name__)
turtle = Turtle(site_config.dock.position.x, site_config.dock.position.y, site_config.dock.heading)


def repower() -> None:
    """
    # TODO: setuid on file
    sudo /home/oholsen/bin/uhubctl -l 1-2.2 -p 4 -a 2 # -d 4
    """
    # TODO: when to repower? Poll status? Only on startup?
    # cmd = "uhubctl"
    pass


async def frames() -> AsyncGenerator[Frame, None]:
    cmd = "rs-pose-apriltag"
    proc = await asyncio.create_subprocess_exec(cmd, stdout=asyncio.subprocess.PIPE)
    assert proc.stdout
    while True:
        data = await proc.stdout.readline()
        line = data.decode("ascii").strip()
        try:
            if not line:
                # source program has stopped?
                return
            yield Frame.from_json(line)
        except JSONDecodeError:
            # TODO: fix .cpp program to drop pose or make t, r optional for confidence 0
            # TODO: threshold on confidence - what to do then? Stop? When in dock, know position and heading, can make some
            # route to check for improved confidence. But that requires dead reckoning on odometry, i.e. broader sensor fusion.
            # '{"frame":277042,"timestamp":1606489924746,"pose":{"confidence":0,"t":[nan,nan,nan],"r":[nan,nan,nan,nan]}}'
            continue
        except Exception:
            logger.exception("Frame line: " + line)


async def driver():
    rs = robot_config.realsense
    assert rs, "No RealSense configuration"

    if rs.depth:
        from . import depth

        depth.start()

    assert rs.driver != "driver2", "driver2 need to fix Quaternion argument order"

    if rs.driver == "none":
        return

    elif rs.driver == "driver1":
        from .driver1 import driver as pos_driver

    elif rs.driver == "driver2":
        from .driver2 import driver as pos_driver

    elif rs.driver == "driver3":
        from .driver3 import driver as pos_driver

    else:
        raise ValueError("Invalid RealSense driver: " + rs.driver)

    await pos_driver()
