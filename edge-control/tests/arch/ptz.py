import asyncio
import logging

from edge_control import topics
from edge_control.arch.roomba.ptz import driver
from edge_control.config import SerialConfig
from edge_control.models.ptz import PanTiltZoomSpeed

logger = logging.getLogger(__name__)


async def main():
    asyncio.create_task(driver(SerialConfig("/dev/ttyACM0")))
    while True:
        for speed in [-10, 0, 10]:
            logger.info("Pan %g", speed)
            await topics.ptz_command.publish(PanTiltZoomSpeed(speed, None, None))
            await asyncio.sleep(2.0)
            logger.info("Tilt %g", speed)
            await topics.ptz_command.publish(PanTiltZoomSpeed(None, speed, None))
            await asyncio.sleep(2.0)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(main())
