import asyncio
import logging

from edge_control.config import gps_config
from edge_control.gps.driver import gps_driver
from edge_control.gps.messages import GGA
from edge_control.gps.ntrip import ntrip_client
from edge_control.gps.site import world_to_site
from edge_control.topics import SitePosition, gps_position, site_position

logger = logging.getLogger(__name__)


async def run():
    asyncio.create_task(gps_driver(gps_config.gps))
    asyncio.create_task(ntrip_client(gps_config.ntrip))
    if 0:
        gga: GGA
        async for gga in gps_position.stream():
            logger.info(
                "GPS position: %s %d %f %.6f %.6f",
                gga.time.isoformat("milliseconds"),
                gga.quality,
                gga.hdop,
                gga.lat,
                gga.lon,
            )
    else:
        asyncio.create_task(world_to_site())
        pos: SitePosition
        async for pos in site_position.stream():
            logger.info("Site position: %s %.2f %.2f %.2f", pos, pos.x, pos.y, pos.hdop)


def main():
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(run(), debug=True)


if __name__ == "__main__":
    main()
