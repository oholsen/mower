import asyncio
import logging

from edge_control.config import gps_config
from edge_control.gps.driver import gps_driver
from edge_control.gps.site import world_to_site

logger = logging.getLogger(__name__)


async def run():
    asyncio.create_task(gps_driver(gps_config))
    await world_to_site()


def main():
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(run(), debug=False)


if __name__ == "__main__":
    main()
