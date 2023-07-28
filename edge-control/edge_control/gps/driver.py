"""
GPS client that receives RTK corrections from a NTRIP service.
"""

import logging

import aioserial

from edge_control import topics
from edge_control.config import GpsConfig

from ..util.tasks import retry, start_task, start_tasks
from . import nmea_driver, ubx_driver
from .ntrip import ntrip_client
from .status import GpsStatus

logger = logging.getLogger(__name__)


async def gps_driver(config: GpsConfig):
    async def _connection():
        logger.info("Connecting to GPS %s", config.gps.port)
        serial = aioserial.AioSerial(config.gps.port, config.gps.baud_rate)
        logger.info("Connected to GPS")

        async def commands():
            async for command in topics.gps_command.stream():
                await serial.write_async(command)
                GpsStatus.commands.inc()

        tasks = [commands()]
        if config.ntrip:
            tasks.append(ntrip_client(config.ntrip))

        if config.mode == "ublox":
            tasks.append(ubx_driver.responses(serial))
        else:
            tasks.append(nmea_driver.parse_nmea())
            tasks.append(nmea_driver.responses(serial))

        start_tasks(tasks, "gps driver")

    await _connection()
    # await retry(_connection, 5)
