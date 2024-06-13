"""
GPS client that receives RTK corrections from a NTRIP service.
"""

import asyncio
import base64
import logging
import time
from typing import Optional

from dataclasses import dataclass

from ..topics import gps_command, gps_position
from ..util.tasks import retry, start_tasks
from . import messages
from .status import GpsStatus

logger = logging.getLogger(__name__)


@dataclass
class NtripConfig:
    server: str
    port: int
    mount_point: str
    username: Optional[str]
    password: str = ""
    interval: float = 10  # seconds
    site: bool = False


def _create_ntrip_header(config: NtripConfig) -> str:
    header = (
        f"GET /{config.mount_point} HTTP/1.0\r\n"
        + "Accept: */*\r\n"
        + "Ntrip-Version: Ntrip/2.0\r\n"
        + "User-Agent: Robot\r\n"
        + "Connection: close\r\n"
    )
    if config.username:
        token = base64.b64encode(f"{config.username}:{config.password}".encode("ascii")).decode("ascii")
        header += f"Authorization: Basic {token}\r\n"
    return header + "\r\n"


async def ntrip_client(config: NtripConfig):
    # topics gps_position? -> gps_command

    async def _connection():

        logger.info("Connecting to NTRIP service %s:%d", config.server, config.port)
        reader, writer = await asyncio.open_connection(config.server, config.port)
        logger.info("Connected to NTRIP service")

        header = _create_ntrip_header(config)
        writer.write(bytes(header, "ascii"))
        await writer.drain()

        data = await reader.read(1024)
        response = str(data, "ascii")
        logger.debug("Header response: %r", response)
        if not response.startswith("HTTP/1.0 200 OK"):
            raise RuntimeError("Invalid response from NTRIP service: " + str(response))

        async def request(nmea: str):
            logger.debug("Write %s", nmea)
            writer.write(bytes(nmea + "\r\n", "ascii"))
            await writer.drain()

        async def responses():
            # NTRIP serves a continuous stream of corrections
            while True:
                correction = await reader.read(1024)
                logger.debug("Read %d bytes", len(correction))
                await gps_command.publish(correction)
                GpsStatus.corrections.inc()

        async def stream_gps_position():
            t_last: float = 0
            async for gga in gps_position.stream():

                # No point in sending bogus position to the NTRIP service.
                # Could figure out how sensitive the RTK corrections is to local position,
                # could possibly use the site reference position when no GPS fix.
                # if gga.quality == messages.Quality.NO_FIX:
                if gga.lat is None or gga.lon is None:
                    continue

                # Throttle NTRIP request rate - GPS rate can be high.
                t = time.time()
                if t < t_last + config.interval:
                    continue
                t_last = t
                nmea = messages.gga_nmea(gga.lat, gga.lon, "GPGGA")
                await request(nmea)

        async def stream_site_reference():
            from edge_control.config import site_config

            pos = site_config.reference
            assert pos
            nmea = messages.gga_nmea(pos.latitude, pos.longitude, "GPGGA")
            while True:
                await asyncio.sleep(config.interval)
                await request(nmea)

        requests = stream_site_reference if config.site else stream_gps_position
        # TODO: test if one fails, still hangs, does not cancel/stop the other?
        await start_tasks((requests(), responses()))

    # await _connection()
    await retry(_connection, 5)
