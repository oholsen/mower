import logging

import aioserial

from .. import topics
from . import messages
from .status import GpsStatus

logger = logging.getLogger(__name__)


async def parse_nmea():
    async for nmea in topics.gps_nmea.stream():
        try:
            m = messages.process(nmea)
        except ValueError:
            logger.exception("Parsing NMEA: " + nmea)
            return

        logger.debug("GPS message: %r", m)
        if isinstance(m, messages.GGA):
            GpsStatus.gga.set(m)
            await topics.gps_position.publish(m)


async def responses(gps: aioserial.AioSerial):
    while True:
        nmea = await gps.readline_async()
        GpsStatus.responses.inc()
        # logger.debug("serial: %r", nmea)
        nmea = nmea.decode("ascii").strip()
        logger.debug("NMEA %s", nmea)
        await topics.gps_nmea.publish(nmea)
