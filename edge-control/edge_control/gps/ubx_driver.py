"""
GPS client that receives RTK corrections from a NTRIP service.
"""

import logging

import aioserial

from edge_control import topics

from .messages import GGA, Quality
from .status import GpsStatus
from .ubx import NavPosLLH, Reader, decode

logger = logging.getLogger(__name__)


async def responses(gps: aioserial.AioSerial):
    reader = Reader()
    while True:
        data = await gps.read_async(reader.remaining())
        # logger.debug("serial: %s", data.hex())
        for frame in reader.frames(data):
            GpsStatus.responses.inc()  # responses, not necessarily positions
            msg = decode(frame)
            # logger.debug("ubx %s", msg)
            if isinstance(msg, NavPosLLH):
                # TODO: Define WorldPosition in models
                gga = GGA(
                    "",
                    lat=msg.latitude(),
                    lon=msg.longitude(),
                    quality=Quality.GPS_FIX,
                    hdop=msg.hdop(),
                    alt=msg.altitude(),
                )
                await topics.gps_position.publish(gga)
