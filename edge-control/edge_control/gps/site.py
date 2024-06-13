import logging

from .. import topics
from ..config import site_config
from ..models.messages import SitePosition
from .messages import GGA
from .status import GpsStatus

logger = logging.getLogger(__name__)


async def world_to_site():

    logger.info("Starting world to site...")

    if not site_config.reference:
        logger.warning("World to site mapping has no world reference")
        return

    async for gga in topics.gps_position.stream():

        # logger.debug("GPS position: %s", gga)

        # No GPS data (yet)
        if gga.lat is None or gga.lon is None:
            logger.debug("GPS position ignored: %r %r", gga.lat, gga.lon)
            continue

        # Filter before feeding the Kalman tracking filter.
        # Could add the hdop as is to the EKF, but sometimes the reported hdop is consistently smaller (non-Gaussian)
        # than the actual error, which would mislead the EKF. Hence filter he

        # Filter on position quality
        if gga.quality < site_config.gps_quality:
            logger.debug("GPS position ignored with quality: %r", gga.quality)
            continue

        if gga.hdop > site_config.gps_hdop:
            logger.debug("GPS position ignored with hdop: %r", gga.hdop)
            continue

        try:
            x, y = site_config.reference.to_site(gga.lat, gga.lon)
        except AssertionError:
            logger.exception("Invalid site coordinate")
            continue
        logger.debug("Site %.3f %.3f %.3f", x, y, gga.hdop)
        s = SitePosition(x, y, gga.hdop)
        GpsStatus.site.set(s)
        await topics.site_position.publish(s)
