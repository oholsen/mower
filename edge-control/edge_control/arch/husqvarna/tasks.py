import asyncio
import logging

from edge_control.config import robot_config

from .driver import motor_driver

logger = logging.getLogger(__name__)


async def start():
    logger.debug("Starting husqvarna tasks...")
    assert robot_config.type == "husqvarna"
    asyncio.create_task(motor_driver())
