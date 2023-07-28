"""
Store robot tracking information per session such that it can be presented in a GUI.
"""

import logging
from typing import List

from edge_control import topics
from edge_control.models.state import State

logger = logging.getLogger(__name__)
_storage = []  # type: List[State]


async def store():
    async for msg in topics.robot_tracking.stream():
        _storage.append(msg)


def get():
    return _storage
