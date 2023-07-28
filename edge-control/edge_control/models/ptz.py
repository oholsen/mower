from typing import Dict, Optional

from dataclasses import dataclass, field


@dataclass(frozen=True)
class ToPanTiltZoom:
    pass


@dataclass(frozen=True)
class PanTiltZoomSpeed(ToPanTiltZoom):
    pan: Optional[float]
    tilt: Optional[float]
    zoom: Optional[float]


@dataclass(frozen=True)
class PanTiltZoomPosition(ToPanTiltZoom):
    pan: float
    tilt: float
    zoom: float


@dataclass(frozen=True)
class Snapshot(ToPanTiltZoom):
    filename: str
    # TODO: metadata such as position++. Store in .jpeg or externally.

    async def completed(self):
        pass


@dataclass(frozen=True)
class StartStream(ToPanTiltZoom):
    pass


@dataclass(frozen=True)
class StopStream(ToPanTiltZoom):
    pass
