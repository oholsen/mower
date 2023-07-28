from __future__ import annotations

from typing import Tuple

import utm
from dataclasses import dataclass


@dataclass
class UTM:
    easting: float
    northing: float
    zone_number: int
    zone_letter: str

    def diff(self, u0: UTM) -> Tuple[float, float]:
        # -> deasting, dnorthing
        if self.zone_number != u0.zone_number:
            raise ValueError("Zone number %r != %r" % (self.zone_number, u0.zone_number))
        if self.zone_letter != u0.zone_letter:
            raise ValueError("Zone letter %r != %r" % (self.zone_letter, u0.zone_letter))
        return self.easting - u0.easting, self.northing - u0.northing

    def add(self, deasting: float, dnorthing: float) -> UTM:
        # Stays within the same UTM zone.
        return UTM(
            self.easting + deasting,
            self.northing + dnorthing,
            self.zone_number,
            self.zone_letter,
        )

    def latlon(self) -> LatLon:
        return LatLon(*utm.to_latlon(self.easting, self.northing, self.zone_number, self.zone_letter))


class LatLon(object):
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon

    def utm(self) -> UTM:
        return UTM(*utm.from_latlon(self.lat, self.lon))
