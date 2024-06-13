from __future__ import annotations

import math
from datetime import time
from enum import IntEnum
from functools import reduce
from typing import Dict, Optional, Type

from dataclasses import dataclass

# All NMEA sentences described here:
# https://gpsd.gitlab.io/gpsd/NMEA.html
# http://www.nvs-gnss.com/support/documentation/item/download/96.html

# http://navspark.mybigcommerce.com/content/NMEA_Format_v0.1.pdf
# online decode with map: https://rl.se/gprmc


def to_float(s: str) -> Optional[float]:
    try:
        return float(s)
    except ValueError:
        return None


def to_int(s: str) -> Optional[int]:
    try:
        return int(s)
    except ValueError:
        return None


def parse_lat(dms: str, a: str) -> Optional[float]:
    if not dms and not a:
        return None
    d = int(dms[0:2])
    m = float(dms[2:])
    lat = d + m / 60.0
    if a == "S":
        lat = -lat
    return lat


def _parse_time(hms: str) -> Optional[float]:
    if not hms:
        return None
    h = int(hms[0:2])
    m = int(hms[2:4])
    s = float(hms[4:])
    return ((h * 60 + m) * 60) + s


def parse_time(hms: str) -> Optional[time]:
    if not hms:
        return None
    h = int(hms[0:2])
    m = int(hms[2:4])
    ss = float(hms[4:])
    s = int(math.floor(ss))
    us = int((ss - s) * 1e6)
    return time(h, m, s, us)  # , timezone.utc)


def parse_lon(dms: str, a: str) -> Optional[float]:
    if not dms and not a:
        return None
    d = int(dms[0:3])
    m = float(dms[3:])
    lon = d + m / 60.0
    if a == "W":
        lon = -lon
    return lon


def checksum(message: str) -> str:
    """message is NMEA sentence after $ and up to but excluding the '*' before the checksum"""
    _checksum = reduce(lambda cs, c: cs ^ ord(c), message, 0)
    return f"{_checksum:02X}"


def _check_sentence(line: str) -> bool:
    if line[0] != "$":
        raise ValueError("Invalid NMEA sentence: " + line)
    message, _checksum = line[1:].split("*")
    return checksum(message) == _checksum


@dataclass
class FromGPS(object):
    nmea: str

    @staticmethod
    def parse(nmea: str, segments) -> FromGPS:
        raise NotImplemented


class Quality(IntEnum):
    """
    GPS Quality Indicator (non null)
    0 - fix not available,
    1 - GPS fix,
    2 - Differential GPS fix (values above 2 are 2.3 features)
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated (dead reckoning)
    7 = Manual input mode
    8 = Simulation mode
    """

    NO_FIX = 0
    GPS_FIX = 1
    DIFF_GPS_FIX = 2
    PPS_FIX = 3
    RTK = 4
    FLOAT_RTK = 5


@dataclass
class GGA(FromGPS):
    time: Optional[float] = None  # seconds since midnight
    lat: Optional[float] = None
    lon: Optional[float] = None
    quality: Optional[int] = None
    sats: Optional[int] = None
    hdop: Optional[float] = None
    alt: Optional[float] = None

    @staticmethod
    def parse(nmea: str, segments) -> GGA:
        # datetime.time is not JSON serializable
        # TODO: set proper absolute time including date
        _time = _parse_time(segments[1])
        lat = parse_lat(segments[2], segments[3])
        lon = parse_lon(segments[4], segments[5])
        quality = to_int(segments[6])
        sats = to_int(segments[7])
        hdop = to_float(segments[8])
        alt = to_float(segments[9])
        return GGA(nmea, _time, lat, lon, quality, sats, hdop, alt)


def dms(x):
    d = int(x)
    m = 60 * (x - d)
    return d, m


def gga_nmea(lat, lon, verb="GNGGA"):
    """
    Generate a GGA NMEA sentence for lat, lon (with fake time, satellites, altitude, etc).
    Primarily for providing sufficient request to NTRIP server.
    """
    assert 0 <= lat <= 90 and 0 <= lon < 180
    lat = "%02d%7.5f" % dms(lat)
    lon = "%03d%7.5f" % dms(lon)
    nmea = f"{verb},010000,{lat},N,{lon},E,4,12,0.59,192.9,M,39.4,M,5,0"
    return f"${nmea}*{checksum(nmea)}"


@dataclass
class VTG(FromGPS):
    course: Optional[float]
    speed: Optional[float]

    @staticmethod
    def parse(nmea: str, segments) -> VTG:
        course = to_float(segments[1])  # True north
        speed = to_float(segments[7])  # km/h
        return VTG(nmea, course, speed)


@dataclass
class RMC(FromGPS):
    time: Optional[float]  # seconds since midnight
    status: str
    lat: Optional[float]
    lon: Optional[float]
    hdop: Optional[float]
    speed_knots: Optional[float]
    course_over_ground: Optional[float]
    date: str
    mode: str

    @staticmethod
    def parse(nmea: str, segments) -> RMC:
        assert len(segments) == 14
        _time = _parse_time(segments[1])
        status = segments[2]
        lat = parse_lat(segments[3], segments[4])
        lon = parse_lon(segments[5], segments[6])
        hdop = float(segments[7])
        speed_knots = to_float(segments[7])
        course_over_ground = to_float(segments[8])
        date = segments[9]  # "ddmmyy"
        mode = segments[12]  # "R" for RTK fix, "F" for floating RTK
        return RMC(nmea, _time, status, lat, lon, hdop, speed_knots, course_over_ground, date, mode)

    def has_rtk(self):
        # - or - adjust tracking filter for float RTK ("F") - include floating mode?
        return self.mode == "R" or self.mode == "F"


_sentences = {
    "GNGGA": GGA,
    "GNVTG": VTG,
    "GNRMC": RMC,
    "GPGGA": GGA,
    # "GPVTG": VTG,
    # "GPRMC": RMC,
}  # type: Dict[str, Type[FromGPS]]


def process(line: str) -> FromGPS:
    line = line.strip()
    if not _check_sentence(line):
        raise ValueError("Invalid NMEA checksum from GPS: " + line)
    segments = line[1:].split(",")
    cmd = segments[0]
    clz = _sentences.get(cmd)
    if clz:
        return clz.parse(line, segments)
    return FromGPS(line)
