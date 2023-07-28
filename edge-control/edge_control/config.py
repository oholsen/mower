from __future__ import annotations

import cmath
import json
import math
from enum import Enum
from typing import List, Optional, Tuple

import shapely.ops
from dataclasses import dataclass, field
from shapely.geometry import Point, Polygon

from .gps.coordinates import UTM, LatLon
from .gps.ntrip import NtripConfig
from .map import geojson
from .util.config import filepath, read_config


class RobotType(Enum):
    simulation = 1
    hagedag = 2
    husqvarna = 3
    roomba = 4
    jackal = 5


@dataclass
class MowerConfig:
    cut_diameter: float  # m
    cut_power: float  # PWM duty cycle [-1, 1]


@dataclass
class DockConfig:
    position: Vector2D
    heading: float  # radians
    distance: float  # meters
    distance2: float  # meters

    def waypoint(self) -> Vector2D:
        # reversing distance out of dock
        p = self.position.complex() - cmath.rect(self.distance, self.heading)
        return Vector2D(p.real, p.imag)

    def waypoint2(self) -> Vector2D:
        # reversing distance2 out of dock
        p = self.position.complex() - cmath.rect(self.distance2, self.heading)
        return Vector2D(p.real, p.imag)


@dataclass
class CameraConfig:
    pan_offset: float  # degrees
    tilt_offset: float  # degrees


@dataclass
class RealSenseConfig:
    position: Vector3D  # front center of module
    # direction: Quaternion
    reference_tags: List[int]
    driver: str = "driver1"
    depth: bool = True


@dataclass
class SerialConfig:
    port: str
    baud_rate: int = 115_200


@dataclass
class SiteReferenceConfig:
    latitude: float
    longitude: float
    rotation: float = 0.0  # degrees
    # The rotation is the "compass course" of the desired site y axis.
    # Rotates the N/E coordinates clockwise around the reference point to get site coordinates.

    _utm0: UTM = field(init=False)

    def __post_init__(self):
        self._utm0 = LatLon(self.latitude, self.longitude).utm()

    def to_site(self, lat: float, lon: float, _height: float = 0) -> Tuple[float, float]:
        return self._to_site(LatLon(lat, lon).utm())

    def _to_site(self, utm: UTM) -> Tuple[float, float]:
        # Convert to site coordinates (x,y) in meters wrt reference and rotation.
        # Rotation is anti-clockwise from East
        p = complex(*utm.diff(self._utm0)) * cmath.rect(1, math.radians(self.rotation))
        return p.real, p.imag

    def to_world(self, x: float, y: float) -> UTM:
        # Convert site to world coordinates.
        # Rotation is clockwise from positive x-axis
        # Stays within the same UTM zone.
        p = complex(x, y) * cmath.rect(1, math.radians(-self.rotation))
        return self._utm0.add(p.real, p.imag)


@dataclass
class Vector2D:
    x: float
    y: float

    def complex(self):
        return complex(self.x, self.y)


@dataclass
class Vector3D(Vector2D):
    z: float


@dataclass
class SiteCoordinate(Vector2D):
    comment: Optional[str]


@dataclass
class TagPosition(SiteCoordinate):
    z: float


@dataclass
class Tag:
    id: int
    position: TagPosition
    # heading/rotation of tag


@dataclass
class TagConfig:
    tags: List[Tag]

    @staticmethod
    def load(filename: str = "tags.yaml") -> TagConfig:
        return read_config(filename, TagConfig)

    def get(self, tag_id: int) -> Optional[Tag]:
        for tag in self.tags:
            if tag.id == tag_id:
                return tag
        return None


@dataclass
class SiteConfig:
    reference: Optional[SiteReferenceConfig]  # required for GPS tracking

    geojson: Optional[str]  # Lon, Lat world coordinates (in that order), replaces

    # exterior and interiors in site coordinates - either configured (without reference), or
    # set up from geojson after mapping from world coordinates.
    exterior: List[SiteCoordinate] = field(default_factory=list)
    interiors: List[List[SiteCoordinate]] = field(default_factory=list)

    # GPS quality requirements depends on site - one site (or mission) may require cm resolution, another m.
    # Embed hdop in tracking (State) and filter in mission runner (mission config?).
    gps_quality: int = 0  # min threshold for GPS quality, ref GGA.quality
    gps_hdop: float = 0.1  # max hdop to accept for site coordinates

    # How to get into the dock. Mission config?
    # Multiple docking stations in a site, and refer
    # to the home base here. It starts to get interesting if a site have multiple compatible docking stations.
    dock: DockConfig = DockConfig(Vector2D(0, 0), 0, 0.4, 0.8)

    shape: Polygon = field(init=False)  # in site coordinates

    def on_site(self, x: float, y: float, buffer: float):
        # more efficient to buffer shape (on to many) than to buffer point (many to many)?
        return self.shape.buffer(-buffer).contains(Point(x, y))
        # return self.shape.contains(Point(x, y).buffer(buffer))

    @staticmethod
    def load(filename: str = "site.yaml") -> SiteConfig:
        sc = read_config(filename, SiteConfig)
        # TODO: separate map and mapping config? So that this path magic is not needed....
        if sc.geojson:
            # load geojson in world coordinates from same directory as SiteConfig and
            # map to exterior and interiors in site coordinates
            with filepath(filename).parent.joinpath(sc.geojson).open() as file:
                feature = json.load(file)
            shape = geojson.to_shape(feature)
            if sc.reference:

                def t(lon: float, lat: float, z=None):
                    assert sc.reference
                    return sc.reference.to_site(lat, lon)

                shape = shapely.ops.transform(t, shape)
            sc.shape = shape
            sc.exterior = [SiteCoordinate(x, y, None) for x, y in shape.exterior.coords]
            sc.interiors = [[SiteCoordinate(x, y, None) for x, y in interior.coords] for interior in shape.interiors]
        else:
            # create shape from configured exterior and interiors
            exterior = [(p.x, p.y) for p in sc.exterior]
            interiors = [[(p.x, p.y) for p in interior] for interior in sc.interiors]
            sc.shape = Polygon(exterior, interiors)
        return sc


@dataclass(frozen=True)
class ControlConfig:
    speed: float = 0.3
    omega: float = 0.2
    theta_distance: float = 0.2
    speed_theta: float = 0.25
    speed_relax: float = 2.0
    speed_overshoot: float = 0.02
    omega_relax: float = 1.0
    omega_overshoot: float = 0.01
    proximity: float = 1.0

    def validate(self):
        assert self.speed > 0, "Mission speed must be positive"
        assert self.omega > 0, "Mission omega must be positive"
        assert self.speed <= robot_config.max_speed, "Robot max speed exceeded"
        assert self.omega <= robot_config.max_omega, "Robot max omega exceeded"

    def angle_to_line(self, distance: float):
        # angle to line at distance from line
        return (math.pi / 2) * (1 - math.exp(-((distance / self.theta_distance) ** 2)))

    def speed_from_distance(self, distance: float):
        # relax towards distance=0, avoid large overshoot
        return min(distance / self.speed_relax + self.speed_overshoot, self.speed)

    def speed_from_angle(self, angle: float):
        # slow down for large angle
        return self.speed * math.exp(-((angle / self.speed_theta) ** 2))

    def omega_from_angle(self, angle: float):
        # relax towards desired theta, ovoid large overshoot
        return math.copysign(min(abs(angle) / self.omega_relax + self.omega_overshoot, self.omega), angle)


@dataclass(frozen=True)
class OnSiteConfig:
    # Only run missing while the robot is on site, pause mission while outside site
    enabled: bool = True

    # interval (s) between each check - to reduce CPU load
    interval: float = 1.0

    # buffer distance (m) to site boundary
    buffer: float = 0.2


@dataclass(frozen=True)
class MissionConfig:
    # For mowing, may want to have multiple AOIs to be reused across missions
    aoi: Optional[List[SiteCoordinate]]  # any shape
    control: ControlConfig
    on_site: OnSiteConfig = OnSiteConfig()

    @staticmethod
    def load(filename: str = "mission.yaml") -> MissionConfig:
        return read_config(filename, MissionConfig)

    def validate(self):
        self.control.validate()


@dataclass
class RoombaConfig:
    # actual speed is different from commanded, scale > 1 for higher actual speed
    wheel_scale_left: float = 1.0
    wheel_scale_right: float = 1.0


@dataclass
class RobotConfig:
    id: str
    name: str
    type: str  # RobotType
    wheel_base: float  # m - to check if combined (speed, omega) motor speed exceeds max motor speed
    max_speed: float  # m/s - absolute value per motor in each direction
    max_omega: float  # rad/s - TODO: remove, implicit with wheel_base and max_speed
    battery_cutoff: float  # volts

    # CDF project, should also have cluster
    project: Optional[str]

    realsense: Optional[RealSenseConfig]
    roomba: Optional[RoombaConfig]
    mower: Optional[MowerConfig]
    motor_control: Optional[SerialConfig]
    camera: CameraConfig = CameraConfig(0.0, 0.0)
    ip_address: str = "0.0.0.0"

    @staticmethod
    def load(filename: str = "robot.yaml") -> RobotConfig:
        return read_config(filename, RobotConfig)


@dataclass
class GpsConfig:
    gps: Optional[SerialConfig] = None
    ntrip: Optional[NtripConfig] = None

    # "nmea" or "ublox"
    mode: str = "nmea"

    # offset of GPS wrt robot base reference origin (x forwards, y left)
    offset: Vector2D = Vector2D(0, 0)

    @staticmethod
    def load(filename: str = "gps.yaml") -> GpsConfig:
        return read_config(filename, GpsConfig)


@dataclass
class SimulationConfig:
    position_sigma: float = 0

    @staticmethod
    def load(filename: str = "simulation.yaml") -> SimulationConfig:
        return read_config(filename, SimulationConfig)


def load_optional(clz, filename=None):
    try:
        return clz.load(filename) if filename else clz.load()
    except FileNotFoundError:
        return None


def load_default(clz, filename=None):
    try:
        return clz.load(filename) if filename else clz.load()
    except FileNotFoundError:
        return clz()


# Configuration singletons
robot_config = RobotConfig.load()
site_config = SiteConfig.load()
mission_config = MissionConfig.load()
mission_config.validate()
gps_config: Optional[GpsConfig] = load_optional(GpsConfig)
tag_config: Optional[TagConfig] = load_optional(TagConfig)
simulation_config: Optional[SimulationConfig] = load_optional(SimulationConfig)
