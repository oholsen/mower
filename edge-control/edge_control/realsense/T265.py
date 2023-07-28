from __future__ import annotations

import json
import math
from typing import List, Optional, Tuple

import dacite
import numpy
from dataclasses import dataclass
from numpy import linalg
from numpy.typing import ArrayLike
from pyquaternion import Quaternion

from ..config import site_config

"""
Coordinate system (and origin) is defined when starting the realsense process. 
The translation vector: Z-axis goes backwards, X-axis to the right.
Rotation: positive yaw from to_euler() below is clockwise seen from above.

Define the site coordinate system wrt to the docking station.
"""

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High
pose_data_confidence_level = ("Failed", "Low", "Medium", "High")


@dataclass(frozen=True)
class Pose:
    confidence: int  # 0-3, 3 is best
    t: List[float]
    r: List[float]  # quaternion x,y,z,w

    def quaternion(self) -> Quaternion:
        # scalar/norm as first argument Quaternion(w, x, y, z)
        return Quaternion(self.r[3], self.r[0], self.r[1], self.r[2])

    def site_heading(self) -> float:
        return to_site_euler(self.quaternion())[2]

    def site_rotation(self) -> Tuple[float, float, float]:
        return to_site_euler(self.quaternion())

    def site_position(self) -> List[float]:
        return to_site_position(self.t)


@dataclass(frozen=True)
class TagPose:
    t: List[float]
    r: List[float]  # 9 elements

    def delta(self, t0: List[float]) -> ArrayLike:
        return numpy.array(self.t) - numpy.array(t0)


@dataclass(frozen=True)
class Tag:
    tagId: int

    # camera t is (right, down, away) wrt current camera pose
    camera: TagPose

    # world is only present if Pose.confidence is 3
    world: Optional[TagPose] = None


@dataclass(frozen=True)
class Frame:
    frame: int
    timestamp: int
    # either pose or tags, always pose before tags, not all poses have tags
    pose: Optional[Pose]
    tags: Optional[List[Tag]] = None

    @staticmethod
    def from_json(json_data: str) -> Frame:
        data = json.loads(json_data)
        return dacite.from_dict(data_class=Frame, data=data)


def horizontal_speed(frame0: Frame, frame1: Frame) -> float:
    # Can diff in realsense frame to get distance, no need to convert to site
    # Rule out speed/difference in vertical - but may want to track drift in vertical wrt tags.
    assert frame0.pose and frame1.pose
    dt = 0.001 * (frame1.timestamp - frame0.timestamp)
    delta = numpy.array(frame1.pose.t) - numpy.array(frame0.pose.t)
    delta[1] = 0
    return linalg.norm(delta) / dt


# Convert to site coordinates, push position every 500ms (as GPS) and calculate speed
# Realsense: origin at startup, X right, Y up, Z backward
# Site: origin at dock, X forward, Y left, Z up, theta is positive anti-clockwise from above (around Z).
# Dock may be slightly above floor
# Figure out how roll/pitch affects tracking camera sitting high, i.e. far away from center of mass/rotation


def to_site_position(t: List[float]) -> List[float]:
    # map from realsense to site
    p = site_config.dock.position
    return [p.x - t[2], p.y - t[0], t[1]]


def to_site_euler(q: Quaternion) -> Tuple[float, float, float]:

    # Get the equivalent yaw-pitch-roll angles aka. intrinsic Tait-Bryan angles following the z-y'-x'' convention
    # yaw, pitch, roll = q.yaw_pitch_roll
    # return pitch, roll, yaw

    # change from realsense to site
    w = q.w
    x = -q.z
    y = q.x
    z = -q.y

    a = w * w - y * y
    b = x * x - z * z
    pitch = -math.asin(2.0 * (x * z - w * y))
    roll = math.atan2(2.0 * (w * x + y * z), a - b)
    yaw = -math.atan2(2.0 * (w * z + x * y), a + b)
    return pitch, roll, yaw
