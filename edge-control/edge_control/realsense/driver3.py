import cmath
import logging
import math
import time

import numpy as np

from .. import topics
from ..config import robot_config, site_config, tag_config
from ..models.tracking import ExtendedKalmanFilter, State, motion_model, position_model, tag_observation_model
from ..robot import RobotState
from ..util.math import a2s, norm_angle
from ..util.tasks import start_task
from ..util.time import now
from .physical import t265_offset_left
from .status import RealsenseStatus, Rotation, Vector
from .T265 import Tag

logger = logging.getLogger(__name__)
_ekf = ExtendedKalmanFilter(
    np.array([[site_config.dock.position.x], [site_config.dock.position.y], [site_config.dock.heading]]),
    np.diag([0.1, 0.1, 0.1]),  # in docking station vs mission...
)

_tags = dict((tag.id, tag) for tag in tag_config.tags) if tag_config else {}
if tag_config:
    assert len(_tags) == len(tag_config.tags), "Duplicate tag ids in tag config"


def position(ekf: ExtendedKalmanFilter, tag: Tag):
    logger.debug("Tag camera %s %s", tag.tagId, a2s(tag.camera.t))

    _tag = _tags.get(tag.tagId)
    if _tag is None:
        logger.debug("Ignoring unknown tag %s", tag)
        return
    p = _tag.position

    # ignore low tags, as duplicates on thin paper from Spot
    # if p.z < 0.7:
    #     logger.debug("Ignoring low tag %s", tag)
    #     return

    # tag.camera.t is right, down, forward
    # tag position in robot frame:
    assert robot_config.realsense
    tag_x = robot_config.realsense.position.x + tag.camera.t[2]
    tag_y = robot_config.realsense.position.y - tag.camera.t[0] + t265_offset_left
    tag_z = robot_config.realsense.position.z - tag.camera.t[1]

    dz = p.z - tag_z
    if abs(dz) > 0.2:
        logger.debug("Ignoring invalid tag height %.3f %s", dz, tag)
        return

    tag_camera = complex(tag_x, tag_y)
    # Ignore bad observations!? At extreme angles??? Check if distance makes sense wrt current state.
    assert ekf.x is not None
    tag_observation = complex(ekf.x[0, 0], ekf.x[1, 0]) + tag_camera * cmath.rect(1, ekf.x[2, 0])
    tag_position = complex(p.x, p.y)
    tag_delta = tag_observation - tag_position
    logger.debug("tag distance %.3f %s", abs(tag_delta), tag)
    if 0 and abs(tag_delta) > 0.3:
        logger.debug("Ignoring invalid tag distance %.3f %s", abs(tag_delta), tag)
        return

    z = np.array([[tag_x], [tag_y]])
    # Horizontal precision is better than depth/distance to tag.
    # Compensate for distortion in fisheye - increase R if angle gets close to pi/2 (by 3x).
    # Use higher R for larger distance to tag (at least for depth, but that also affects horizontal distance).
    # TODO: Have good angle estimate independent of distance estimate - covariance below is not diagonal.
    angle = math.atan2(tag_y, tag_x)
    r = np.diag([0.2, 0.1]) * math.exp((angle / 1.4) ** 2) * math.exp((abs(tag_camera) ** 2))
    ekf.correct(z, lambda x: tag_observation_model(x, tag_position), r)
    ekf.x[2, 0] = norm_angle(ekf.x[2, 0])


def docked(ekf: ExtendedKalmanFilter):
    assert site_config.dock.position

    # observe the robot at docking station position - with low error.
    # heading is unknown - although nearly the heading defined in the dock (non-zero mean).
    # TODO: should make sure that any position offset is not corrected as though the robot was driving there.
    logger.debug("docked")
    z = np.array([[site_config.dock.position.x], [site_config.dock.position.y]])
    r = np.diag([0.01, 0.01])
    observation_offset = np.array([[0], [0]])
    ekf.correct(z, lambda x: position_model(x, observation_offset), r)
    assert ekf.x is not None
    ekf.x[2, 0] = norm_angle(ekf.x[2, 0])


async def _tracker():
    from .driver import frames

    t_last: int = 0

    async for frame in frames():
        try:
            if not frame.pose:
                logger.debug("tags %s", frame)
                if frame.tags is not None:
                    for tag in frame.tags:
                        position(_ekf, tag)
                    # publish ekf state in odometry, to get a steady rate
                if RobotState.docked:
                    docked(_ekf)
                continue

            # Report T265 pose for reference at reasonably low rate (not spam logs).
            if frame.timestamp < t_last + 200:
                continue
            t_last = frame.timestamp
            lag = now() - frame.timestamp
            logger.debug("frame %d %s", lag, frame)
            pitch, roll, heading = frame.pose.site_rotation()
            x, y, z = frame.pose.site_position()
            logger.debug(
                "vpose %d %.3f %.3f %.3f %.1f %.1f %.1f",
                frame.pose.confidence,
                x,
                y,
                z,
                math.degrees(heading),
                math.degrees(pitch),
                math.degrees(roll),
            )

            RealsenseStatus.confidence.set(frame.pose.confidence)
            RealsenseStatus.position.set(Vector(x, y, z))
            RealsenseStatus.rotation.set(Rotation(pitch, roll, heading))

            # For now, replace the state estimate in the neighborhood of the docking station
            # where heading alignment (rotation) does not matter so much - to aid in precision docking.
            # TODO: add tracking observation to EKF with low covariance
            if frame.pose.confidence == 3:
                distance_to_dock = abs(complex(x, y) - site_config.dock.position.complex())
                if distance_to_dock < 3:
                    t = time.time()
                    RealsenseStatus.override = t
                    s = State(x, y, heading)
                    logger.debug("track override %.3f %.3f %.3f %.3f", t, x, y, heading)
                    await topics.robot_tracking.publish(s)
                    continue
            RealsenseStatus.override = 0.0  # trigger expiration

        except:
            logger.exception("Frame: %s" % frame)
            raise


async def _odometry():
    # Output topic: robot_tracking
    t_last = 0.0
    async for odometry in topics.odometry.stream():
        # logger.debug("Odometry %s", odometry)
        if t_last > 0:
            dt = odometry.time - t_last
            logger.debug("odometry %.4f %s", dt, odometry)
            u = np.array([[odometry.speed], [odometry.omega]])
            _ekf.predict(u, motion_model, dt)
            logger.debug("state %.3f %s", odometry.time, a2s(_ekf.x.flatten()))
            logger.debug("cov %.3f %s", odometry.time, a2s(_ekf.P.flatten()))
            t = time.time()  # odometry.time
            if t >= RealsenseStatus.override + 1.0:
                logger.debug("track ekf %.3f %s", odometry.time, a2s(_ekf.x.flatten()))
                s = State.from_array(_ekf.x)
                await topics.robot_tracking.publish(s)
        t_last = odometry.time


async def driver():
    logger.info("Starting RealSense driver3...")
    start_task(_odometry())
    await _tracker()
