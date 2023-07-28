import cmath
import logging
import time
from math import atan2, pi, sqrt
from typing import List, Tuple

import numpy
from numpy import linalg
from numpy.typing import ArrayLike

from edge_control.config import TagPosition, robot_config, tag_config
from edge_control.util.filters import LP

from . import status
from .T265 import Frame, Pose, Tag, to_site_position

logger = logging.getLogger(__name__)

_tag_index = dict((tag.id, tag) for tag in tag_config.tags) if tag_config else {}
if tag_config:
    assert len(_tag_index) == len(tag_config.tags), "Duplicate tag ids in tag config"


def norm_angle(a: float) -> float:
    while a > pi:
        a -= 2 * pi
    while a <= -pi:
        a += 2 * pi
    return a


"""
Track error in pose (heading) in first go based on random heading in docking station and that T265 initializes
with 0 on startup, relative to the startup pose.

Trust position from T265 for now, although relative to startup. Assume the startup position is pretty much fixed in the dock.

Current yaw angle is atan2(x, z) in camera coordinates (assuming the camera is level).
Real yaw angle to tag atan2(dX, dY) where dX, dY are defined in site coordinates and robot is at origin.
"""


def heading_camera(tag: Tag) -> float:
    x, y, z = tag.camera.t
    # camera coordinates, assume camera is level
    return atan2(-x, z)


def vector3(p: TagPosition):
    return numpy.array([p.x, p.y, p.z])


def tag_position(camera_position: ArrayLike, tag: Tag, pose: Pose) -> ArrayLike:
    q = pose.quaternion()
    camera = numpy.array(tag.camera.t)
    camera_rotate = q.rotate(camera)
    logger.debug(
        "Tag %d vectors %s %s %s",
        tag.tagId,
        camera_position,
        camera_rotate,
        tag.camera.t,
    )
    return camera_position + numpy.array([camera_rotate[2], camera_rotate[0], camera_rotate[1]])


def process(tag_frame: Frame, pose_frame: Frame, yaw_lp: LP):
    assert pose_frame.pose is not None
    assert tag_frame.tags is not None
    assert robot_config.realsense  # TODO: pass tags, pose, and realsense config as parameters

    pose = pose_frame.pose
    # TODO: offset tags in site by camera offset on base - AND - move to office fixed origin (and rotate x,y)
    # NOTE: have to rotate the horizontal offset by the heading/rotation of the robot to get to camera position in site
    camera_offset = numpy.array([0.0, 0, 1.17])  # wrt center of wheel base on floor level
    camera_position = numpy.array(pose.site_position()) + camera_offset
    heading = pose.site_heading()  # Heading may have error for matching against world

    _max = (0, None, None)
    tags = []
    for tag_pose in tag_frame.tags:
        tag_id = tag_pose.tagId

        # log both camera and world coordinates and compare accuracy
        position = tag_position(camera_position, tag_pose, pose)
        logger.debug("Tag %d camera %s", tag_id, position)
        if tag_pose.world:
            position = numpy.array(to_site_position(tag_pose.world.t))
            position = position + camera_offset
            logger.debug("Tag %d world %s", tag_id, position)

        logger.debug("Tag %d site %s", tag_id, position)

        logger.debug(
            "Tag %d dist %.3f %s",
            tag_id,
            norm(position - camera_position),
            position - camera_position,
        )
        # Check angle wrt camera - maybe more inaccurate around the edges, need to correct with camera matrix!?

        e = None
        dyaw = None
        tag = _tag_index.get(tag_id)
        if tag is None:
            logger.debug("Tag %d ignored", tag_id)
        else:
            real_tag_position = vector3(tag.position)
            if tag_id in robot_config.realsense.reference_tags:
                # for reference tag only for now, need to fix setup for the other tags and make sure of no mishaps
                yaw1 = heading + heading_camera(tag_pose)  # heading in camera frame
                # yaw1 = heading_camera(tag_pose)  # heading in camera frame
                dpos = real_tag_position - camera_position  # site coordinates
                yaw2 = atan2(dpos[1], dpos[0])  # wrt x-axis in site coordinates
                dyaw = norm_angle(yaw1 - yaw2)  # error in yaw
                logger.debug(
                    "Tag %d yaw %.4f %.4f %.4f %.4f %s",
                    tag_id,
                    dyaw,
                    heading,
                    yaw1,
                    yaw2,
                    dpos,
                )
                yaw_lp(dyaw)
            error = position - real_tag_position
            e = norm(error)
            logger.debug("Tag %d error %.3f %s", tag_id, e, error)
            # if e > _max[0]: _max = (e, tag_id, dyaw)

        tags.append(status.TagStatus(tag_id, e, dyaw))

    if 0:
        e, tag_id, yaw = _max
        status.RealsenseStatus.tags.set(tags)
        status.RealsenseStatus.tag_error.set(status.TagStatus(tag_id, e, yaw) if tag_id is not None else None)


def distance(x1, x2):
    return norm(x2 - x1)


def norm(x):
    return linalg.norm(x[:2])


def position(x0, pairs, camera_offset):
    """
    Find the optimal x for the observed pairs
    :param x0: (x, y, heading)
    :param pairs: list of (tag, complex) where tag is an observation and complex is the world position of the tag.
    :param camera_offset: 2D/3D offset of camera
    :return: (x, y, heading) or None if no match.
    """
    from scipy.optimize import minimize

    if len(pairs) < 2:
        return

    # camera t is (right, down, ahead) in direction of camera
    # camera_offset is (ahead, left, up) wrt robot ahead

    def error(robot_pos: complex, camera_yaw: float, debug=False) -> float:
        # pos in world coords
        # yaw in radians in world coords
        _error = 0
        for tag, pos_real in pairs:
            tc = tag.camera.t
            # assumes camera points straight ahead in robot frame
            pos_est = robot_pos + complex(tc[2] + camera_offset.x, -tc[0] + camera_offset.y) * cmath.rect(1, camera_yaw)
            delta = pos_est - pos_real
            _error += abs(delta) ** 2
            if debug:
                logger.debug(
                    "Tag delta %d %.3f %.3f %.3f",
                    tag.tagId,
                    abs(delta),
                    delta.real,
                    delta.imag,
                )
        return _error

    def error_vector(x, debug=False) -> float:
        return error(complex(x[0], x[1]), x[2], debug)

    t_start = time.time()
    res = minimize(error_vector, x0)
    # res = minimize(error_vector, x0, method='nelder-mead', options={'xatol': 1e-8, 'disp': True})
    dt = time.time() - t_start
    logger.debug("Position elapsed: %.3f", dt)

    # Log deltas
    _e = error_vector(res.x, debug=True)
    e = sqrt(res.fun / len(pairs))
    logger.debug("Position minimize residual: %g", e)

    if not res.success:
        logger.warning("Position minimize failed: %s", res.message)
        return None

    if e > 0.2:  # quite large if tag db is exact...
        logger.warning("Position minimize residual too high: %g", e)
        return None

    # TODO: check that it is on a local minimum, i.e. sensitivity to changes in x or y
    # If not very sensitive to a change, it is a poor fit and discard.

    # TODO: check that distance to x0 is not too large - or do that externally,
    # subject to velocity constraints - and accumulated uncertainty per axis.

    # TODO: check bounds
    # TODO: check that no single tag error is large
    # TODO: lower weight on depth to tag as it has a (much) bigger error (a few cm compared to a few mm on the bearing tag)

    return res.x


def select(tags: List[Tag]) -> List[Tuple[Tag, complex]]:
    # returns the same format or None if no match
    pairs = []
    for tag in tags:
        # Filter out tags low in the camera, possible duplicates for Spot
        if tag.camera.t[1] > 0.5:  # axis points down from camera offset
            logger.warning("Ignoring tag %d too low: %s", tag.tagId, tag.camera.t)
            continue

        _tag = _tag_index.get(tag.tagId)
        if _tag:
            p = _tag.position
            # ignore low tags, as duplicates on thin paper from Spot AND
            # does not provide any more support if there is also one at camera height
            if p.z < 0.7:
                continue
            pairs.append((tag, complex(p.x, p.y)))
    return pairs
