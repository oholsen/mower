import asyncio
import logging
import math

import numpy

from .. import topics
from ..config import robot_config, site_config
from ..models.state import State
from ..models.turtle import Turtle
from ..util.math import a2s
from . import tags
from .driver import frames
from .status import RealsenseStatus, Rotation, Vector
from .T265 import Frame

logger = logging.getLogger(__name__)
_bin_dir = "/home/oholsen/bin"
turtle = Turtle(site_config.dock.position.x, site_config.dock.position.y, site_config.dock.heading)


def position(frame: Frame):
    assert frame.tags is not None
    logger.debug("tags: %s", frame)
    pairs = tags.select(frame.tags)
    # add camera offset here to position the camera in the world
    p0 = numpy.array([turtle.x, turtle.y, turtle.theta])
    assert robot_config.realsense
    p = tags.position(p0, pairs, robot_config.realsense.position)

    if p is None:
        return

    d = p - p0
    # dtheta = d[2]
    distance = tags.norm(d[:2])
    logger.debug("dfix %.3f %s", distance, d)
    # subtract camera offset here to get back into the robot base frame
    logger.debug("pfix %.3f %.3f %.3f", p[0], p[1], p[2])
    if distance > 20:  # TODO: current speed + margin? should be updated on odometry...
        logger.warning("Position fix distance too high: %g", distance)
        # FIXME: show status text in GUI!
        # TODO: detect continuous error situation and stop/reset
        # TODO: merge with confidence 3 - EKF!!!! tags (also single), odometry and T265
        return
    turtle.fix(p[0], p[1], p[2], 0.25)
    logger.debug("fix %.3f %.3f %.3f", turtle.x, turtle.y, turtle.theta)


async def visual_tracker():

    t_last: int = 0

    async for frame in frames():
        try:
            if frame.tags:
                # loop = asyncio.get_event_loop()
                # loop.run_in_executor(executor, position, frame)
                position(frame)
                # publish now or wait for next odometry... wait to get a steady position rate..??
                continue

            # Reduce pose rate.
            # Source is 30 frames per second - could select reduced frame rate in source program
            if not frame.pose or frame.timestamp < t_last + 200:
                continue

            logger.debug("frame: %s", frame)
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
            t_last = frame.timestamp

        except Exception:
            logger.exception("Frame: %s" % frame)


async def odometry_tracker():
    from edge_control.models.messages import Odometry

    global turtle
    logger.debug("Starting odometry tracker...")
    o: Odometry
    t_last: float = None
    async for o in topics.odometry.stream():
        # logger.debug("Odometry %s", o)
        if t_last is not None:
            dt = o.time - t_last
            logger.debug("Odometry %.4f %s", dt, o)
            turtle.update_speed_omega(o.speed, o.omega, dt)
            state = State(turtle.x, turtle.y, theta=turtle.theta, speed=o.speed)
            logger.debug("state %s", state)
            logger.debug("opose %.3f %.3f %.1f", turtle.x, turtle.y, math.degrees(turtle.theta))
            RealsenseStatus.state.set(state)
            await topics.robot_tracking.publish(state)
        t_last = o.time


async def driver():
    # Output topic: robot_tracking
    logger.info("Starting Realsense driver2...")
    asyncio.create_task(odometry_tracker())
    await visual_tracker()
