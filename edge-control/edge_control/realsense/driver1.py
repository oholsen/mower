import cmath
import logging
import math

from .. import topics
from ..models.state import State
from ..util.filters import LP
from . import tags
from .driver import frames
from .status import RealsenseStatus, Rotation, Vector

logger = logging.getLogger(__name__)
_bin_dir = "/home/oholsen/bin"


async def states():
    # Convert to site coordinates, push position every 500ms (as GPS) and calculate speed

    tags_id = None
    last_pose = None

    # One update per second for tags. Smooth quite hard, drift changes very slowly.
    # Would be nice to filter out errors in early values, or just stay in dock a while to settle in.
    heading_filter = LP(0.05)

    # 30 frames per second - could select reduced frame rate in source program
    async for frame in frames():
        try:
            if frame.pose is None:
                logger.debug("tags: %s", frame)
                assert frame.frame % 30 == 0, f"Tags frame no {frame.frame}"
                # Tag coordinates are not mapped to site coordinates (with dock offset).
                # Moved to offline analysis (tests/tags.py).
                # Need tag processing for heading offset estimate while in dock

                tags_id = frame.frame
                if last_pose:
                    if last_pose.frame == frame.frame:
                        tags.process(frame, last_pose, heading_filter)
                        RealsenseStatus.stream_error.set(None)
                    else:
                        # Stuck with old Pose, still getting tags.
                        # FIXME: Detect stuck realsense!
                        RealsenseStatus.stream_error.set("tag/pose frame number mismatch")
                else:
                    # Init only
                    RealsenseStatus.stream_error.set("No pose for tag frame")
                continue

            # wait for first tags and then 5 per second
            if tags_id is None or (frame.frame - tags_id) % 6 != 0:
                continue

            logger.debug("frame: %s", frame)

            if last_pose is None:
                # record first frame
                last_pose = frame
                continue

            pitch, roll, heading = frame.pose.site_rotation()
            x, y, z = frame.pose.site_position()

            # Correct on heading error from tracking stationary april tags at known positions
            heading_error = heading_filter.get()
            RealsenseStatus.heading_correction.set(heading_error)
            if heading_error is not None:
                heading -= heading_error
                zz = complex(x, y) * cmath.rect(1, -heading_error)
                # correct for camera offset wrt wheel base centre
                # TODO: fix this properly - not here, also for tags
                # zz -= cmath.rect(0.11, heading)
                x, y = zz.real, zz.imag

            logger.debug(
                "pose %d %.3f %.3f %.3f %.1f %.1f %.1f",
                frame.pose.confidence,
                x,
                y,
                z,
                math.degrees(heading),
                math.degrees(pitch),
                math.degrees(roll),
            )

            # Speed jitters - should take from odometry, visual tracking can jump abruptly
            # Not used anyway in control?
            state = State(x, y, heading)

            last_pose = frame
            RealsenseStatus.state.set(state)
            RealsenseStatus.confidence.set(frame.pose.confidence)
            RealsenseStatus.position.set(Vector(x, y, z))
            RealsenseStatus.rotation.set(Rotation(pitch, roll, heading))
            yield state

        except Exception:
            logger.exception("Frame: %s" % frame)


async def driver():
    # Output topic: robot_tracking
    logger.info("Starting Realsense driver1...")
    async for state in states():
        logger.debug("state %s", state)
        await topics.robot_tracking.publish(state)
