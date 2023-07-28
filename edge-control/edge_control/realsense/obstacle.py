import logging

import cv2
import numpy as np

from ..models.messages import ObstacleDetection

logger = logging.getLogger(__name__)
width = 640
height = 480
frame_rate = 6

# TODO: configuration!!!
y2 = 300
y1 = height - 1
x0 = width // 2 + 7
dx2 = [0, 75, 150]  # top
dx1 = [0, 90, 150]  # bottom
distance_threshold = 25  # mm
obstacle_threshold = 0.01  # ratio of number of obstacle pixels to total number of pixels in region
_offsets = [-2, -1, 1, 2]


def region(offset: int):
    assert offset != 0
    i = abs(offset)
    assert i <= 2
    dxs = [dx1[i], dx2[i], dx2[i - 1], dx1[i - 1]]
    points = zip([x0 + dx if offset > 0 else x0 - dx for dx in dxs], [y1, y2, y2, y1])
    return np.array([[int(x), int(y)] for x, y in points])


def mask(points):
    m = np.zeros((height, width), dtype=np.uint8)
    cv2.fillConvexPoly(m, points, 1)
    # Mask out robot base in bottom center, creates noisy depth pixels
    # Before moving camera during wheel repairs until 2021-03-22
    # cv2.ellipse(m, (x0, y1 + 5), (70, 50), 0, 0, 360, 0, -1)
    # After moving camera and recalibration 2021-03-23
    cv2.ellipse(m, (x0, y1 + 5), (84, 84), 0, 0, 360, 0, -1)
    return m


def lines(image, points):
    points = points.reshape((-1, 1, 2))
    cv2.polylines(image, [points], True, (0, 0, 200), 1)


_points = [region(offset) for offset in _offsets]
_masks = [mask(pts) > 0 for pts in _points]
_areas = [mask.sum() for mask in _masks]


def load_floor(file_name: str):
    logger.info("Loading floor depth from %r", file_name)
    # load and reshape, if required, floor level
    with open(file_name, "rb") as f:
        floor_depth = np.load(f)
        floor_depth = np.ma.masked_equal(floor_depth, 0).astype(np.int16)
        # revert decimation from saved image
        h, w = floor_depth.shape
        if 2 * h == height:
            # in case stored images is decimated
            floor_depth = np.repeat(floor_depth, 2, axis=0)
            floor_depth = np.repeat(floor_depth, 2, axis=1)
        return floor_depth


class ObstacleDetector:
    def __init__(self, floor_depth):
        self.floor_depth = floor_depth

    def process(self, t: float, depth) -> ObstacleDetection:
        assert self.floor_depth.shape == depth.shape
        # Pixels without stereo depth information, e.g. where one camera is occluded,
        # have depth = 0. Consider these as obstacles, e.g. when peeking around a corner.
        # Should have very few false positives in the regions of interest immediately in front of the robot
        # (for camera mounted high and pointing down).
        # depth = np.ma.masked_equal(depth, 0)
        # also handles "holes", avoid falling down stairs

        obstacles = abs(depth - self.floor_depth) >= distance_threshold

        def regions():
            for mask, area in zip(_masks, _areas):
                count = obstacles[mask].sum()
                blocked = bool(count > obstacle_threshold * area)  # convert from numpy.bool_
                logger.debug("REGION %s %s %s", count, area, blocked)
                yield blocked

        return ObstacleDetection(t, *list(regions()))
