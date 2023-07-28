import numpy as np

from edge_control.realsense.obstacle import ObstacleDetector, distance_threshold, height, width


def test_clear():
    floor = 1000 * np.ones((height, width), dtype=np.int16)
    image = floor
    d = ObstacleDetector(floor)
    detection = d.process(12.5, image)
    assert detection.time == 12.5
    assert not detection.left
    assert not detection.center_left
    assert not detection.center_right
    assert not detection.right


def test_all_blocked():
    floor = 1000 * np.ones((height, width), dtype=np.int16)
    image = floor - (10 + distance_threshold)  # 10 mm above floor threshold
    d = ObstacleDetector(floor)
    detection = d.process(12.5, image)
    assert detection.time == 12.5
    assert detection.left
    assert detection.center_left
    assert detection.center_right
    assert detection.right
