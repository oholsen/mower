import asyncio
import logging

import numpy as np
import pyrealsense2 as rs

from edge_control.robot import RobotState
from edge_control.util.fstreamer import Writer
from edge_control.util.tasks import start_task

from .obstacle import ObstacleDetector, load_floor

logger = logging.getLogger(__name__)
width = 640
height = 480
frame_rate = 6


def start_pipeline():
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, frame_rate)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, frame_rate)
    pipeline = rs.pipeline()
    pipeline.start(config)
    return pipeline


def get_depth(frames):
    depth_frame = frames.get_depth_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    t = depth_frame.get_timestamp()
    return t, depth_image


def start():
    start_task(driver(), "Depth camera")


async def driver():
    # OR run entire depth process in new process and post to MQTT
    logger.info("Starting obstacle detection from depth camera...")

    floor_depth = load_floor("floor.npy")
    detector = ObstacleDetector(floor_depth)

    # writer = Writer(open("data/depth.bag", "wb"), 500)
    loop = asyncio.get_running_loop()
    pipeline = await loop.run_in_executor(None, start_pipeline)

    def process():
        frames = pipeline.wait_for_frames()
        t, depth_image = get_depth(frames)
        # writer.write(t, depth_image)
        return detector.process(t, depth_image)

    try:
        while True:
            detection = await loop.run_in_executor(None, process)
            logger.debug("detection %s", detection)
            RobotState.obstacle_depth = detection
            await asyncio.sleep(0.5)
    finally:
        pipeline = await loop.run_in_executor(None, pipeline.stop)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(driver(), debug=True)
