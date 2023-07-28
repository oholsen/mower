#!/usr/bin/env python3
import asyncio

from edge_control.realsense.driver1 import frames, states
from edge_control.realsense.T265 import to_site_euler


async def main1():
    print("Testing realsense...")
    async for frame in frames():
        if frame.pose is None:
            # tags
            print(frame)
        else:
            q = frame.pose.rotation()
            pitch, roll, yaw = to_site_euler(q)
            print(frame.frame, frame.pose.t, pitch, roll, yaw)


async def main():
    print("Testing realsense...")
    async for state in states():
        print(state)


if __name__ == "__main__":
    asyncio.run(main())
