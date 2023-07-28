import asyncio
import logging
import sys

import aioserial

from edge_control.arch.roomba.servo import Servo

logger = logging.getLogger(__name__)
tilt = Servo(0)
pan = Servo(1)


async def loop():
    import sys

    port = sys.argv[1]
    servos = aioserial.AioSerial(port, 9600)

    while True:
        print("SET -0.2")
        tilt.set(-0.2)
        pan.set(-0.2)
        await pan.write(servos)
        await tilt.write(servos)
        await asyncio.sleep(3)
        print("SET 0.7")
        tilt.set(0.7)
        pan.set(0.7)
        await pan.write(servos)
        await tilt.write(servos)
        await asyncio.sleep(3)


async def set():
    import sys

    port = sys.argv[1]
    pan_angle = float(sys.argv[2])
    tilt_angle = float(sys.argv[3])

    servos = aioserial.AioSerial(port, 9600)
    pan.set(pan_angle / 60.0)
    tilt.set(tilt_angle / 60.0)
    await pan.write(servos)
    await tilt.write(servos)
    await asyncio.sleep(3)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    if len(sys.argv) == 4:
        asyncio.run(set())
    else:
        asyncio.run(loop())
