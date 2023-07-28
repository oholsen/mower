import asyncio
import logging

import aioserial

"""
All in one command line utility for FLIR pan tilt PoC on RB1.

Servo control via Polulo Maestro Servo Controller
https://www.pololu.com/docs/0J40 
"""

logger = logging.getLogger(__name__)


class Servo:
    # pulse width in us

    MIN_PULSE_WIDTH = 500
    CENTER_PULSE_WIDTH = 1500
    MAX_PULSE_WIDTH = 2500

    def __init__(self, channel: int, pulse_width=CENTER_PULSE_WIDTH):
        self.channel = channel
        self.pulse_width = pulse_width

    async def write(self, port: aioserial.AioSerial):
        # Set target - subject to velocity and acceleration in the controller
        pulse_width = int(self.pulse_width * 4)
        await port.write_async(bytes([0x84, self.channel, pulse_width & 0x7F, pulse_width >> 7 & 0x7F]))

    async def read(self, port: aioserial.AioSerial) -> int:
        # Get current position
        await port.write(bytes([0x90, self.channel]))
        response = await port.read_async(2)
        return response[0] + 256 * response[1]

    def update(self, pulse_width: float):
        self.pulse_width = max(min(pulse_width, Servo.MAX_PULSE_WIDTH), Servo.MIN_PULSE_WIDTH)

    def add(self, delta_us):
        self.update(self.pulse_width + delta_us)

    def set(self, p: float):
        self.update(Servo.CENTER_PULSE_WIDTH + p * (Servo.MAX_PULSE_WIDTH - Servo.CENTER_PULSE_WIDTH))


tilt = Servo(0)
pan = Servo(1)


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
    # await asyncio.sleep(3)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(set())
