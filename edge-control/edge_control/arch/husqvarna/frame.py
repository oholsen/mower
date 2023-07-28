import time

import aioserial

_FRAME_START = 0xDE
_FRAME_END = 0xAD


def frame(message: bytes) -> bytes:
    return bytes([_FRAME_START, len(message)]) + message + bytes([_FRAME_END])


async def read_frames(device: aioserial.AioSerial):
    while True:
        ch = await device.read_async()
        while ord(ch) != _FRAME_START:
            ch = await device.read_async()

        # time stamp beginning of frame
        t = time.time()
        ch = await device.read_async()
        frame_length = ord(ch)
        data = await device.read_async(frame_length)
        ch = await device.read_async()
        if ord(ch) == _FRAME_END:
            yield t, data
