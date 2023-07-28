import asyncio
import logging

import aioserial

from edge_control.arch.jackal.lights import *
from edge_control.config import SerialConfig

logger = logging.getLogger(__name__)


async def main():
    import sys

    device = SerialConfig("/dev/arduino")
    arduino = aioserial.AioSerial(device.port, device.baud_rate)

    async def read():
        while True:
            line = await arduino.readline_async()
            logger.debug("Status: %r", line)

    async def write(cmd: str):
        logger.debug("To Arduino: %s", cmd)
        logger.debug("To Arduino: %r", f"{cmd}\n".encode("ascii"))
        await arduino.write_async(f"{cmd}\n".encode("ascii"))

    asyncio.create_task(read())
    # must wait a little for the write to be accepted

    await asyncio.sleep(3)
    print(sys.argv)
    for command in sys.argv[1:]:
        print("Command", command)
        await write(eval(f"{command}()"))

    await asyncio.sleep(3)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(main())
