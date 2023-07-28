import asyncio
import json
import logging
import time
from typing import Any, Set

from dataclasses import asdict
from websockets import WebSocketServerProtocol  # type: ignore
from websockets import serve as serve_ws  # type: ignore

from edge_control import storage, topics
from edge_control.util.pubsub import Topic

from .config import site_config
from .robot import RobotState

"""
A WebSocket end-point for 
  * control of the robot (TBD)
  * state stream from the robot
"""

logger = logging.getLogger(__name__)
connections = set()  # type: Set[WebSocketServerProtocol]


async def feed(topic: Topic):
    async for msg in topic.stream():
        try:
            logger.debug("Feed %s", msg)
            await post_as_json(topic.name, asdict(msg))
        except ValueError:
            logger.exception("feed")


async def status():
    while True:
        await asyncio.sleep(1)
        s = RobotState.as_dict(time.time())
        logger.debug("API robot status: %s", s)
        await post_as_json("status", s)


async def post_map():
    await post_as_json("map", {"exterior": site_config.exterior, "interiors": site_config.interiors})


async def post_as_json(topic: str, message: Any):
    from .util.json import dumps

    envelope = {"topic": topic, "message": message}
    await post(dumps(envelope))


async def post(message: str):
    """Post the message to all current connections"""
    if connections:
        # return exceptions to avoid one closed connection to block sending to others
        await asyncio.gather(*[c.send(message) for c in connections], return_exceptions=True)


async def connection(ws: WebSocketServerProtocol, path: str):
    from .models.messages import MissionAbort, MissionStart, MoveCommand, StopCommand

    logger.info("Connection from %r %r", ws, path)
    connections.add(ws)
    logger.info("Number of connections: %d", len(connections))
    try:
        await post_map()
        await post_as_json("robot_tracking", [asdict(message) for message in storage.get()])
        async for msg in ws:
            # parse envelope and forward to topic...
            logger.debug("Received %r" % msg)
            envelope = json.loads(msg)
            topic = envelope.get("topic")
            message = envelope.get("message")
            if topic == "move/stop":
                await topics.robot_command.publish(StopCommand())
            elif topic == "move" and message:
                timeout = message.get("timeout")
                speed = message.get("speed")
                omega = message.get("omega")
                logger.debug("Publish command %r", MoveCommand(timeout, speed, omega))
                await topics.robot_command.publish(MoveCommand(timeout, speed, omega))
            elif topic == "mission/start" and message:
                await topics.mission_command.publish(MissionStart(message))
            elif topic == "mission/abort":
                await topics.mission_command.publish(MissionAbort())
            else:
                logger.warning("Ignoring %s", msg)
    finally:
        connections.remove(ws)
        logger.info("Connection closed")


async def serve(host: str, port: int):
    # returns when all set up to serve
    logger.debug("Starting web socket api on port %s:%d...", host, port)
    asyncio.create_task(feed(topics.robot_tracking))
    asyncio.create_task(status())
    await serve_ws(connection, host, port)


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="", help="Bind to host address")
    parser.add_argument("-p", "--port", type=int, default=8000, help="Bind to port")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    args = parser.parse_args()
    logging.basicConfig(level=args.verbose and logging.DEBUG or logging.INFO)
    asyncio.run(serve(args.host, args.port), debug=True)
    asyncio.get_event_loop().run_forever()


if __name__ == "__main__":
    main()
