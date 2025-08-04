import asyncio
import json
import logging
import time
from typing import Any

import dacite
import gmqtt

from . import topics
from .config import robot_config
from .missioncontrol import mission_abort, mission_start
from .models.messages import DockCommand, LightsCommand, MoveCommand
from .models.ptz import PanTiltZoomPosition, PanTiltZoomSpeed
from .models.state import ModeState
from .robot import RobotState
from .util.inrobot import Envelope
from .util.json import dumps
from .util.mqtt import Client
from .util.time import now

logger = logging.getLogger(__name__)


def publish_to_inrobot(client: Client, topic: str, payload: Any, timestamp: int = None):
    return
    m = Envelope(topic, payload, timestamp or now()).to_json()
    logger.debug("To InRobot: %s", m)
    client.publish("api", m)


async def publish_to_topic(payload, clz, topic: topics.Topic):
    try:
        msg = json.loads(payload)
        logger.debug("To topic %s: %r", topic, msg)
        cmd = dacite.from_dict(clz, msg)
        await topic.publish(cmd)
    except Exception:
        logger.error("Failed to publish %r to %s %r", payload, topic, clz)


async def command_lights(client: Client, topic: str, payload):
    await publish_to_topic(payload, LightsCommand, topics.lights)


async def command_ptz_move(client: Client, topic: str, payload):
    await publish_to_topic(payload, PanTiltZoomSpeed, topics.ptz_command)


async def command_ptz_absolute(client: Client, topic: str, payload):
    await publish_to_topic(payload, PanTiltZoomPosition, topics.ptz_command)


async def command_mode(client: Client, topic: str, payload):
    # Roomba: Full, Safe, Passive, Dock, ...
    # Modes are really ignored - always in Full/Safe.
    # Should only ack recognized modes.
    msg = json.loads(payload)
    logger.info("Mode command: %r", msg)
    mode = msg.get("mode")
    if not mode:
        logger.warning("Invalid mode: %r", mode)
        return
    publish_to_inrobot(client, "mode/status", {"mode": mode})
    if mode == "Dock":
        await topics.robot_command.publish(DockCommand(time.time()))
        return
    logger.info("Ignoring mode command: %s", mode)


async def command_twist(client: Client, topic: str, payload):
    msg = json.loads(payload)
    logger.debug("Twist command: %r", msg)
    timestamp = msg.get("timestamp")
    timeout = 0.0
    if timestamp:
        dt = now() - timestamp
        if dt < 0 or dt > 1000:
            logger.warning("Command timestamp mismatch %d ms: %r", dt, msg)
            return
        timeout = 0.001 * timestamp + 1.0
    # speeds are 0-1, scale to appropriate speeds for robot
    # need to make sure that speed + omega is within motor speed range
    speed = msg["linear"]["x"] * robot_config.max_speed
    omega = -msg["angular"]["z"] * robot_config.max_omega
    await topics.robot_command.publish(MoveCommand(timeout, speed, omega))


async def command_mission_start(client: Client, topic: str, payload):
    msg = json.loads(payload)
    logger.info("Mission start: %r", msg)
    name = msg.get("name")
    await mission_start(name)


async def command_mission_abort(client: Client, topic: str, payload):
    msg = json.loads(payload)
    logger.info("Mission abort: %r", msg)
    await mission_abort()


async def command_estop(client: Client, topic: str, payload):
    msg = json.loads(payload)
    logger.info("estop command: %r", msg)
    if msg.get("signal") == "STOP":
        logger.info("Mission abort from estop: %r", msg)
        publish_to_inrobot(client, "estop/status", {"acquired": True, "active": True})
        await mission_abort()


async def command_mission_pause(client: Client, topic: str, payload):
    msg = json.loads(payload)
    # mission automatically paused on invalid state? or just waits while invalid?
    # paused for a reason - report mission state, progress
    logger.info("Mission pause: %r", msg)


async def command_mission_continue(client: Client, topic: str, payload):
    msg = json.loads(payload)
    logger.info("Mission continue: %r", msg)


_dispatch = {
    # From InRobot web app:
    "ptz/move": command_ptz_move,
    "ptz/absolute": command_ptz_absolute,
    "cmd/lights": command_lights,
    "robot_cmd/twist_cmd": command_twist,
    "robot_cmd/mode_cmd": command_mode,
    "robot_cmd/navigate_mission": command_mission_start,
    "bosdyn/EstopSignal": command_estop,
    "estop/cmd": command_estop,
    # Proposal:
    "mission/start": command_mission_start,
    "mission/abort": command_mission_abort,
    "mission/pause": command_mission_pause,
    "mission/continue": command_mission_continue,
}


async def cloud_mqtt_driver():
    _client = gmqtt.Client("robot-autonomy")
    client = Client(_client, "mqtt", _dispatch)
    await client.connect("localhost", 1883)

    async def _robot_status():
        # publish RobotState periodically (also a task doing the same on API).
        # Could publish it on a pubsub topic and have local tasks publishing to API and MQTT.
        while True:
            await asyncio.sleep(1)
            s = dumps(RobotState.as_dict(time.time()))
            logger.info("Robot status: %s", s)
            client.publish("autonomy/status", s)
            publish_to_inrobot(client, "motors/status", {"powered": True})
            publish_to_inrobot(client, "estop/status", {"acquired": True, "active": False})

    async def _to_mqtt(topic: topics.Topic, mqtt_topic: str, qos: int = 0):
        async for o in topic.stream():
            logger.debug("To MQTT %r: %r", mqtt_topic, o)
            client.publish(mqtt_topic, dumps(o), qos=qos)

    async def _to_api(topic: topics.Topic, topic_type: str):
        async for o in topic.stream():
            publish_to_inrobot(client, topic_type, o)

    await asyncio.gather(
        _robot_status(),
        _to_api(topics.edge_status, "EdgeStatus"),
        _to_api(topics.batteries_status, "BatteryStatus"),
        _to_mqtt(topics.gauge_captures, "gaugecapture", 1),
        _to_mqtt(topics.missions, "mission", 1),
    )


def mission_control(args):
    from .tasks import start
    from .util import tasks

    loop = asyncio.get_event_loop()
    loop.set_exception_handler(tasks.handle_exception)

    async def _run():
        await start()
        await cloud_mqtt_driver()

    asyncio.run(_run(), debug=args.verbose)
    # logger.info("Shutting down tasks")
    # asyncio.run(tasks.shutdown())


def parse_args():
    import argparse

    parser = argparse.ArgumentParser(
        prog="edge-control.mqtt",
        description="Robot control service",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    return parser.parse_args()


def main():
    import logging.config

    import yaml

    args = parse_args()
    try:
        with open("logging.yaml") as f:
            logging.config.dictConfig(yaml.full_load(f))
    except FileNotFoundError:
        logging.basicConfig(level=args.verbose and logging.DEBUG or logging.INFO)

    logging.info("Starting edge-control")
    mission_control(args)


if __name__ == "__main__":
    main()
