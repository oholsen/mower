from __future__ import annotations

import asyncio
import cmath
import logging
import random
from time import time
from typing import Optional

from edge_control import topics
from edge_control.config import gps_config, simulation_config, site_config
from edge_control.models.messages import DockCommand, MoveCommand, Odometry, SitePosition, Time, ToRobot
from edge_control.models.state import State
from edge_control.models.turtle import Turtle
from edge_control.tasks import start_task

from .status import SimulationStatus

logger = logging.getLogger(__name__)


class SimulatedRobot:
    def __init__(self, x: float, y: float, theta: float):
        self.turtle = Turtle(x, y, theta)
        self.speed = 0.0
        self.omega = 0.0  # not part of tracking state
        self.cut_power = 0.0
        self.t0 = time()
        self.move_timeout: Optional[float] = None

    @staticmethod
    def from_dock() -> SimulatedRobot:
        x = site_config.dock.position.x
        y = site_config.dock.position.y
        theta = site_config.dock.heading
        return SimulatedRobot(x, y, theta)

    def time(self) -> float:
        return time() - self.t0

    def set_speed_omega(self, speed: float, omega: float):
        self.omega = omega
        self.speed = speed

    def update(self, dt: float):
        self.turtle.update_speed_omega(self.speed, self.omega, dt)

    def observe(self) -> complex:
        """returns observed state"""
        # observation_offset = 0
        # observation_offset = complex(0.11, 0)  # camera offset in roomba3: ahead of wheel base center
        # observation_offset = complex(-0.20, 0)  # GPS offset on jackal: behind center of robot
        if gps_config:
            observation_offset = complex(gps_config.offset.x, gps_config.offset.y)
        else:
            observation_offset = 0
        sigma = simulation_config.position_sigma if simulation_config else 0
        noise = complex(random.gauss(0, sigma), random.gauss(0, sigma))
        return complex(self.turtle.x, self.turtle.y) + observation_offset * cmath.rect(1, self.turtle.theta) + noise

    def state(self) -> State:
        observation = self.observe()
        return State(observation.real, observation.imag, self.turtle.theta)

    def site(self) -> SitePosition:
        observation = self.observe()
        return SitePosition(observation.real, observation.imag, hdop=0.1)

    async def heartbeat(self):
        logger.debug("Starting heartbeat task")
        i = 0
        while True:
            await asyncio.sleep(1)
            t = self.time()
            SimulationStatus.time.set(t)
            SimulationStatus.ok.set(False if i % 20 == 0 else True)
            SimulationStatus.cut_power = self.cut_power
            logger.debug("Heartbeat from simulator: %f", t)
            await topics.robot_state.publish(Time(t, t))
            i += 1

    async def run_real_time(self, time_step=0.5):
        # Run the simulation in real-time
        logger.debug("Starting real-time simulator")
        start_task(self.heartbeat())
        start_task(self.commands())
        t_last = self.time()
        while True:

            await asyncio.sleep(time_step)

            t = self.time()
            if self.move_timeout is not None and t >= self.move_timeout:
                logger.warning("Move timeout")
                self.move_timeout = None
                self.set_speed_omega(0, 0)

            self.update(t - t_last)
            t_last = t
            logger.debug("Update %.3f %s", t, self.turtle)
            SimulationStatus.turtle.set(self.turtle)

            if 0:
                # Bypass EKF tracker - use exact state:
                await topics.robot_tracking.publish(self.state())
            else:
                # Send to EKF tracker to find heading:
                await topics.site_position.publish(self.site())

    def do_command(self, command: ToRobot) -> Optional[Odometry]:
        # used for real-time and "fast-time"
        logger.debug("Command %s", command)

        if isinstance(command, MoveCommand):
            t = self.time()
            if command.timeout is not None and command.timeout > 0:
                dt = command.timeout - t
                if dt < 0 or dt > 2:
                    logger.warning("Invalid time for move command: %g %s", dt, command)
                    return None
                self.move_timeout = command.timeout
            else:
                self.move_timeout = None
            self.set_speed_omega(command.speed, command.omega)
            return Odometry(time(), command.speed, command.omega)

        if isinstance(command, DockCommand):
            logger.info("Docking")
            self.set_speed_omega(0, 0)
            self.move_timeout = None
            self.turtle.x = 0
            self.turtle.y = 0
            self.turtle.theta = 0
            return Odometry(time(), 0, 0)

        return None

    async def commands(self):
        command: ToRobot
        logger.debug("Starting commands task")
        async for command in topics.robot_command.stream():
            logger.debug("Received command %r", command)
            odometry = self.do_command(command)
            if odometry:
                # could publish from current state (after every command)
                await topics.odometry.publish(odometry)
