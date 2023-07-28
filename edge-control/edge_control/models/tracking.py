"""
Based on Extended kalman filter (EKF) localization sample in PythonRobotics by Atsushi Sakai (@Atsushi_twi)
"""

import asyncio
import logging
import math
import time
from typing import Optional, Tuple

import numpy as np
from numpy.typing import ArrayLike

from .. import topics
from ..config import Vector2D
from ..util.math import norm_angle
from .messages import SitePosition
from .state import State

logger = logging.getLogger(__name__)

# Covariance for EKF simulation
Q = (
    np.diag(
        [
            0.02,  # variance of location on x-axis
            0.02,  # variance of location on y-axis
            np.deg2rad(10.0),  # variance of yaw angle
        ]
    )
    ** 2
)  # predict state covariance


def rotation(theta: float) -> ArrayLike:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def rotation_jac(theta: float) -> Tuple[ArrayLike, ArrayLike]:
    # returns rotation and Jacobian (derivative wrt theta)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]]), np.array([[-s, -c], [c, -s]])


def motion_model(x, u, dt: float):
    """
    x is [x, y, yaw]
    u is [speed, omega]

    motion model:
        x_{t+1} = x_t + v*dt*cos(yaw)
        y_{t+1} = y_t + v*dt*sin(yaw)
        yaw_{t+1} = yaw_t + omega*dt
        v_{t+1} = v{t}

    Jacobian of motion model:
        dx/dyaw = -v*dt*sin(yaw)
        dy/dyaw = v*dt*cos(yaw)
        d_/dx = 1 or 0
        d_/dy = 1 or 0
    """
    # u is (speed, omega)'
    yaw = x[2, 0] + u[1, 0] * dt / 2
    s = math.sin(yaw)
    c = math.cos(yaw)

    B = np.array([[dt * c, 0], [dt * s, 0], [0.0, dt]])

    x = x + B @ u
    x[2, 0] = norm_angle(x[2, 0])

    d = u[0, 0] * dt
    jac = np.array(
        [
            [1.0, 0.0, -d * s],
            [0.0, 1.0, d * c],
            [0.0, 0.0, 1.0],
        ]
    )

    return x, jac


def position_model(x, observation_offset):
    """
    Return observation z and Jacobian for the given state x.
    Observation model: mapping state x to observation z
        x is [x y yaw]
        z is [x y]  (GPS)

    The actual observation is offset wrt robot base as specified in observation_offset.

    Jacobian of z:
        Dx/Dx = Dy/Dy = 1
        Dx/Dyaw = x + (observation_offset rotated by yaw)_x
    """
    yaw = x[2, 0]
    position_offset = rotation(yaw) @ observation_offset
    z = x[:2] + position_offset
    jac = np.hstack((np.eye(2), position_offset))
    return z, jac


def tag_observation_model(x, tag_position: complex):
    """
    Observation model: mapping state x to observation z
    Return observation z and Jacobian for the given state x.

    x is [x y yaw]
    z is camera [x' y'] where x' is forward, y' is left wrt robot pose.

    z = (tag_position - x) * rot(-yaw)

    rot theta is
    x' = dx c - dy s
    y' = dx s + dy c

    d/dx, d/dy, d/dyaw
    dx' = [-c,  s, -dx s - dy c]
    dy' = [-s, -c,  dx c - dy s]

    but rotate by -theta, so negate d/dyaw
    """
    yaw = x[2, 0]
    dp = np.array([[tag_position.real - x[0, 0]], [tag_position.imag - x[1, 0]]])

    # FIXME: use camera observation offset as above

    # TODO: could optimize by reusing s,c below - or rather - compose j with r and jj
    s = math.sin(-yaw)
    c = math.cos(-yaw)

    r, j = rotation_jac(-yaw)
    z = r @ dp
    jj = j @ dp
    jac = np.array(
        [
            [-c, s, -jj[0, 0]],
            [-s, -c, -jj[1, 0]],
        ]
    )
    return z, jac


class ExtendedKalmanFilter:
    def __init__(self, x: Optional[np.ndarray] = None, P: Optional[np.ndarray] = None):
        self.x = x
        self.P = P

    def predict(self, u, prediction_model, dt: float):
        """Update state from motion model"""
        assert self.x is not None
        self.x, jF = prediction_model(self.x, u, dt)
        self.P = jF @ self.P @ jF.T + Q

    def correct(self, z, observation_model, R):
        """Update state from an observation with the predicted z and Jacobian from the current state"""
        assert self.x is not None
        z_pred, jH = observation_model(self.x)
        y = z - z_pred
        S = jH @ self.P @ jH.T + R
        K = self.P @ jH.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(len(self.x)) - K @ jH) @ self.P


class RobotTracker(ExtendedKalmanFilter):
    # observation z: x, y, hdop
    # control input u: speed, omega
    # observation covariance R: hdop from GPS (hdop is radius of circle or "square"?)

    def __init__(self, observation_offset: Vector2D, state: Optional[State] = None):
        self.observation_offset = np.array([[observation_offset.x], [observation_offset.y]])
        super().__init__(None, None)
        if state:
            self.init(state)

    def init(self, state: State):
        # TODO: ones!? for both position, velocity, and theta? will rapidly settle
        # state vectors [x y yaw]'
        self.x = state.to_array()
        self.P = np.eye(3)

    def get_state(self) -> State:
        return State.from_array(self.x)

    def observation_model(self, x):
        return position_model(x, self.observation_offset)

    def odometry(self, speed: float, omega: float, dt: float):
        if self.x is not None:
            u = np.array([[speed], [omega]])
            self.predict(u, motion_model, dt)
        else:
            logger.debug("Ignoring odometry on empty state")

    def position(self, site_position: SitePosition):
        if self.x is None:
            # use first observation as state if not initialized
            self.init(State(site_position.x, site_position.y, 0))
        z = np.array([[site_position.x], [site_position.y]])
        R = (site_position.hdop**2) * np.eye(2)
        self.correct(z, self.observation_model, R)

    def update(self, site_position: SitePosition, speed: float, omega: float, dt: float):
        self.odometry(speed, omega, dt)
        self.position(site_position)


async def run_tracker(tracker: RobotTracker):
    # Run tracker on topics - tracker is not initialized
    # inputs: odometry, site_position
    # output: robot_tracking
    logger.debug("Starting robot tracker...")

    # It allows robot to move without position feedback (GPS) when it is safe to do so,
    # updating the covariance (increasing when no/poor position feedback).
    async def prediction():
        time_odo = None
        async for odometry in topics.odometry.stream():
            # TODO: capture time at source, embed in topic
            _time = time.time()
            logger.debug("Odometry %s", odometry)
            if time_odo is not None:
                tracker.odometry(odometry.speed, odometry.omega, _time - time_odo)
                # TODO: could publish new position here too - with covariance
            time_odo = _time

    async def correction():
        o: SitePosition
        async for o in topics.site_position.stream():
            logger.debug("Position %s", o)
            tracker.position(o)
            s = tracker.get_state()
            # logger.debug("STATE %g %g %g %g", s.x, s.y, s.theta)
            logger.debug("State %s", s)
            await topics.robot_tracking.publish(s)

    asyncio.create_task(prediction())
    await correction()
