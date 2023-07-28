"""
Bridge robot and motor control command topics.
"""

import logging
from math import pi

from edge_control.config import robot_config
from edge_control.models.messages import MoveCommand

from .messages import Heartbeat, MotorSpeed

logger = logging.getLogger(__name__)


_update_rate = 20.0  # Hz
_dist_per_rev = 0.20 * pi
_ticks_per_rev = 588
_ticks_per_dist = _ticks_per_rev / _dist_per_rev


def target_delta(v: float) -> int:
    return int(round(_ticks_per_dist * v / _update_rate))


def from_move(move: MoveCommand, cut_power: float) -> MotorSpeed:
    v_rot = move.omega * robot_config.wheel_base / 2
    v_left = move.speed - v_rot
    v_right = move.speed + v_rot
    end_millis = int(1000 * move.timeout) if move.timeout else None
    # end_millis = None
    # TODO: check that speeds are within bounds (max and min)!
    # logger.debug("from_move %g %g %g", v_rot, v_left, v_right)
    # reverse direction of left wheel
    cut_delta = 1 if cut_power > 0 else -1 if cut_power < 0 else 0
    return MotorSpeed([target_delta(v_right), target_delta(-v_left), cut_delta], end_millis)


def to_move(m: Heartbeat):
    # 2-bit low pass filter
    rate0 = m.count0 * _update_rate / 4
    rate1 = m.count1 * _update_rate / 4
    vr = rate0 / _ticks_per_dist
    vl = -rate1 / _ticks_per_dist
    # logger.debug("to_move speeds %.3f %.3f", vl, vr)
    speed = (vr + vl) / 2
    omega = (vr - vl) / robot_config.wheel_base
    return speed, omega
