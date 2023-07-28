import cmath
import logging
import math
from math import pi
from typing import Optional, Tuple

from ..config import mission_config, robot_config
from ..models.messages import MoveCommand, ObstacleDetection, StopCommand, ToRobot
from ..models.state import State
from ..robot import RobotState
from ..util.math import norm_angle
from . import Control, StateIndependentControl

logger = logging.getLogger(__name__)


class GetStateControl(StateIndependentControl):
    """Terminates immediately just to get current state"""

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        raise Exception("GetStateControl update() should not be called")

    def end(self, t: float, state: State) -> bool:
        return True


class HLineControl(Control):
    def __init__(self, y: float, right: bool, end_x: float, speed=0.2, omega=0.2):
        self.y = y
        self.right = right
        self.end_x = end_x
        self.speed = speed
        self.omega = omega

    def __str__(self):
        return f"HLineControl({self.right},{self.end_x})"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        d = self.y - state.y
        # angle is 90 for large d
        theta = (pi / 2) * (1 - math.exp(-((d / 0.25) ** 2)))
        if d < 0:
            theta = -theta
        if not self.right:
            theta = pi - theta
        angle = norm_angle(theta - state.theta)
        speed = self.speed * math.exp(-abs(4 * angle) ** 2)
        if math.sin(theta) > 0:
            speed = min(speed, abs(d) / math.sin(theta) + 0.02)
        omega = math.copysign(min(abs(angle) + 0.02, self.omega), angle)
        return MoveCommand(t, speed, omega)

    def end(self, t: float, state: State) -> bool:
        return self.right == (state.x >= self.end_x)


class LineControl(Control):
    def __init__(self, p0, p1, speed, omega, proximity=0.0):
        self.p0 = complex(*p0)
        self.p1 = complex(*p1)
        dp = self.p1 - self.p0
        self.p2 = self.p1 + dp  # p2 is p0 mirrored around p1
        self.theta = cmath.phase(dp)
        self.dp = dp / abs(dp)
        # TODO: use mission_config
        self.speed = speed
        self.omega = omega
        self.proximity = proximity

    def __str__(self):
        return f"LineControl(({self.p0.real:.2f},{self.p0.imag:.2f}), ({self.p1.real:.2f},{self.p1.imag:.2f}, {self.proximity:.2f}))"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        p = complex(state.x, state.y)

        # Cross product gives distance to line and which side of line.
        dp = p - self.p0
        d = dp.real * self.dp.imag - dp.imag * self.dp.real

        angle = mission_config.control.angle_to_line(d)
        # Correct for which side of the line we are on.
        if d < 0:
            angle = -angle
        # change in robot heading (with respect to current theta)
        angle = norm_angle(self.theta + angle - state.theta)
        speed = mission_config.control.speed_from_angle(angle)
        speed = min(speed, mission_config.control.speed_from_distance(abs(p - self.p1)))
        omega = mission_config.control.omega_from_angle(angle)
        return MoveCommand(t, speed, omega)

    def end(self, t: float, state: State) -> bool:
        p = complex(state.x, state.y)
        return abs(p - self.p0) + self.proximity >= abs(p - self.p2) - self.proximity


class PointControl(Control):
    def __init__(self, x: float, y: float, speed: float, omega: float, end):
        self.x = x
        self.y = y
        self.speed = speed
        self.omega = omega
        self._end = end

    def __str__(self):
        return f"PointControl({self.x},{self.y})"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        dx = self.x - state.x
        dy = self.y - state.y
        d, angle = cmath.polar(complex(dx, dy))
        angle = norm_angle(angle - state.theta)
        speed = mission_config.control.speed_from_angle(angle)
        speed = min(speed, mission_config.control.speed_from_distance(d))
        omega = mission_config.control.omega_from_angle(angle)
        return MoveCommand(t, speed, omega)

    def end(self, t: float, state: State) -> bool:
        return self._end(t, state)


class PointControl2(Control):
    # End when crossing end line: normal through end point wrt line from starting point.

    def __init__(self, p0: Tuple[float, float], p1: Tuple[float, float], proximity=0.0):
        self.p0 = complex(*p0)  # start point
        self.p1 = complex(*p1)  # end point
        self.p2 = self.p1 + (self.p1 - self.p0)  # mirror image of p0 around normal at p1
        self.proximity = proximity
        self._end = False

    def __str__(self):
        return f"PointControl2({self.p1.real},{self.p1.imag})"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        assert not self._end
        p = complex(state.x, state.y)
        if abs(p - self.p0) + 2 * self.proximity >= abs(p - self.p2):
            self._end = True
            return None

        d, angle = cmath.polar(self.p1 - p)
        angle = norm_angle(angle - state.theta)
        speed = mission_config.control.speed_from_angle(angle)
        speed = min(speed, mission_config.control.speed_from_distance(d))
        omega = mission_config.control.omega_from_angle(angle)
        return MoveCommand(t, speed, omega)

    def end(self, t: float, state: State) -> bool:
        return self._end


def avoid_obstacle(move_command: MoveCommand, o: ObstacleDetection) -> MoveCommand:

    t = move_command.timeout
    speed = move_command.speed
    omega = move_command.omega

    # Obstacle avoidance overrides point control.
    # Cannot use same logic on bumper sensors - need some form of hysteresis to avoid oscillations (e.g. a map).
    # How to handle two sources? Merge depth camera nd bumper sensors.
    # 1. New heading fix will also offset integrated heading offset.
    # 2. Toggling between obstacle avoidance and point control. Use a continuous distance in outer regions.
    # Stop when angle to point is huge - and distance is short, avoid running in circles around destination.

    if o is None:
        # missing obstacle sensor reading
        return StopCommand(t)

    if o.center_left or o.center_right:

        if o.left and o.right:
            # No free flank, stuck in a corner. In general, need a obstacle map to escape trap. Just turn
            # and hopefully escape.
            speed = 0
            omega = mission_config.control.omega
            return MoveCommand(t, speed, omega)

        # one flank is free
        assert not o.left or not o.right

        # stop forwards motion, allow reverse speed
        speed = min(speed, 0.0)

        # Turn to free flank, preferred order:
        # 1. direction with free center + flank - ignoring desired turn since one center is blocked
        # 2. turn in desired direction and free flank - in case both flanks are free
        # 3. turn to only free flank
        left = False
        right = False
        if not o.left and not o.center_left:
            # turn left to free center + flank
            left = True
        elif not o.right and not o.center_right:
            # turn right to free center + flank
            right = True
        elif omega >= 0 and not o.left:
            # preferred left turn
            left = True
        elif omega < 0 and not o.right:
            # preferred right turn
            right = True
        elif not o.left:
            # turn left to free flank
            left = True
        else:
            assert not o.right
            # turn right to free flank
            right = True
        assert right != left
        omega = mission_config.control.omega
        if right:
            omega = -omega
    else:
        # Don't turning into busy flank.
        # Can go full speed, no need to go slowly to adjust theta.
        if omega > 0 and o.left or omega < 0 and o.right:
            omega = 0
            speed = mission_config.control.speed

    return MoveCommand(t, speed, omega)


def stop_obstacle(move_command: MoveCommand, o: ObstacleDetection) -> MoveCommand:

    t = move_command.timeout

    if o is None:
        # missing obstacle sensor reading
        return StopCommand(t)

    if o.center_left or o.center_right:
        return StopCommand(t)

    return move_command


class AvoidObstacleControl(Control):
    def __init__(self, control: Control):
        self.control = control

    def __str__(self):
        return f"AvoidObstacleControl({self.control})"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        cmd = self.control.update(t, state)
        if isinstance(cmd, MoveCommand):
            o = RobotState.obstacle_depth
            if o is not None:
                cmd2 = avoid_obstacle(cmd, o)
                logger.debug("Avoid obstacle %s %s %s", o, cmd, cmd2)
                return cmd2
        return cmd

    def end(self, t: float, state: State) -> bool:
        return self.control.end(t, state)


class StopObstacleControl(Control):
    def __init__(self, control: Control):
        self.control = control

    def __str__(self):
        return f"StopObstacleControl({self.control})"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        cmd = self.control.update(t, state)
        if isinstance(cmd, MoveCommand):
            o = RobotState.obstacle_depth
            if o is not None:
                cmd2 = stop_obstacle(cmd, o)
                logger.debug("Stop obstacle %s %s %s", o, cmd, cmd2)
                return cmd2
        return cmd

    def end(self, t: float, state: State) -> bool:
        return self.control.end(t, state)


def distance(x, y, r):
    def d(t: float, state: State):
        dx = x - state.x
        dy = y - state.y
        return math.hypot(dx, dy) < r

    return d


def nearest(x0, y0, x1, y1):
    def end(t: float, state: State):
        d0 = math.hypot(x0 - state.x, y0 - state.y)
        d1 = math.hypot(x1 - state.x, y1 - state.y)
        return d1 <= d0

    return end


class TimeControl(StateIndependentControl):
    def __init__(self, speed: float, omega: float, time: float):
        self.speed = speed
        self.omega = omega
        self.time = time  # seconds
        self.t0 = 0  # type: float

    def __str__(self):
        return f"TimeControl({self.time})"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        if self.t0 <= 0:
            self.t0 = t
        return MoveCommand(t, self.speed, self.omega)

    def end(self, t: float, state: State) -> bool:
        return self.t0 > 0 and t >= self.t0 + self.time


class TimeControl2(StateIndependentControl):
    def __init__(self, speed: float, omega: float, end_time: float):
        self.speed = speed
        self.omega = omega
        self.end_time = end_time

    def __str__(self):
        return f"TimeControl2"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        return MoveCommand(t, self.speed, self.omega)

    def end(self, t: float, state: State) -> bool:
        return t >= self.end_time


def start_arc(radius: float, speed: float, omega: float, direction: bool):
    # speed and omega may be too high for outer motor in tight turn
    # if outer motor speed exceeds max, then have to reduce speed (and omega)

    # outer motor speed is speed + omega * wheelbase / 2
    # max_speed_of_motor = speed + speed * wheelbase / radius / 2
    # speed = min(speed, RobotModel.MAX_SPEED / (1 + 0.5 * RobotModel.WHEEL_BASE / radius))
    _omega = speed / radius
    if _omega < omega:
        omega = _omega
    else:
        speed = omega * radius

    max_speed = speed + omega * robot_config.wheel_base / 2
    assert max_speed < robot_config.max_speed
    if not direction:
        omega = -omega
    return speed, omega


class ArcControl(Control):
    def __init__(self, speed, omega, end_theta):
        self.end_theta = end_theta
        self.angle_last = None
        self.speed = speed
        self.omega = omega

    def __str__(self):
        return f"ArcControl({self.end_theta})"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        # TODO: slow down (both!) near end state such that it does not overshoot - but should get to end
        # zero crossing on the next update
        return MoveCommand(t, self.speed, self.omega)

    def end(self, t: float, state: State) -> bool:
        # stop at first zero crossing, but not a random jump +-pi
        angle = norm_angle(state.theta - self.end_theta)
        if self.angle_last is None:
            self.angle_last = angle
            return False

        if abs(angle) < 0.9:
            if self.angle_last < 0 and angle >= 0:
                return True
            if self.angle_last > 0 and angle <= 0:
                return True
        self.angle_last = angle
        return False


class HeadingControl(Control):
    def __init__(self, end_theta, error=0.1):
        self.end_theta = end_theta
        self.error = error
        self.last_theta = None

    def __str__(self):
        return f"HeadingControl({self.end_theta})"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        angle = norm_angle(self.end_theta - state.theta)
        omega = mission_config.control.omega_from_angle(angle)
        return MoveCommand(t, 0, omega)

    def end(self, t: float, state: State) -> bool:
        angle = norm_angle(state.theta - self.end_theta)
        if abs(angle) <= self.error:
            return True

        # stop at first zero crossing, but not a random jump +-pi
        if self.last_theta is not None and abs(angle) < 0.9:
            if self.last_theta < 0 and angle >= 0:
                return True
            if self.last_theta > 0 and angle <= 0:
                return True

        self.last_theta = angle
        return False


class SpeedDistanceControl(Control):
    def __init__(self, speed: float, distance: float):
        self.speed = speed
        self.distance = distance
        self.s0 = None  # type: Optional[State]

    def __str__(self):
        return f"SpeedDistanceControl({self.speed},{self.distance})"

    def update(self, t: float, state: State) -> Optional[ToRobot]:
        if self.s0 is None:
            self.s0 = state
        return MoveCommand(t, self.speed, 0)

    def end(self, t: float, state: State) -> bool:
        return self.s0 is not None and self.s0.distance(state) >= self.distance
