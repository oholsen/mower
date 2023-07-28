import cmath
from math import cos, sin

from dataclasses import dataclass

from .state import State


@dataclass
class Turtle:

    # position could be complex with properties for accessing x and y
    x: float = 0
    y: float = 0
    theta: float = 0

    def position(self) -> complex:
        return complex(self.x, self.y)

    def update_speed_omega_line(self, speed: float, omega: float, dt: float):
        # Deprecated, see update_speed_omega(). Remains just to see the difference.
        # Straight line from start position and heading, then turn by omega * dt
        dpos = cmath.rect(speed * dt, self.theta)
        self.x += dpos.real
        self.y += dpos.imag
        self.theta += omega * dt

    def update_speed_omega(self, speed: float, omega: float, dt: float):
        # speed, omega at start or end of dt?
        distance = speed * dt
        angle = omega * dt
        self.update_distance_angle(distance, angle)

    def update_distances(self, dist_left: float, dist_right: float, wheel_base: float):
        distance = (dist_left + dist_right) / 2  # along arc
        angle = (dist_right - dist_left) / wheel_base
        self.update_distance_angle(distance, angle)

    def update_distance_angle(self, distance: float, dtheta: float):
        if dtheta > 0.01:
            r = distance / dtheta
            dlon = r * sin(dtheta)
            dlat = r * (1 - cos(dtheta))
        else:
            # for small dtheta get inf * 0 above, expand Taylor series, x is dtheta:
            # sin(x) is x - x**3/3!
            # cos(x) is 1 - x**2/2!
            # to second order:
            dlon = distance * (1 - dtheta * dtheta / 6)
            dlat = distance * dtheta / 2
        p = complex(self.x, self.y) + complex(dlon, dlat) * cmath.rect(1, self.theta)
        self.x = p.real
        self.y = p.imag
        self.theta += dtheta

    def fix(self, x: float, y: float, theta: float, alpha: float):
        # relax towards the fix coordinates with the factor alpha
        self.x += alpha * (x - self.x)
        self.y += alpha * (y - self.y)
        self.theta += alpha * (theta - self.theta)

    def state(self) -> State:
        return State(self.x, self.y, self.theta)
