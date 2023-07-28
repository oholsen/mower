import cmath
import random

import numpy as np

from edge_control.models.tracking import ExtendedKalmanFilter, motion_model, tag_observation_model
from edge_control.models.turtle import Turtle
from edge_control.util.math import norm_angle
from tests.plotting import Plot, State, States


def main():

    speed = 1
    omega = 0
    dt = 0.05

    # True motion
    turtle = Turtle(0, 1, 0)

    tags = [
        ("1b", 1, 2, 90),
        ("1a", 1, 0, -90),
        ("3b", 3, 2, 90),
        ("5a", 5, 0, -90),
        ("6b", 6, 2.5, 90),
    ]

    x0 = np.array([[0], [1], [0]])
    r0 = np.diag([0.05, 0.05, 0.1])  # in docking station
    ekf = ExtendedKalmanFilter(x0, r0)
    true_path = States(".b")
    tracked_path = States(".r")
    plot = Plot([true_path, tracked_path])
    t = 0
    while t < 7:

        print("Turtle", turtle.x, turtle.y)
        u = np.array([[random.normalvariate(speed, 0.05)], [random.normalvariate(omega, 0.02)]])
        ekf.predict(u, motion_model, dt)

        for tid, tx, ty, _ in tags:
            tag_position = complex(tx, ty)
            dp = tag_position - turtle.position()
            distance, theta = cmath.polar(dp)
            angle = norm_angle(theta - turtle.theta)
            if distance < 3 and abs(angle) < 1.2:
                # observe the tag
                print("Tag", tid, distance, angle)
                camera_pos = cmath.rect(distance, angle)
                z = np.array(
                    [[random.normalvariate(camera_pos.real, 0.05)], [random.normalvariate(camera_pos.imag, 0.2)]]
                )
                r = np.diag([0.05, 0.2])
                ekf.correct(z, lambda x: tag_observation_model(x, tag_position), r)

        true_path.update(State(turtle.x, turtle.y, turtle.theta))
        tracked_path.update(State.from_array(ekf.x))
        plot.render()

        # True motion - simulate some omega offset in the mechanics that is not included in the odometry
        turtle.update_speed_omega(speed, omega + 0.05, dt)
        t += dt
    plot.pause()
    plot.show()


if __name__ == "__main__":
    main()
