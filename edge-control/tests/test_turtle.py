import math

from pytest import approx

from edge_control.models.turtle import Turtle


def test_circle():
    # simulate one circle in n steps
    radius = 10
    period = 20
    omega = 2 * math.pi / period
    speed = omega * radius
    n = 10
    dt = period / n

    turtle1 = Turtle(radius, 0, math.pi / 2)
    turtle2 = Turtle(radius, 0, math.pi / 2)

    for i in range(n):
        turtle1.update_speed_omega_line(speed, omega, dt)
        turtle2.update_speed_omega(speed, omega, dt)  # slightly more accurate, but more so on random path

    # print(turtle1.y, turtle2.y)  # should be 0

    assert turtle1.x == approx(radius)
    assert turtle1.y == approx(0)
    assert turtle1.theta == approx(2.5 * math.pi)

    assert turtle2.x == approx(radius)
    assert turtle2.y == approx(0)
    assert turtle2.theta == approx(2.5 * math.pi)


def test_spiral():
    period = 20
    omega = 2 * math.pi / period
    speed = 1
    n = 100
    dt = period / n

    turtle1 = Turtle(0, 0, 0)
    turtle2 = Turtle(0, 0, 0)

    for i in range(n):
        turtle1.update_speed_omega_line(speed, omega, dt)
        turtle2.update_speed_omega(speed, omega, dt)  # more accurate
        speed *= 1.01

    # shows inaccuracy in the non-arc method
    # print(turtle1.x, turtle2.x)
    # print(turtle1.y, turtle2.y)

    assert turtle1.x == approx(turtle2.x, abs=0.2)
    assert turtle1.y == approx(turtle2.y, abs=0.05)
    assert turtle1.theta == approx(turtle2.theta)
