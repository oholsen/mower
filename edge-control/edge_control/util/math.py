from math import pi


def norm_angle(a: float) -> float:
    while a > pi:
        a -= 2 * pi
    while a <= -pi:
        a += 2 * pi
    return a


def a2s(x):
    """Convert sequence (numpy.array) to space-separated string (for simple parsing)"""
    return " ".join(str(e) for e in x)
