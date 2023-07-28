import logging
import random

from shapely.geometry import JOIN_STYLE, LineString, MultiPolygon, Point, Polygon

from ..map import geometry
from .controls import *

_logger = logging.getLogger(__name__)


def LineTest(xl, xr, y0):
    end_x = {True: xr, False: xl}
    speed_max = 0.20
    omega_max = 0.20
    t, state = yield GetStateControl()
    right = state.x < (xl + xr) / 2
    while True:
        t, state = yield HLineControl(y0, right, end_x[right])
        t, state = yield TimeControl(0, omega_max * random.uniform(-1, 1), t + 3)
        t, state = yield TimeControl(speed_max * random.uniform(0.1, 1), 0, t + 4)
        t, state = yield TimeControl(0, omega_max * random.uniform(-1, 1), t + 3)
        right = not right


def ScanHLine(x0, y0, x1, y1, speed, omega, dy):
    assert x1 > x0
    assert y1 > y0
    y = y0
    right = True
    end_x = {True: x1, False: x0}
    end_theta = {True: 0, False: pi}
    while True:
        # TODO: dummy first control, with end() just to get state?
        yield HLineControl(y, right, end_x[right], speed, omega)
        y += dy
        right = not right
        if y > y1:
            y = y0
        else:
            s, o = start_arc(dy / 2, speed, omega, right)
            yield ArcControl(s, -o, end_theta[right])


def end_inside(poly: Polygon):
    def f(t: float, state: State):
        return poly.contains(Point(state.x, state.y))

    return f


def end_outside(poly: Polygon):
    def f(t: float, state: State):
        return not poly.contains(Point(state.x, state.y))

    return f


def RingControls(coords, speed, omega):
    x0, y0 = coords[0]
    yield PointControl(x0, y0, speed, omega, distance(x0, y0, 0.2))
    # aim a little beyond the desired point to make sure it crosses the line before turning wildly to reach the point
    ahead = 0.2  # meters
    for x, y in coords[1:]:
        # x1,y1 is x0,y0 mirrored around perpendicular at x,y - crossing perpendicular when closer to x1,y1
        dx = x - x0
        dy = y - y0
        x1 = x + dx
        y1 = y + dy
        d = math.hypot(dx, dy)
        yield PointControl(
            x + ahead * dx / d,
            y + ahead * dy / d,
            speed,
            omega,
            nearest(x0, y0, x1, y1),
        )
        x0 = x
        y0 = y


def PathControls(coords, speed, omega):
    p0 = coords[0]
    for p in coords[1:]:
        yield LineControl(p0, p, speed, omega)
        p0 = p


def PointControls(coords):
    p0 = coords[0]
    for p in coords[1:]:
        yield PointControl2(p0, p)
        p0 = p


def FenceShrink(limits, aoi, speed: float, omega: float, shrink: float):

    assert shrink < 0, "Fence shrink must be negative: %g" % shrink

    _, state = yield GetStateControl()

    area = 0
    shapes = [aoi]
    while shapes:
        # select the closest area:
        _, shape = min((s.distance(Point(state.x, state.y)), s) for s in shapes)
        shapes.remove(shape)
        lap = 1
        area += 1
        while True:
            if isinstance(shape, MultiPolygon):
                _logger.info("Area %d MultiPolygon %d", area, len(shape.geoms))
                shapes.extend(shape.geoms)
                break

            _logger.info("Area %d lap %d area %g", area, lap, shape.area)
            # find nearest point in shape
            # l = [(math.hypot(state.x - x, state.y - y), i, x, y) for i, (x, y) in enumerate(shape.exterior.coords)]
            # l.sort()
            assert shape.exterior.coords[0] == shape.exterior.coords[-1]
            _, i = min((math.hypot(state.x - x, state.y - y), i) for i, (x, y) in enumerate(shape.exterior.coords[:-1]))

            # shuffle coordinates (rotate) to start with the nearest point
            coords = shape.exterior.coords[i:-1] + shape.exterior.coords[:i]
            coords.append(coords[0])
            assert len(coords) == len(shape.exterior.coords)

            path = geometry.path((state.x, state.y), coords[0], limits)
            if path is None:
                _logger.warning("Area %d: no path to first point %s", area, coords[0])
                break

            # move along path to first point in ring
            for c in PathControls(path.coords, speed, omega):
                _, state = yield c

            # move around ring
            for c in PathControls(coords, speed, omega):
                _, state = yield c

            # make a smaller ring
            shape2 = shape.buffer(shrink, join_style=JOIN_STYLE.mitre)

            if shape2.area < 0.0001:
                _logger.info("Area %d almost completed: area %g", area, shape2.area)
                shape2 = shape.buffer(shrink / 2, join_style=JOIN_STYLE.mitre)
                _logger.info("Area %d reduced shrink: area %g", area, shape2.area)

            if shape2.area < 0.0001:
                # also covers area==0 with no coords
                _logger.info("Area %d completed: area %g", area, shape2.area)
                break

            shape = shape2
            lap += 1
