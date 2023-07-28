from .config import site_config
from .control import CompositeControl
from .control.geometry import *
from .map import geometry


def from_lat_lon(lat: float, lon: float) -> Tuple[float, float]:
    assert site_config.reference, "Site reference missing"
    x, y = site_config.reference.to_site(lat, lon)
    return x, y


def mowing():
    from shapely.geometry import JOIN_STYLE

    # TODO: return exterior and aoi to web GUI and plot, depending on mission
    exterior = geometry.polygon(site_config.exterior)
    # mission config may define an area of interest to mow a selected area
    if mission_config.aoi:
        aoi = exterior.intersection(geometry.polygon(mission_config.aoi))
    else:
        aoi = exterior
    speed = mission_config.control.speed
    omega = mission_config.control.omega
    # make room for a half cutter diameter - may buffer additionally to make sure we don't crash:
    # body width, GPS error, tracking error, control error, ...
    # TODO: move this to .map.geometry
    # TODO: should use the exterior bounds of the robot - plus some buffer margin - and not the cut diameter!
    aoi = aoi.buffer(-0.5 * robot_config.mower.cut_diameter, join_style=JOIN_STYLE.mitre)

    # if plot:
    #     plot.add_shape(aoi, facecolor="darkkhaki")
    #     plot.pause()
    #     # input("Wait for key press")

    # half cut overlap to cover above errors - or use less overlap and assume will cover misses on the next mission
    return FenceShrink(exterior, aoi, speed, omega, -0.5 * robot_config.mower.cut_diameter)


def rectangle_scan():
    speed = mission_config.control.speed
    omega = mission_config.control.omega
    return ScanHLine(-2, -3, 1, -1.25, speed, omega, 0.5 * robot_config.cut_diameter)


def rectangle_loop():
    speed = mission_config.control.speed
    omega = mission_config.control.omega
    return PathControls(geometry.rectangle(0, 1, -2, 2).exterior.coords, speed, omega)


def triangle():

    from .control.controls import LineControl, SpeedDistanceControl

    speed = mission_config.control.speed
    omega = mission_config.control.omega

    def _controls():
        def path():
            # triangle forever
            while True:
                yield -1, 0
                yield -1, 1
                yield -2, 0

        # reverse out of dock, no heading control
        yield SpeedDistanceControl(-0.1, 1)

        p0 = None
        for p in path():
            if p0 is not None:
                yield LineControl(p0, p, speed, omega)
            p0 = p

    return _controls()


def corridor():
    from .control.camera import ImageCaptureControl, capture_gauge
    from .control.controls import AvoidObstacleControl, SpeedDistanceControl, TimeControl2
    from .control.docking import dock
    from .models.ptz import PanTiltZoomPosition

    omega = mission_config.control.omega

    # distance to reverse out of dock and where dock command is issued.
    # also serves as a buffer for position inaccuracy when returning to the dock.
    dock_x = site_config.dock.position.x
    dock_y = site_config.dock.position.y

    undock_x = -0.4
    undock_position = (dock_x + undock_x, dock_y)

    # corridor width is x in 0 - 1.7m, so middle should be x=0.85m.
    path = [
        # avoid Stein's chair and get a view of the tags,
        # also ensure heading slightly to corridor before any obstacle detection of walls, avoiding slipping to IT door
        (1.5, -1.2),
        (0.9, -1.5),  # head to wall but with slight heading to corridor
        (0.9, -2.9),  # along wall, # avoid running straight against windows
        (1.1, -4.0),  # between chair and shelves
        (1.1, -9),  # avoid shelves and boxes
        (0.8, -10),  # towards middle of corridor
        (0.8, -13.6),  # middle of intersection
        (-3.4, -13.6),  # down N corridor
        (-3.4, -14.5),  # against gauge (and tag)
    ]

    # reverse out of dock without heading control
    t, state = yield SpeedDistanceControl(-0.1, abs(undock_x))

    # Ensure turning left towards door to get tag fix
    wait_time = 0.5 * math.pi / omega  # seconds
    t, state = yield TimeControl2(0, omega, t + wait_time)

    # Follow path
    for c in PointControls([undock_position] + path):
        t, state = yield AvoidObstacleControl(c)

    # Head against palm gauge
    t, state = yield HeadingControl(-pi / 2)

    # gauge_id = "tag"
    # pos = PanTiltZoomPosition(0, 0, 1)

    # Gauge on palm above April tag 235
    gauge_id = "gauge:palm"
    pos = PanTiltZoomPosition(0, 10, 1)  # degrees! Should be radians!?
    t, state = yield ImageCaptureControl(capture_gauge(pos, gauge_id))

    # Return to dock
    path.reverse()
    for c in PointControls(path):
        t, state = yield AvoidObstacleControl(c)

    # Run last leg to dock_waypoint until inside IR beam from dock.
    # Don't avoid obstacles from depth camera, but stop (hangs) on bumper.
    t, state = yield dock()


def corridor_short():
    from .control.controls import AvoidObstacleControl, SpeedDistanceControl, TimeControl2
    from .control.docking import dock

    omega = mission_config.control.omega

    # distance to reverse out of dock and where dock command is issued.
    # also serves as a buffer for position inaccuracy when returning to the dock.
    dock_x = site_config.dock.position.x
    dock_y = site_config.dock.position.y

    undock_x = -0.4
    undock_position = (dock_x + undock_x, dock_y)

    # corridor width is x in 0 - 1.7m, so middle should be x=0.85m.
    path = [
        # avoid Stein's chair and get a view of the tags,
        # also ensure heading slightly to corridor before any obstacle detection of walls, avoiding slipping to IT door
        (1.5, -1.2),
        (0.9, -1.5),  # head to wall but with slight heading to corridor
        (0.9, -2.9),  # along wall, # avoid running straight against windows
    ]

    # reverse out of dock without heading control
    t, state = yield SpeedDistanceControl(-0.1, abs(undock_x))

    # Ensure turning left towards door to get tag fix
    wait_time = 0.5 * math.pi / omega  # seconds
    t, state = yield TimeControl2(0, omega, t + wait_time)

    # Follow path
    for c in PointControls([undock_position] + path):
        t, state = yield AvoidObstacleControl(c)

    # Return to dock
    path.reverse()
    for c in PointControls(path):
        t, state = yield AvoidObstacleControl(c)

    # Run last leg to dock_waypoint until inside IR beam from dock.
    # Don't avoid obstacles from depth camera, but stop (hangs) on bumper.
    t, state = yield dock()


def dock_capture():
    from .control.camera import ImageCaptureControl, capture_gauge
    from .control.controls import GetStateControl, TimeControl2
    from .models.ptz import PanTiltZoomPosition

    gauge_id = "dock"
    pos = PanTiltZoomPosition(0, 0, 1)
    wait_time = 5  # seconds

    t, state = yield GetStateControl()
    t, state = yield TimeControl2(0, 0, t + wait_time)
    for i in range(3):
        logger.info("Gauge capture %d", i)
        t, state = yield ImageCaptureControl(capture_gauge(pos, gauge_id))
        t, state = yield TimeControl2(0, 0, t + wait_time)


def office_plaza_lap():
    from .control.controls import LineControl

    speed = mission_config.control.speed
    omega = mission_config.control.omega
    proximity = mission_config.control.proximity

    def points():
        for lap in range(3):
            # Starting point
            yield 0, 2

            # Stairs NE:
            yield 50, 2
            # yield from_lat_lon(59.905026, 10.626389)

            # Stairs NE:
            yield 50, -11
            # yield from_lat_lon(59.904963, 10.626571)

            # Corner inset:
            yield 31, -11
            # yield from_lat_lon(59.904828, 10.626370)

            # Corner:
            yield 31, -20.5
            # yield from_lat_lon(59.904771, 10.626519)
            yield 30, -22

            # Stairs SW:
            yield -29, -22
            # yield from_lat_lon(59.904344, 10.625859)

            # Escalator:
            yield -32, 3
            # yield from_lat_lon(59.904447, 10.625477)

            yield -2, 3

    # TODO: check that the robot starts in the neighborhood of p0
    _points = points()
    p0 = next(_points)
    p1 = p0
    for p2 in _points:
        yield LineControl(p1, p2, speed, omega, proximity)
        p1 = p2

    # back to start
    yield LineControl(p1, p0, speed, omega)


def office_plaza_rectangle():
    from .control.controls import LineControl

    speed = mission_config.control.speed
    omega = mission_config.control.omega
    proximity = mission_config.control.proximity

    p1 = (0, 1)
    p2 = (7, 1)
    p3 = (7, 3)
    p4 = (0, 3)

    for lap in range(3):
        t, state = yield LineControl(p1, p2, speed, omega, proximity)
        t, state = yield LineControl(p2, p3, speed, omega, proximity)
        t, state = yield LineControl(p3, p4, speed, omega, proximity)
        t, state = yield LineControl(p4, p1, speed, omega, proximity)


def turns():
    from .control.controls import ArcControl, TimeControl2

    # speed = mission_config.control.speed
    omega = mission_config.control.omega

    theta = 0
    while True:
        theta += math.pi
        t, state = yield ArcControl(0, omega, theta)
        t, state = yield TimeControl2(0, 0, t + 5)


def dock_test1():
    # reverse out of dock then dock
    from .control.controls import SpeedDistanceControl
    from .control.docking import DockControl

    t, state = yield SpeedDistanceControl(-0.1, 0.3)
    t, state = yield TimeControl2(-0.2, -0.1, t + 2)
    yield DockControl()


def dock_test2():
    from .control.controls import SpeedDistanceControl
    from .control.docking import dock

    t, state = yield SpeedDistanceControl(-0.1, 0.3)
    t, state = yield TimeControl2(-0.2, -0.1, t + 2)
    yield dock()


def turn_360():
    from .control.controls import GetStateControl, TimeControl2

    omega = mission_config.control.omega
    turn_time = 2 * math.pi / omega  # seconds
    t, state = yield GetStateControl()
    for i in range(3):
        t, state = yield TimeControl2(0, omega, t + turn_time)
        t, state = yield TimeControl2(0, 0, t + 3.0)
        t, state = yield TimeControl2(0, -omega, t + turn_time)
        t, state = yield TimeControl2(0, 0, t + 3.0)


_missions = {
    "Mowing": mowing,
    "RectangleScan": rectangle_scan,
    "RectangleLoop": rectangle_loop,
}


async def get_mission(name: str):
    mission = _missions.get(name)
    if mission:
        return CompositeControl(mission())
    raise ValueError("Undefined mission: %s" % name)
