import asyncio
import logging
from datetime import timedelta

from edge_control.control import Control
from edge_control.models.state import State

logger = logging.getLogger(__name__)

# TODO: define abstract base class to get typing right below without loading mathplotlib
plot = None  # tttype: Optional[Plot] - move singleton...
tracked_path = None  # tttype: Optional[States] - move singleton...


def log(t: float, state: State, speed: float, omega: float):
    logger.debug(
        "state %.3f %.3f %.3f %.3f %.3f %.3f",
        t,
        state.x,
        state.y,
        state.theta,
        speed,
        omega,
    )


def plot_map():
    from edge_control.config import site_config
    from edge_control.map.geometry import polygon

    plot.add_shape(polygon(site_config.exterior), facecolor="khaki")
    for interior in site_config.interiors:
        plot.add_shape(polygon(interior), facecolor="white")
    plot.pause()


def run_simulation(control: Control, dt: float):
    """Run in simulated time, much quicker than real time"""
    from edge_control.arch.simulation.robot import SimulatedRobot
    from edge_control.config import gps_config
    from edge_control.models.tracking import RobotTracker

    robot = SimulatedRobot.from_dock()
    logger.info("Starting robot simulation with time step %.3f...", dt)
    t = 0.0
    state = robot.state()
    assert gps_config
    tracker = RobotTracker(gps_config.offset, state)
    log(t, state, robot.speed, robot.omega)
    if tracked_path:
        tracked_path.update(state)
    while not control.end(t, state):
        t += dt
        robot.update(dt)

        # state = robot.state()  # By-pass EKF
        tracker.update(robot.site(), robot.speed, robot.omega, dt)
        state = tracker.get_state()

        log(t, state, robot.speed, robot.omega)
        if tracked_path:
            assert plot is not None
            tracked_path.update(state)
            plot.render()
        command = control.update(t, state)
        if command:
            command.timeout = 0  # running in simulated time, disable time check
            robot.do_command(command)
    logging.info("Mission %s completed in %s", control, timedelta(seconds=t))


def mission_control(args):
    from edge_control.missioncontrol import realtime_control
    from edge_control.missions import get_mission
    from edge_control.tasks import start
    from edge_control.util import tasks

    mission_name = args.mission[0]
    logger.info("Starting mission control for %s", mission_name)

    if args.sim:

        async def _run():
            controls = await get_mission(mission_name)
            run_simulation(controls, args.dt)

    else:

        async def _run():
            controls = await get_mission(mission_name)
            await asyncio.wait(
                [
                    start(),
                    realtime_control(controls),
                ]
            )

    loop = asyncio.get_event_loop()
    loop.set_exception_handler(tasks.handle_exception)
    asyncio.run(_run(), debug=args.verbose)
    asyncio.run(tasks.shutdown())

    if plot:
        plot.pause()
        plot.show()


def parse_args():
    import argparse

    parser = argparse.ArgumentParser(
        prog="edge-control",
        description="Husqvarna robot edge control service",
    )
    parser.add_argument("--config", type=str, help="Configuration directory")
    parser.add_argument("--plot", action="store_true", help="Plot a map view")
    parser.add_argument("--sim", action="store_true", help="Simulate robot")
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--dt", type=float, default=0.5, help="Simulation time step")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    parser.add_argument("mission", type=str, nargs=1)
    # TODO: reference site from mission!?
    # TODO: argument for config directory??? Have a separate stable config setup for CI tests.
    return parser.parse_args()


def main():
    import os

    from edge_control.util.config import config_logging

    args = parse_args()

    if args.config:
        os.environ["CONFIG_DIR"] = args.config
    config_logging("logging.yaml")

    if args.plot:
        # only depend on mathplotlib if --plot on cmd line
        from .plotting import Plot, States

        global plot, tracked_path
        tracked_path = States()
        plot = Plot([tracked_path], frames_per_plot=args.speed)
        plot_map()

    mission_control(args)


if __name__ == "__main__":
    main()
