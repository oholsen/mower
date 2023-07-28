import logging
from datetime import timedelta

from edge_control import config
from edge_control.control import CompositeControl, Control
from edge_control.control.geometry import LineControl, PointControl2
from edge_control.models.state import State

from .plotting import Plot, States

logger = logging.getLogger(__name__)

tracked_path = States()
plot = Plot([tracked_path])
config.gps_config = None

# tune with mission config - theta_distance controls radius, proximity when to switch control
# speed_theta, speed, omega controls overshoot.
proximity = 1
p0 = (0, 0)
p1 = (5, 0)
p2 = (5, 5)
p3 = (3, 8)


def controls2():
    yield PointControl2(p0, p1, proximity)
    yield PointControl2(p1, p2, proximity)
    yield PointControl2(p2, p3)


def controls1():
    speed = config.mission_config.control.speed
    omega = config.mission_config.control.omega
    yield LineControl(p0, p1, speed, omega, proximity)
    yield LineControl(p1, p2, speed, omega, proximity)
    yield LineControl(p2, p3, speed, omega)


controls = controls1


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


def run_simulation(control: Control, dt):
    """Run in simulated time, much quicker than real time"""
    from edge_control.arch.simulation.robot import SimulatedRobot

    logger.info("Starting robot simulation with time step %.3f...", dt)
    logger.info("Mission config: %s", config.mission_config)
    t = 0.0
    robot = SimulatedRobot(0, 0, 0)
    state = robot.state()
    log(t, state, robot.speed, robot.omega)
    tracked_path.update(state)
    while not control.end(t, state):
        t += dt
        robot.update(dt)
        state = robot.state()
        log(t, state, robot.speed, robot.omega)
        tracked_path.update(state)
        plot.render()
        command = control.update(t, state)
        if command:
            command.timeout = 0  # running in simulated time, disable time check
            robot.do_command(command)
    logging.info("Mission %s completed in %s", control, timedelta(seconds=t))


def parse_args():
    import argparse

    parser = argparse.ArgumentParser(
        prog="control",
        description="Control dev tool",
    )
    parser.add_argument("--config", type=str, help="Configuration directory")
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--dt", type=float, default=0.2, help="Simulation time step")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    return parser.parse_args()


def main():
    import os

    from edge_control.util.config import config_logging

    args = parse_args()

    if args.config:
        os.environ["CONFIG_DIR"] = args.config
    config_logging("logging.yaml", args.verbose)
    plot.frames_per_plot = args.speed

    control = CompositeControl(controls())
    run_simulation(control, args.dt)
    plot.pause()
    plot.show()


if __name__ == "__main__":
    main()
