import logging

from . import storage
from .config import gps_config, robot_config, site_config
from .missioncontrol import mission_control
from .robot import start as start_robot_state
from .util.tasks import start_task

logger = logging.getLogger(__name__)


async def start():
    # For every message from GPS/robot or on timeout
    # run control loop and provide output to robot - OR - on fail safe, stop robot
    # (possibly with the option to resume the control loop, saving the last known good position)

    from . import api

    await api.serve("", 18888)
    start_robot_state()
    start_task(storage.store())
    start_task(mission_control())

    if gps_config:
        from .gps.driver import gps_driver
        from .gps.site import world_to_site
        from .models.tracking import RobotTracker, run_tracker

        logger.info("Starting GPS...")
        assert site_config
        start_task(world_to_site())
        # tracker not required for realsense, but for GPS and simulation
        start_task(run_tracker(RobotTracker(gps_config.offset)))
        if gps_config.gps:
            start_task(gps_driver(gps_config))

    if robot_config.realsense:
        from .realsense.driver import driver as pos_driver

        start_task(pos_driver())

    logger.info("Starting tasks for robot type %s", robot_config.type)

    if robot_config.type == "simulation":
        from .arch.simulation.robot import SimulatedRobot

        robot = SimulatedRobot.from_dock()
        start_task(robot.run_real_time(0.5))

    elif robot_config.type == "hagedag":
        from .arch.hagedag.driver import motor_driver

        start_task(motor_driver())

    elif robot_config.type == "husqvarna":
        from .arch.husqvarna.tasks import start

        start_task(start())

    else:
        raise Exception("Invalid robot type: " + robot_config.type)
