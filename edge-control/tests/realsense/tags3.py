import logging
import re
from datetime import datetime

import numpy as np

from edge_control.models.messages import MoveCommand, Odometry, StopCommand  # for eval()
from edge_control.models.turtle import Turtle
from edge_control.util.math import a2s
from tests.plotting import Plot, States


def parse_args():
    import argparse

    parser = argparse.ArgumentParser(
        prog="tags",
        description="Offline Apriltag tracking from robot log using EKF",
    )
    parser.add_argument("--after", type=str, help="Plot after timestamp")
    parser.add_argument("--config", type=str, help="Configuration directory")
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    parser.add_argument("filename")
    return parser.parse_args()


def main():
    import os

    args = parse_args()
    if args.config:
        os.environ["CONFIG_DIR"] = args.config
    logging.basicConfig(level=logging.DEBUG)

    # Delay imports to use config directory argument
    from edge_control.config import RoombaConfig, robot_config, site_config, tag_config
    from edge_control.models.tracking import ExtendedKalmanFilter, State, motion_model, tag_observation_model
    from edge_control.realsense.driver3 import position
    from edge_control.realsense.T265 import Frame, Pose, Quaternion, Tag, TagPose  # for eval()

    _tags = dict((t.id, t) for t in tag_config.tags)
    assert len(_tags) == len(tag_config.tags), "Duplicate tag ids in tag config"
    print("TAGDB", _tags.keys())

    x0 = np.array([[site_config.dock.position.x], [site_config.dock.position.y], [site_config.dock.heading]])
    r0 = np.diag([0.05, 0.05, 0.1])  # in docking station
    ekf = ExtendedKalmanFilter(x0, r0)
    # print("EKF", ekf.x, ekf.P)
    odometer = Turtle()

    # offline estimate here
    tracked_path = States(".r", "r")
    odom_path = States(".m", "m")

    # from log file as recorded by robot:
    state_path = States(".y", "y")  # as tracked by robot
    vpose_path = States(".c", "c")  # RS T265
    turtle_path = States(".b", "b")  #

    plot = Plot([tracked_path, odom_path, state_path, vpose_path, turtle_path])
    plot.frames_per_plot = args.speed
    _odom_time = None

    roomba_config = robot_config.roomba if robot_config.roomba else RoombaConfig()
    wheel_odometry = True  # if false, use autonomous mission control odometry
    _module = "edge_control.realsense.driver3"
    _cols = re.compile(r" +")

    for line in open(args.filename):
        # print("LINE", line)
        line = line.strip()
        cols = _cols.split(line)
        if len(cols) < 4:
            print("SKIP LINE", line)
            continue
        module = cols[3]

        timestamp = datetime.fromisoformat(" ".join(cols[:2]))

        def show():
            print("TRACK", a2s(ekf.x.flatten()))
            if not args.after or line >= args.after:
                plot.render()

        def odometry(dt: float, speed: float, omega: float):
            print(timestamp, "odometry", dt, speed, omega)
            u = np.array([[speed], [omega]])
            ekf.predict(u, motion_model, dt)
            tracked_path.update(State.from_array(ekf.x))
            show()

        if module == "edge_control.arch.roomba.driver":
            if cols[4] == "turtle":
                # edge_control.arch.roomba.driver turtle 0.000 0.000 0.000
                turtle_path.update(State(*map(float, cols[5:])))
            elif cols[4] == "encoder":
                print("ENCODER", cols[5:])
                # edge_control.arch.roomba.driver encoder 0.20280 0 0
                ticks_to_m = 0.072 * np.pi / 508.8  # wheel diameter is 72 mm, 508.8 ticks/rev
                dt = float(cols[5])
                tl, tr, _, _ = map(int, cols[6:])
                ticks_to_v = ticks_to_m / dt
                vl = roomba_config.wheel_scale_left * tl * ticks_to_v
                vr = roomba_config.wheel_scale_right * tr * ticks_to_v
                speed = (vl + vr) / 2
                omega = (vr - vl) / robot_config.wheel_base
                odometer.update_speed_omega(speed, omega, dt)
                odom_path.update(odometer.state())

            continue

        if not wheel_odometry and module == "edge_control.missioncontrol" and " ".join(cols[4:6]) == "Control command:":
            # When odometry is absent from old logs
            # edge_control.missioncontrol Control command: MoveCommand(timeout=1613047822.574416, speed=-0.1, omega=0)
            cmd = "".join(cols[6:])
            if not cmd.startswith("StopCommand(") and not cmd.startswith("MoveCommand("):
                continue
            # StopCommand = MoveCommand  # str(StopCommand()) includes speed, omega, but not allowed in constructor
            move_command = eval(cmd)
            print(timestamp, "MOVE", move_command)
            if _odom_time is not None:
                dt = (timestamp - _odom_time).total_seconds()
                odometry(dt, move_command.speed, move_command.omega)
            _odom_time = timestamp
            continue

        if module != _module:
            continue

        if wheel_odometry and cols[4] == "odometry":
            dt = float(cols[5])
            odom = eval("".join(cols[6:]))
            print(timestamp, "ODOM", dt, odom)
            odometry(dt, odom.speed, odom.omega)

        elif cols[4] == "tags":
            frame = eval("".join(cols[5:]))
            print("TAGS", frame)
            tags = frame.tags
            for tag in tags:
                position(ekf, tag)
            tracked_path.update(State.from_array(ekf.x))
            show()

        elif cols[4] == "state":
            # TODO: compare state of recording with that calculated here
            print("STATE", cols[5:])
            t, x, y, heading = map(float, cols[5:])
            state = State(x, y, heading)
            state_path.update(state)
            show()

        elif cols[4] == "vpose":
            # edge_control.realsense.driver3 vpose 2 0.890 -1.493 -0.014 -161.2 2.3 0.3
            confidence = int(cols[5])
            # YPR in degrees
            x, y, z, yaw, pitch, roll = map(float, cols[6:])
            state = State(x, y, np.deg2rad(yaw))
            vpose_path.update(state)
            show()

    plot.show()


if __name__ == "__main__":
    main()
