import cmath
import math
import re

import numpy as np
from scipy.optimize import minimize

from edge_control.config import tag_config
from edge_control.realsense.T265 import Frame, Pose, Quaternion, Tag, TagPose  # for eval()
from edge_control.util.math import a2s


def triangulate(x0, pairs):

    # camera t is (right, down, away)
    camera_world_height = 1.17
    for tag, pos in pairs:
        delta_height = camera_world_height - tag.camera.t[1] - pos[2]
        if abs(delta_height) > 0.1:
            print("HEIGHT DELTA", tag.tagId, delta_height)

    def error(camera_pos: complex, camera_yaw: float, debug=False) -> float:
        # pos in world coords
        # yaw in radians in world coords
        _error = 0.0
        for tag, pos in pairs:
            tc = tag.camera.t
            pos_tag = camera_pos + complex(tc[2], -tc[0]) * cmath.rect(1, camera_yaw)
            pos_real = complex(pos[0], pos[1])
            delta = pos_tag - pos_real
            if debug:
                print("DELTA", tag.tagId, abs(delta), delta, pos_tag, pos_real)
            _error += abs(delta) ** 2
            # print("_ERROR", _error)
        return _error

    def error_vector(x, debug=False) -> float:
        return error(complex(x[0], x[1]), x[2], debug)

    e0 = error_vector(x0)
    print("ERROR0", e0)
    res = minimize(error_vector, x0)
    # res = minimize(error_vector, x0, method='nelder-mead', options={'xatol': 1e-8, 'disp': True})
    # print("MIN", res)
    e = res.fun
    x = res.x
    print("ERROR", e)
    _e = error_vector(x, debug=True)
    assert res.success
    return res.x, res.fun


# Site coordinates when in dock (origin for realsense).
# Offset only to be applied to old logs before the offset was added in realsense.driver.
dock_x = 2.51
dock_y = -1.70
pos_dock = complex(dock_x, dock_y)


def parse_args():
    import argparse

    parser = argparse.ArgumentParser(
        prog="tags",
        description="Offline Apriltag analysis from robot log",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    parser.add_argument("-o", "--offset", action="store_true", help="Offset old logs with dock position")
    parser.add_argument("filename")
    return parser.parse_args()


def main():
    args = parse_args()

    _tags = dict((t.id, t) for t in tag_config.tags)
    assert len(_tags) == len(tag_config.tags), "Duplicate tag ids in tag config"

    pose = None
    timestamp = None
    _cols = re.compile(r" +")

    for line in open(args.filename):
        line = line.strip()
        cols = _cols.split(line)
        if len(cols) < 4:
            print("SKIP LINE", line)
            continue
        module = cols[3]
        if module == "edge_control.realsense.driver2" and cols[4] == "vpose":
            pose = cols[5:]
            timestamp = " ".join(cols[:2])
            print(timestamp, "POSE", " ".join(pose))
            continue

        if module == "edge_control.realsense.driver2" and cols[4] == "tags:":
            frame = eval("".join(cols[5:]))
            # print("TAGS", frame)
            tags = frame.tags
            if len(tags) < 2:
                continue
            print()
            # print(timestamp, "POSE", " ".join(pose))
            # print("TAGS", tags)
            pairs = []
            for tag in tags:
                _tag = _tags.get(tag.tagId)
                if _tag:
                    p = _tag.position
                    world_pos = np.array([p.x, p.y, p.z])
                    # ignore low tags, as duplicates on thin paper from Spot AND
                    # does not provide any more support if there is also one at camera height
                    if p.z < 0.7:
                        continue
                    print("CAMERA", tag.tagId, a2s(tag.camera.t))
                    pairs.append((tag, world_pos))
            print("TRIANGULATE", len(pairs))
            if len(pairs) < 2:
                continue

            confidence = int(pose[0])
            pos0 = complex(float(pose[1]), float(pose[2]))
            if args.offset:
                pos0 += pos_dock
            yaw0 = math.radians(float(pose[4]))
            x0 = np.array([pos0.real, pos0.imag, yaw0])
            x, e = triangulate(x0, pairs)
            pos = complex(x[0], x[1])
            yaw = x[2]
            print("ERROR", e)
            print("MOVE", abs(pos - pos0), yaw - yaw0, pos0.real, pos0.imag, pos - pos0, e)
            # if e < 0.05:
            print(
                timestamp,
                "POSN",
                pos0.real,
                pos0.imag,
                yaw0,
                pos.real,
                pos.imag,
                yaw,
                confidence,
            )
            continue


if __name__ == "__main__":
    main()
