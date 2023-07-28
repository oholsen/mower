import numpy
from pytest import approx

from edge_control.config import Vector3D
from edge_control.realsense.T265 import Frame, Pose, Quaternion, Tag, TagPose, to_site_euler, to_site_position
from edge_control.realsense.tags import distance, position, tag_position


def test_pose():
    json = '{"frame":1380,"timestamp":1601456003782,"pose":{"confidence":3,"t":[0.0966,-0.0211,0.696],"r":[-0.00525,-0.99,0.0107,0.138]}}'
    frame = Frame.from_json(json)
    assert frame.frame == 1380
    assert frame.timestamp == 1601456003782
    assert frame.tags is None
    assert frame.pose.confidence == 3
    assert len(frame.pose.t) == 3
    assert len(frame.pose.r) == 4
    assert frame.pose.site_heading() == approx(-2.8644, 0.001)


def test_pose2():
    # string issue in r !!
    json = '{"frame":198,"timestamp":1601885681698,"pose":{"confidence":2,"t":[-8.24e-05,0.000149,0.000181],"r":[0.0209,-8e-05,0.00482,1]}}'
    frame = Frame.from_json(json)
    assert frame.frame == 198
    assert frame.tags is None
    assert frame.pose.confidence == 2
    assert len(frame.pose.t) == 3
    assert len(frame.pose.r) == 4


def test_tags():
    json = """
{
    "frame":1380,"timestamp":1601456003782,
    "tags":[
        {"tagId":0, "camera":{"r":[0.948,-0.021,0.318,0.013,1.000,0.027,-0.318,-0.021,0.948],"t":[-0.120,-0.189,1.936]}, 
         "world":{"r":[-0.999,0.010,-0.041,-0.010,-1.000,-0.009,-0.041,-0.008,0.999],"t":[0.783,0.198,2.510]}}, 
        {"tagId":210, "camera":{"r":[0.980,-0.011,0.196,-0.000,0.998,0.056,-0.197,-0.055,0.979],"t":[-0.473,-0.450,2.021]}, 
         "world":{"r":[-0.996,-0.009,0.084,0.005,-0.999,-0.039,0.085,-0.038,0.996],"t":[1.147,0.457,2.488]}}, 
        {"tagId":212, "camera":{"r":[0.964,-0.042,0.264,0.010,0.992,0.122,-0.267,-0.115,0.957],"t":[0.458,-0.439,1.769]}, 
         "world":{"r":[-1.000,0.004,0.013,-0.006,-0.995,-0.104,0.012,-0.104,0.994],"t":[0.183,0.449,2.506]}}, 
        {"tagId":213, "camera":{"r":[0.995,-0.026,-0.094,0.016,0.995,-0.103,0.097,0.101,0.990],"t":[0.004,0.183,1.917]}, 
         "world":{"r":[-0.929,0.049,0.367,-0.006,-0.993,0.118,0.370,0.108,0.923],"t":[0.658,-0.174,2.532]}}]
}
"""
    frame = Frame.from_json(json)
    assert frame.frame == 1380
    assert frame.timestamp == 1601456003782
    assert frame.pose is None
    assert len(frame.tags) == 4
    assert [t.tagId for t in frame.tags] == [0, 210, 212, 213]
    assert len(frame.tags[0].camera.r) == 9
    assert len(frame.tags[0].camera.t) == 3
    assert len(frame.tags[0].world.t) == 3


def _test_driver2_position():
    # DISABLED: requires scipy dependency

    # NOTE: with origin in dock, before realigning origin to corner of office
    # 2020-10-29 12:40:55.730
    # Facing IT wall with two tags in view

    # Docked pose with > 0.5m error:
    # 2020-10-29 12:42:50.064 DEBUG    edge_control.realsense.driver pose 3 0.246 -0.161 -0.337 -6.9 -1.7 -176.7
    # Tracking pose with error:
    # 2020-10-29 12:40:55.069 DEBUG    edge_control.realsense.driver pose 3 -0.879 0.559 -0.339 81.0 0.7 -178.6
    # Frame(frame=21900, timestamp=1603971655395, pose=None, tags=[])

    # t is (right, down, away)
    tag1 = Tag(
        tagId=229,
        camera=TagPose(
            t=[0.463, 0.245, 0.902],
            r=[0.99, 0.008, -0.141, -0.006, 1.0, 0.014, 0.141, -0.013, 0.99],
        ),
        world=TagPose(
            t=[-1.384, -0.566, 0.337],
            r=[-0.009, 0.005, -1.0, 0.025, -1.0, -0.005, -1.0, -0.025, 0.009],
        ),
    )

    tag2 = Tag(
        tagId=236,
        camera=TagPose(
            t=[-0.411, 0.12, 0.784],
            r=[0.988, -0.026, -0.151, 0.024, 1.0, -0.01, 0.152, 0.006, 0.988],
        ),
        world=TagPose(
            t=[-1.381, -0.458, 1.222],
            r=[-0.02, -0.018, -1.0, -0.005, -1.0, 0.018, -1.0, 0.006, 0.019],
        ),
    )

    # tag1_world = numpy.array([-0.64, 1.59, 0.93])
    # tag2_world = numpy.array([-1.60, 1.65, 1.05])
    tag1_pos = complex(-0.64, 1.59)
    tag2_pos = complex(-1.60, 1.65)
    # dock     pose 3 0.246 -0.161 -0.337 -6.9 -1.7 -176.7
    # tracking pose 3 -0.879 0.559 -0.339 81.0 0.7 -178.6
    x1 = numpy.array([-1.3, 0.8, 1.4])
    x0 = numpy.array([-0.8, 0.5, 1.1])
    x = position(x0, [(tag1, tag1_pos), (tag2, tag2_pos)], Vector3D(0, 0, 0))
    assert distance(x, x1) < 0.13
