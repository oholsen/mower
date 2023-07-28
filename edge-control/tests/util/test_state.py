from edge_control.models.state import State


def test_distance():
    s1 = State(1, 2, 0)
    s2 = State(4, 6, 1)
    assert s1.distance(s2) == 5
