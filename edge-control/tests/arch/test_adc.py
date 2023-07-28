from edge_control.arch.roomba.adc import decode


def test_query_list():
    assert decode("CH6:1467\t1.186V") == (6, 1467)
