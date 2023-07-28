from edge_control.util.status import Counter, Status


def test_set_get():
    s = Status[int]()

    assert not s.present()
    assert s.get() is None

    s.set(23, 100.0)
    assert s.present(110.0)
    assert s.get(110.0) == 23
    assert s.get(170.0) is None
    assert not s.present(200.0)


def test_clz():
    s = Status[int]()
    assert s.clz() == int


def test_inc():
    s = Counter()
    assert s.get() is None

    assert s.inc(2) == 1
    assert s.get(3) == 1
    assert s.get(200) is None

    assert s.inc(23) == 2
    assert s.get(24) == 2
    assert s.get(200) is None
