from pytest import approx

from edge_control.config import mission_config


def test_mission_config():
    # test default and configured parameters
    assert mission_config.on_site.enabled
    assert mission_config.on_site.interval == approx(1)
    assert mission_config.on_site.buffer == approx(10)
