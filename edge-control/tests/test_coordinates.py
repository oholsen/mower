from pytest import approx

from edge_control.gps.coordinates import LatLon


def test_utm_add_diff():
    u0 = LatLon(60, 10).utm()
    u1 = u0.add(100, 50)
    de, dn = u1.diff(u0)
    assert (de, dn) == (100, 50)


def test_utm_latlon():
    u0 = LatLon(60, 10).utm()
    ll = u0.latlon()
    assert ll.lat == approx(60)
    assert ll.lon == approx(10)
