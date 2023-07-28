from pytest import approx

from edge_control.config import SiteConfig, SiteReferenceConfig

reference = SiteReferenceConfig(59.5, 10.2, 90)


def site_approx(x):
    return approx(x, abs=0.01)


def test_origin():
    o = reference._utm0
    x, y = reference.to_site(reference.latitude, reference.longitude)
    assert x == 0
    assert y == 0
    assert reference.to_world(0, 0) == o


def test_map():
    o = reference._utm0
    de = 100
    dn = 20
    ll = o.add(de, dn).latlon()
    x, y = reference.to_site(ll.lat, ll.lon)
    assert x == site_approx(-20)
    assert y == site_approx(100)
    u2 = reference.to_world(x, y)
    de2, dn2 = u2.diff(o)
    assert de2 == site_approx(de)
    assert dn2 == site_approx(dn)


def test_office_plaza():
    site = SiteConfig.load("config/sites/office-plaza/site.yaml")
    # Fence/stairs corner to the left outside main entrance
    x, y = site.reference.to_site(59.905002, 10.626391)
    assert x == site_approx(47.434)
    assert y == site_approx(-0.005)

    assert len(site.exterior) == 11
    assert len(site.interiors) == 3
    assert set(len(interior) for interior in site.interiors) == {5, 15, 9}

    assert site.on_site(1, 1, 0.4)
    assert not site.on_site(-1, -1, 0.4)
