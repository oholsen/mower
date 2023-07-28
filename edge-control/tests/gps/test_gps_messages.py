from edge_control.gps.messages import checksum, gga_nmea, process


def test_checksum():
    # https://nmeachecksum.eqth.net/
    nmea = "GPGGA,154645.50,5948.99814,N,01021.67878,E,1,12,0.70,201.8,M,39.4,M,,"
    assert checksum(nmea) == "56"


def test_gga():
    nmea = "$GNGGA,140416.00,5948.99861,N,01021.67811,E,4,12,0.59,192.9,M,39.4,M,1.0,1405*68"
    gga = process(nmea)
    assert gga.lat == 59.8166435
    assert gga.lon == 10.361301833333334
    assert gga.quality == 4
    assert gga.alt == 192.9
    assert gga.time == 50656


def test_rmc():
    nmea = "$GNRMC,140417.00,A,5948.99864,N,01021.67811,E,0.068,,250620,,,R,V*0C"
    rmc = process(nmea)
    assert rmc.speed_knots == 0.068
    assert rmc.course_over_ground is None
    assert rmc.date == "250620"
    assert rmc.mode == "R"


def test_gngga_nmea():
    nmea = gga_nmea(59.5, 10.25)
    gga = process(nmea)
    assert gga.lat == 59.5
    assert gga.lon == 10.25


def test_gpgga_nmea():
    nmea = gga_nmea(59.1, 10.2, "GPGGA")
    gga = process(nmea)
    assert gga.lat == 59.1
    assert gga.lon == 10.2


def test_nofix_gpgga():
    # from Jackal
    nmea = "$GPGGA,021103.899,,,,,0,0,,,M,,M,,*41"
    gga = process(nmea)
    assert gga.time == 7863.899
    assert gga.lat is None
    assert gga.lon is None


def test_rtk_gga_init():
    nmea = "$GNGGA,,,,,,0,00,99.99,,,,,,*56"
    gga = process(nmea)
    assert gga.time is None
    assert gga.lat is None
    assert gga.lon is None
