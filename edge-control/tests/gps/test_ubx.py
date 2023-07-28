from pytest import approx

from edge_control.gps.ubx import NavPosLLH, Reader, decode

"""
                      
17:02:26 [18:13:17.060] 0000  24 47 4E 54 58 54 2C 30 31 2C 30 31 2C 30 30 2C  $GNTXT,01,01,00,
                        0010  74 78 62 75 66 20 61 6C 6C 6F 63 2A 36 31 0D 0A  txbuf alloc*61...
                        
17:02:26 [18:13:17.100] 0000  B5 62 01 02 1C 00 42 67 98 22 F1 00 2D 06 26 4D  µb....Bg."ñ.-.&M
                        0010  A7 23 8A 92 03 00 A5 F8 02 00 0E 00 00 00 0E 00  §#....¥ø........
                        0020  00 00 BD A9                                      ..½©.
"""


def test_frames():
    data = bytes.fromhex("b56201021c0042679822f1002d06264da7238a920300a5f802000e0000000e000000bda9")
    reader = Reader()
    frames = list(reader.frames(data))
    assert len(frames) == 1
    frame = frames[0]
    assert frame.hex() == "b56201021c0042679822f1002d06264da7238a920300a5f802000e0000000e000000bda9"


def test_navposllh():
    frame = bytes.fromhex("b56201021c0042679822f1002d06264da7238a920300a5f802000e0000000e000000bda9")
    msg = decode(frame)
    assert msg == NavPosLLH(iTOW=580413250, lon=103612657, lat=598166822, height=234122, hMSL=194725, hAcc=14, vAcc=14)
    assert msg.latitude() == approx(59.8166822)
    assert msg.longitude() == approx(10.3612657)
    assert msg.altitude() == 194.725
    assert msg.hdop() == 0.014
    assert msg.vdop() == 0.014
