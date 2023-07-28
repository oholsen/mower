from dataclasses import dataclass

from edge_control.util.binmsg import Int32, UInt32, decode


@dataclass
class NavPosLLH:
    iTOW: UInt32
    lon: Int32
    lat: Int32
    height: Int32
    hMSL: Int32
    hAcc: UInt32
    vAcc: UInt32


def test_decode():
    data = bytes.fromhex("42679822f1002d06264da7238a920300a5f802000e0000000e000000")
    msg = decode(NavPosLLH, data, "<")
    assert msg == NavPosLLH(iTOW=580413250, lon=103612657, lat=598166822, height=234122, hMSL=194725, hAcc=14, vAcc=14)
