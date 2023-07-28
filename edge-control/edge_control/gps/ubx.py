import logging

from dataclasses import dataclass

from edge_control.util.binmsg import Int32, UInt32
from edge_control.util.binmsg import decode as _decode

logger = logging.getLogger(__name__)

# Frame format: 0xb5, 0x62, class, id, payload length, payload, ck a, ck b
# See https://www.u-blox.com/sites/default/files/ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf
FRAME_START1 = 0xB5
FRAME_START2 = 0x62
HEADER_SIZE = 6


def checksum(data: bytes) -> bytes:
    a = 0
    b = 0
    for c in data:
        a += int(c)
        a &= 0xFF
        b += a
        b &= 0xFF
    return bytes((a, b))


class Reader:
    def __init__(self):
        self._buffer = bytes()
        self._skipped = bytes()

    def remaining(self):
        assert len(self._buffer) < 1 or self._buffer[0] == FRAME_START1
        assert len(self._buffer) < 2 or self._buffer[1] == FRAME_START2
        if len(self._buffer) >= HEADER_SIZE:
            # TODO: handle length > 255
            length = int(self._buffer[4])
            return 8 + length - len(self._buffer)
        return HEADER_SIZE - len(self._buffer)

    def skip(self, n):
        self._skipped += bytes(self._buffer[:n])
        self._buffer = self._buffer[n:]

    def skip_all(self):
        self._skipped += self._buffer
        self._buffer = bytes()

    def skip_clear(self):
        if self._skipped:
            logger.debug("Skipping %s", self._skipped)  # displays as ASCII
            # logger.debug("Skipping %s", self._skipped.hex())
            self._skipped = bytes()

    def frames(self, b: bytes):
        self._buffer += b
        while self._buffer:
            i = self._buffer.find(FRAME_START1)
            if i < 0:
                # logger.debug("No start")
                self.skip_all()
                return
            if i > 0:
                # logger.debug("To start")
                self.skip(i)

            assert self._buffer[0] == FRAME_START1
            if len(self._buffer) < 2:
                return

            if self._buffer[1] != FRAME_START2:
                # logger.debug("Invalid start 2")
                self.skip(1)
                continue

            if len(self._buffer) < HEADER_SIZE:
                return

            # length is two bytes little-endian
            # TODO: parse length > 255
            assert self._buffer[5] == 0
            payload_length = int(self._buffer[4])
            frame_length = 8 + payload_length
            if len(self._buffer) < frame_length:
                return

            frame = self._buffer[:frame_length]

            _checksum = checksum(frame[2:-2])
            if _checksum != frame[-2:]:
                # skip first two bytes (second is FRAME_START2) and look for FRAME_START1
                logger.debug("Checksum failed")
                self.skip(2)
                continue

            # Valid frame - check if needed to skip anything to get to this state.
            self.skip_clear()
            yield frame
            self._buffer = self._buffer[frame_length:]


class UbxMessage:
    pass


@dataclass
class NavPosLLH(UbxMessage):
    iTOW: UInt32  # time of week, ms
    lon: Int32  # deg scale 1e-7
    lat: Int32  # deg scale 1e-7
    height: Int32  # mm above ellipsiod
    hMSL: Int32  # mm above mean sea level
    hAcc: UInt32  # mm horizontal accuracy estimate
    vAcc: UInt32  # mm vertical accuracy estimate

    def latitude(self) -> float:
        return self.lat * 1e-7

    def longitude(self) -> float:
        return self.lon * 1e-7

    def altitude(self) -> float:
        return self.hMSL * 1e-3

    def hdop(self) -> float:
        return self.hAcc * 1e-3

    def vdop(self) -> float:
        return self.vAcc * 1e-3


def _message_type(c, i) -> bytes:
    return bytes((c, i))


_message_types = {_message_type(1, 2): NavPosLLH}


def decode(frame: bytes) -> UbxMessage:
    message_type = frame[2:4]
    clz = _message_types.get(message_type)
    if clz is None:
        raise ValueError("Invalid UBX message class and id: " + message_type.hex())
    data = frame[6:-2]
    return _decode(clz, data, "<")
