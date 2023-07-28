from struct import unpack_from


class UInt32(int):
    format = "L"


class Int32(int):
    format = "l"


class UInt16(int):
    format = "H"


class Int16(int):
    format = "h"


class UInt8(int):
    format = "B"


class Int8(int):
    format = "b"


def decode(clz, data: bytes, endian=""):
    """Decode binary data defined in a dataclass defined in terms of the above fields"""
    fmt = "".join(t.format for t in clz.__annotations__.values())
    return clz(*unpack_from(endian + fmt, data))
