import time


def now() -> int:
    return time_to_timestamp(time.time())


def time_to_timestamp(t: float) -> int:
    return int(t * 1000)
