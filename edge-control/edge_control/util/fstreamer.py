from typing import Tuple

import numpy
from numpy.typing import ArrayLike


class Writer:
    def __init__(self, file, period: int):
        self.file = file
        self.period = period
        self.timestamp = 0

    def write(self, timestamp: int, image: ArrayLike):
        if timestamp >= self.timestamp + self.period:
            self.timestamp = timestamp
            numpy.save(self.file, timestamp)
            numpy.save(self.file, image)


class Reader:
    def __init__(self, file):
        self.file = file

    def read(self) -> Tuple[int, ArrayLike]:
        timestamp = numpy.load(self.file)
        image = numpy.load(self.file)
        return int(timestamp), image

    def stream(self):
        while True:
            yield self.read()
