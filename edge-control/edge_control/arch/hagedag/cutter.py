import atexit
import logging
import signal

import pigpio

logger = logging.getLogger(__name__)
pi = pigpio.pi()
In1 = 23
In2 = 22
CutterPin = 23


class BrushedCutter:
    # No power regulation, geared motor is too slow at full speed

    def __init__(self):
        pi.set_mode(In1, pigpio.OUTPUT)
        pi.set_mode(In2, pigpio.OUTPUT)

    def off(self):
        pi.write(In1, 0)
        pi.write(In2, 0)

    def stop(self):
        self.off()

    def power(self, power):
        assert abs(power) <= 100
        if power == 0:
            self.stop()
        elif power > 0:
            # forward
            pi.write(In1, 1)
            pi.write(In2, 0)
        else:
            # backward
            pi.write(In1, 0)
            pi.write(In2, 1)


class BrushlessCutterUnidirectional:
    # Forward only (airplane) motor controller
    def __init__(self):
        from .servo import Servo

        self.cutter = Servo(CutterPin)

    def off(self):
        self.cutter.off()

    def stop(self):
        self.cutter.power(0)

    def power(self, power):
        self.cutter.power(power)


class BrushlessCutterBidirectional:
    # Car/boat motor controller with reverse
    def __init__(self):
        from .servo import Servo

        self.cutter = Servo(CutterPin)

    def off(self):
        self.cutter.off()

    def stop(self):
        self.cutter.power(0)

    def power(self, power):
        self.cutter.amplitude(power)


cutter = BrushedCutter()
atexit.register(cutter.stop)


def reset_timeout():
    signal.alarm(5)  # seconds


def alarm_timeout():
    logger.info("Cutter timeout")
    cutter.stop()


def start_watch_dog(loop):
    loop.add_signal_handler(signal.SIGALRM, alarm_timeout)


if __name__ == "__main__":
    import sys

    try:
        cutter.power(float(sys.argv[1]))
    finally:
        cutter.off()
