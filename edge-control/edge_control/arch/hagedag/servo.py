import pigpio

pi = pigpio.pi()


class Servo(object):
    def __init__(self, pin, pulseWidthRange=500):
        # neutral is 1.5ms pulse
        # pulseWidthRange in us
        # +-0.5ms is 90 degrees, depending on servo
        self.pulseWidthRange = pulseWidthRange
        self.pin = pin

    def off(self):
        pi.set_servo_pulsewidth(self.pin, 0)

    def amplitude(self, amplitude):
        # amplitude is -100 to 100 percent
        # neutral is 1.5ms pulse
        amplitude = max(min(amplitude, 100), -100)
        pw = 1500 + self.pulseWidthRange * amplitude / 100
        pi.set_servo_pulsewidth(self.pin, pw)

    def power(self, power):
        # power is 0 to 100 percent across full range, 0 is min, 100 is max
        # mid-point is 1.5ms pulse
        assert abs(power) <= 100
        power = max(min(power / 100, 1), 0)
        pw = 1500 + self.pulseWidthRange * (2 * power - 1)
        pi.set_servo_pulsewidth(self.pin, pw)
