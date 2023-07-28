class PID:
    """A simple PID controller."""

    def __init__(self, Kp, Ti, Td, Imax):
        self.Kp = Kp
        self.Ti = Ti
        self.Td = Td
        self.Imax = Imax
        self.clear()

    def clear(self):
        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0
        self.previous_error = None

    def update(self, error, dt):
        # error is y - y_ref, apply negative feedback outside: u = -pid.update(y - y_ref, dt)
        self.Cp = self.Kp * error
        self.Ci += self.Kp * error * dt / self.Ti
        self.Ci = min(self.Ci, self.Imax)
        self.Ci = max(self.Ci, -self.Imax)
        if self.previous_error is not None:
            de = error - self.previous_error
            self.Cd = self.Kp * self.Td * de / dt
        self.previous_error = error

        return self.Cp + self.Ci + self.Cd
