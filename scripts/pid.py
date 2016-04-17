from time import time

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.setpoint = None

        self.error_prev = None
        self.error_sum = 0.0
        self.time_prev = 0.0

    def calc(self, state):
        if self.setpoint == None:
            return 0

        time_curr = time()

        error = self.setpoint - state

        effort = self.Kp * error + self.Ki * self.error_sum
        if self.error_prev != None and time_curr > self.time_prev:
            dt = time_curr - self.time_prev
            effort += (error - self.error_prev) * self.Kd / dt
            self.error_sum += (error + self.error_prev) / 2 * dt

        self.error_prev = error
        self.time_prev = time_curr

        return effort
