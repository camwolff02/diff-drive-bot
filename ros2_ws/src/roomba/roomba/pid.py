#!/usr/bin/env python3
import time

class PidController:
    def __init__(self, Kp: float, Ki: float, Kd: float):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self._i_error = 0
        self._previous_error = 0
        self._previous_time = 0

    def next(self, error: float) -> float:
        # calculate delta t
        now = time.time()
        dt = now - self._previous_time 
        self._previous_time = now

        # calculate derivative and integral of error
        d_error = (error - self._previous_error) / dt
        self._i_error += error
        self._previous_error = error

        # calculate and return control
        return self.Kp*error + self.Ki*self._i_error + self.Kd*d_error