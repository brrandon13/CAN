from geopy import distance
from collections import deque
from math import sqrt
import numpy as np
class LongitudinalController:
    def __init__(self, destination = None, max_speed = 8, max_accel = 1, K_P=1.0, K_I=0.5, K_D=0.1, dt = 0.02): # speed : 30km/h -> 8.3m/s accel : 1m/s^2
        '''
        max_speed = 8 m/s
        max_accel = 1 m/s^2

        '''
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._dt = dt
        self._error_buffer = deque(maxlen = 10)

        self.max_speed = max_speed
        self.max_accel = max_accel
        self.destination = destination

    def target_speed(self, distance):
        if distance < self.max_speed**2/(2*self.max_accel):
            ret = sqrt(2*self.max_accel*distance)
        else:
            ret = self.max_speed
        return ret


    def run_step(self, cur_speed, cur_position):
        '''
        a = Kp(v_t - v_c) + Ki Int(v_t - v_c) + Kd d/dt(v_t-v_c)
        
        '''
        if self.destination:
            dist = distance.distance(self.destination, cur_position) * 1000  # km to m
            target_speed = self.target_speed(dist)

            error = target_speed - cur_speed
            self._error_buffer.append(error)

            if len(self._error_buffer) >= 2:
                _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._k_d
                _ie = sum(self._error_buffer) * self._dt
            else:
                _de = 0.0
                _ie = 0.0

            return np.clip(self._k_p*error+self._k_d*_de+self._k_i*_ie, -self.max_accel, self.max_accel)

    def set_pid_parameters(self, kp, ki, kd):
        self._k_p = kp
        self._k_i = ki
        self._k_d = kd