"""Shaft and DC prime mover (PRD §4.1 DC motor + E9 drive, §5.3 J and B).

    J * dw_m/dt = T_pm + T_e - B * w_m

T_e is the machine's electromagnetic torque in motor convention (§5.2), so
a generating DFIG (T_e < 0) brakes the shaft. T_pm is the DC motor torque.

The 4-quadrant DC drive is modelled as a first-order torque lag with a
torque limit. It runs in one of two modes, like the real drive's analog
reference input:
  * TORQUE: reference is a torque (turbine emulation, §5.4).
  * SPEED:  reference is a speed; an internal PI makes the torque
            (speed sweeps in Exercises 2 and 6).
"""

import math

TORQUE = 0
SPEED = 1


class Shaft:
    def __init__(self, p, rpm0=0.0):
        self.p = p
        self.w_m = rpm0 * 2.0 * math.pi / 60.0   # rad/s mechanical
        self.theta_m = 0.0                        # rad mechanical, wraps at 2 pi
        self.t_pm = 0.0                           # actual drive torque
        self.mode = SPEED
        self.ref = self.w_m
        self._integ = p.b * self.w_m              # start in equilibrium

    def set_speed_rpm(self, rpm):
        self.mode = SPEED
        self.ref = rpm * 2.0 * math.pi / 60.0

    def set_torque(self, t):
        self.mode = TORQUE
        self.ref = t

    def step(self, t_e):
        p = self.p
        dt = p.dt
        if self.mode == SPEED:
            err = self.ref - self.w_m
            t_cmd = p.kp_speed * err + self._integ
            # conditional integration = anti-windup
            if -p.t_pm_max < t_cmd < p.t_pm_max:
                self._integ += p.ki_speed * err * dt
        else:
            t_cmd = self.ref
        t_cmd = min(max(t_cmd, -p.t_pm_max), p.t_pm_max)
        self.t_pm += (t_cmd - self.t_pm) * dt / p.tau_drive

        dw = (self.t_pm + t_e - p.b * self.w_m) / p.j
        self.w_m += dw * dt
        self.theta_m = (self.theta_m + self.w_m * dt) % (2.0 * math.pi)
        return self.w_m, self.theta_m

    @property
    def rpm(self):
        return self.w_m * 60.0 / (2.0 * math.pi)
