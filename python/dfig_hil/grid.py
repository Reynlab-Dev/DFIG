"""Grid / stator supply model (PRD §5.2 inputs, §7.2 scenario screen, grid_events.c).

In the shipped HIL unit V_s and f_s are measured. On the simulation-only
LaunchPad build they come from this model instead. It produces the stator
voltage magnitude, frequency and angle once per frame, with sag, swell and
frequency-deviation events from a small fixed table (one table row is one
event, so the C port is an array of structs).
"""

import math

SAG = 0      # value = remaining fraction, e.g. 0.7 for a 30 % sag
FREQ = 1     # value = new frequency in Hz


class GridEvent:
    __slots__ = ("kind", "t_start", "duration", "value")

    def __init__(self, kind, t_start, duration, value):
        self.kind = kind
        self.t_start = t_start
        self.duration = duration
        self.value = value


class Grid:
    def __init__(self, p, v_ll=None, f=None):
        self.p = p
        self.v_ll_nom = p.v_ll_rated if v_ll is None else v_ll
        self.f_nom = p.f_rated if f is None else f
        self.events = []
        self.theta_v = 0.0        # angle of the stator voltage vector, rad
        self.t = 0.0
        self.v_s = math.sqrt(2.0 / 3.0) * self.v_ll_nom   # peak phase V
        self.w_s = 2.0 * math.pi * self.f_nom

    def add_sag(self, t_start, duration, depth):
        """depth 0.3 = 30 % sag (voltage falls to 70 %). Negative = swell."""
        self.events.append(GridEvent(SAG, t_start, duration, 1.0 - depth))

    def add_freq_step(self, t_start, duration, f_new):
        self.events.append(GridEvent(FREQ, t_start, duration, f_new))

    def step(self):
        """Advance one frame. Returns (v_s_peak_phase, w_s, theta_v)."""
        scale = 1.0
        f = self.f_nom
        for e in self.events:
            if e.t_start <= self.t < e.t_start + e.duration:
                if e.kind == SAG:
                    scale *= e.value
                else:
                    f = e.value
        self.v_s = scale * math.sqrt(2.0 / 3.0) * self.v_ll_nom
        self.w_s = 2.0 * math.pi * f
        self.theta_v = (self.theta_v + self.w_s * self.p.dt) % (2.0 * math.pi)
        self.t += self.p.dt
        return self.v_s, self.w_s, self.theta_v

    def v_abc(self):
        """Instantaneous phase voltages (what LV 25-P transducers would see)."""
        a = self.theta_v
        k = 2.0 * math.pi / 3.0
        return (self.v_s * math.cos(a),
                self.v_s * math.cos(a - k),
                self.v_s * math.cos(a + k))
