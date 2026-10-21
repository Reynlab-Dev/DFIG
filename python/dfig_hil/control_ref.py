"""Reference rotor current controller, for tests and validation plots only.

This is NOT the student's controller (that lives in /control on the C28x).
It exists so the plant can be exercised closed-loop in Python: two PI loops
with cross-coupling and back-EMF feed-forward, tuned by pole-zero
cancellation to a 200 Hz bandwidth, one frame of computation delay like
the real ISR (computed in frame k, applied in frame k+1).
"""

import math


class RefCurrentController:
    def __init__(self, p, bandwidth_hz=200.0, v_limit=None):
        d = p.derived()
        wc = 2.0 * math.pi * bandwidth_hz
        self.p = p
        self.sl = d["sigma_l_r"]
        self.k = p.l_m / d["l_s"]
        self.kp = self.sl * wc
        self.ki = p.r_r * wc
        self.dt = p.dt
        # SVPWM linear range, referred: V_dc / sqrt(3) / turns ratio
        self.v_lim = v_limit if v_limit is not None else d["v_dc_referred"] / math.sqrt(3.0)
        self.int_d = self.int_q = 0.0
        self.out = (0.0, 0.0)          # what gets applied this frame

    def step(self, i_rd_ref, i_rq_ref, i_rd, i_rq, slip, v_s, w_s):
        applied = self.out
        sws = slip * w_s
        ed, eq = i_rd_ref - i_rd, i_rq_ref - i_rq
        ff_d = -sws * self.sl * i_rq
        ff_q = sws * (self.sl * i_rd + self.k * v_s / w_s)
        vd = self.kp * ed + self.int_d + ff_d
        vq = self.kp * eq + self.int_q + ff_q
        mag = math.hypot(vd, vq)
        if mag < self.v_lim:            # anti-windup: integrate only when unsaturated
            self.int_d += self.ki * ed * self.dt
            self.int_q += self.ki * eq * self.dt
        else:
            vd, vq = vd * self.v_lim / mag, vq * self.v_lim / mag
        self.out = (vd, vq)
        return applied
