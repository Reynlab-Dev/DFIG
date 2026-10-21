"""Incremental encoder + eQEP model and the speed estimate the control side sees.

E2 is a 1024 PPR quadrature encoder; the eQEP counts all four edges, so the
position counter runs 0 .. 4*PPR-1 and wraps (QPOSMAX = 4095). This module
produces exactly that integer each frame, then estimates speed with the
M-method over a sliding window of frames (a ring buffer of counts, which
ports directly to the CLA or C28x).

Resolution of the M-method is 60 / (counts_per_rev * window * dt) rpm:
with 4096 counts and a 20 ms window that is 0.73 rpm, inside A1's
+/-1 rpm. A 1-frame window would be 146 rpm per count, which is why a
per-frame difference is not usable as a speed signal.
"""

import math


class Encoder:
    def __init__(self, p):
        self.p = p
        self.cpr = int(4 * p.encoder_ppr)
        self.n = int(p.speed_window)
        self.ring = [0] * self.n
        self.idx = 0
        self.filled = 0
        self.count = 0
        self.pos = 0          # multi-turn count
        self.w_est = 0.0

    def step(self, theta_m):
        """theta_m: true mechanical angle in [0, 2 pi). Returns (count, w_est rad/s)."""
        count = int(theta_m * self.cpr / (2.0 * math.pi)) % self.cpr
        # Per-frame unwrap into a multi-turn position (int32 on the target).
        # Valid while the shaft moves < half a revolution per frame,
        # i.e. below 300 000 rpm at 10 kHz.
        d = count - self.count
        half = self.cpr // 2
        if d > half:
            d -= self.cpr
        elif d < -half:
            d += self.cpr
        self.count = count
        self.pos += d

        oldest = self.ring[self.idx]
        self.ring[self.idx] = self.pos
        self.idx = (self.idx + 1) % self.n
        if self.filled < self.n:
            self.filled += 1
            return self.count, self.w_est
        self.w_est = (self.pos - oldest) * 2.0 * math.pi / (self.cpr * self.n * self.p.dt)
        return self.count, self.w_est

    def theta_m(self):
        """Mechanical angle as the controller reconstructs it from the count."""
        return self.count * 2.0 * math.pi / self.cpr

    @property
    def rpm_est(self):
        return self.w_est * 60.0 / (2.0 * math.pi)
