"""Full 4th-order DFIG model in a CLA-portable form (not in the PRD; README Q3).

Same machine as reference.FullDfig (stator and rotor flux dynamics, R_s
included), but written for the target: two complex states held as four
floats, the trapezoidal rule at the 100 us frame, and an explicit 2x2
complex solve, so there is no inner loop and no matrix library.

    x = [psi_s, psi_r]                      (complex, referred)
    x' = M x + u,  u = [v_s, v_r]
    M  = [[-R_s L_r/D - j w_s,   R_s L_m/D        ],
          [ R_r L_m/D,          -R_r L_s/D - j w_sl]],   D = L_s L_r - L_m^2
    (I - h/2 M) x1 = (I + h/2 M) x0 + h u

Why it matters: the §5.2 model has no stator-flux mode. With the rotor
current held by a controller that mode rings at grid frequency with a
time constant of about L_s/R_s = 62 ms, which is what a grid sag
(Exercise 9) excites. The outputs and step() signature match DfigModel,
so Plant can run either.
"""

import math

from .dfig import FAULT_NONE, FAULT_CLAMP, FAULT_NAN


class DfigFullModel:
    def __init__(self, p):
        d = p.derived()
        self.p = p
        self.h = p.dt
        self.r_s, self.r_r = p.r_s, p.r_r
        self.l_s, self.l_r, self.l_m = d["l_s"], d["l_r"], p.l_m
        self.inv_det = 1.0 / (self.l_s * self.l_r - self.l_m ** 2)
        self.pp = p.pole_pairs
        self.i_guard = d["i_guard"]
        # constant real parts of M
        self.m11r = -self.r_s * self.l_r * self.inv_det
        self.m12 = self.r_s * self.l_m * self.inv_det
        self.m21 = self.r_r * self.l_m * self.inv_det
        self.m22r = -self.r_r * self.l_s * self.inv_det
        self.reset()

    def reset(self):
        self.psd = self.psq = self.prd = self.prq = 0.0
        self.fault = FAULT_NONE
        self.started = False
        self.i_sd = self.i_sq = self.i_rd = self.i_rq = 0.0
        self.slip = self.psi_s = 0.0
        self.p_s = self.q_s = self.t_e = 0.0
        self.p_r = self.p_r_approx = self.p_total = 0.0
        self.v_r_emf = 0.0

    def _start(self, v_s, w_s):
        # energise into the no-load steady state (rotor open / i_r = 0)
        i_s = complex(0.0, v_s) / complex(self.r_s, w_s * self.l_s)
        self.psd, self.psq = self.l_s * i_s.real, self.l_s * i_s.imag
        self.prd, self.prq = self.l_m * i_s.real, self.l_m * i_s.imag
        self.started = True

    def step(self, v_rd, v_rq, v_s, w_s, w_m):
        if not self.started:
            self._start(v_s, w_s)
        s = (w_s - w_m * self.pp) / w_s
        w_sl = s * w_s
        if self.fault:
            v_rd = v_rq = 0.0      # converter off; rotor flux decays through R_r
        hh = 0.5 * self.h

        # A = I - h/2 M,  B = I + h/2 M  (complex 2x2, entries as (re, im))
        a11r, a11i = 1.0 - hh * self.m11r, hh * w_s
        a12 = -hh * self.m12
        a21 = -hh * self.m21
        a22r, a22i = 1.0 - hh * self.m22r, hh * w_sl
        b11r, b11i = 1.0 + hh * self.m11r, -hh * w_s
        b22r, b22i = 1.0 + hh * self.m22r, -hh * w_sl

        # rhs = B x0 + h u   (u_s = j v_s, since v_s lies on the q axis)
        r1r = b11r * self.psd - b11i * self.psq + hh * self.m12 * self.prd
        r1i = b11r * self.psq + b11i * self.psd + hh * self.m12 * self.prq + self.h * v_s
        r2r = b22r * self.prd - b22i * self.prq + hh * self.m21 * self.psd + self.h * v_rd
        r2i = b22r * self.prq + b22i * self.prd + hh * self.m21 * self.psq + self.h * v_rq

        # solve A x1 = rhs by Cramer: det = a11 a22 - a12 a21 (a12, a21 real)
        dr = a11r * a22r - a11i * a22i - a12 * a21
        di = a11r * a22i + a11i * a22r
        k = 1.0 / (dr * dr + di * di)
        # x1_s = (rhs1 a22 - a12 rhs2) / det
        nr = r1r * a22r - r1i * a22i - a12 * r2r
        ni = r1r * a22i + r1i * a22r - a12 * r2i
        psd, psq = (nr * dr + ni * di) * k, (ni * dr - nr * di) * k
        # x1_r = (a11 rhs2 - a21 rhs1) / det
        nr = a11r * r2r - a11i * r2i - a21 * r1r
        ni = a11r * r2i + a11i * r2r - a21 * r1i
        prd, prq = (nr * dr + ni * di) * k, (ni * dr - nr * di) * k

        ird = (self.l_s * prd - self.l_m * psd) * self.inv_det
        irq = (self.l_s * prq - self.l_m * psq) * self.inv_det
        if not all(math.isfinite(x) for x in (psd, psq, prd, prq)):
            self.fault = FAULT_NAN
            self._start(v_s, w_s)
            psd, psq, prd, prq = self.psd, self.psq, self.prd, self.prq
        elif abs(ird) > self.i_guard or abs(irq) > self.i_guard:
            self.fault = FAULT_CLAMP
        self.psd, self.psq, self.prd, self.prq = psd, psq, prd, prq

        k = self.inv_det
        self.i_sd = (self.l_r * psd - self.l_m * prd) * k
        self.i_sq = (self.l_r * psq - self.l_m * prq) * k
        self.i_rd = (self.l_s * prd - self.l_m * psd) * k
        self.i_rq = (self.l_s * prq - self.l_m * psq) * k
        self.slip = s
        self.psi_s = math.hypot(psd, psq)
        self.p_s = 1.5 * v_s * self.i_sq
        self.q_s = 1.5 * v_s * self.i_sd
        self.t_e = 1.5 * self.pp * (psd * self.i_sq - psq * self.i_sd)
        self.p_r = 1.5 * (v_rd * self.i_rd + v_rq * self.i_rq)
        self.p_r_approx = -s * self.p_s
        self.p_total = self.p_s + self.p_r_approx
        self.v_r_emf = abs(w_sl) * self.l_m / self.l_s * self.psi_s
        return self.i_rd, self.i_rq
