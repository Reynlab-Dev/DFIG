"""Host-side mirror of firmware/dfig_hil_fw/model/dfig_model_cla.cla.

Why this file exists: there is no CLA instruction-set simulator wired
into this pipeline yet (that needs the LaunchPad or CCS's simulator
target, see DFIG_HANDOFF.md next-steps). The risk in hand-porting
dfig_full.py's math into CLA C is a translation bug -- a dropped sign,
a swapped term, the wrong variable -- not CLA-specific instruction
semantics. This module is a line-for-line transcription of the .cla
file (same operation order, same intermediate names), with every
operation recast to float32 to approximate the target's single-
precision arithmetic, so a comparison against the float64
dfig_full.py reference exercises exactly that translation step.

This is NOT a substitute for running the actual compiled .cla task on
real hardware or a cycle-accurate simulator -- it cannot catch a CLA
compiler bug or a wrong linker/memory-section wiring. It catches "did
I port the algebra correctly," which is the more likely failure mode
and the one worth catching before anything touches the board.
"""

import numpy as np

f32 = np.float32

FAULT_NONE = 0
FAULT_CLAMP = 1


class ClaMirror:
    """Mirrors Cla1Task7 (init) and Cla1Task1 (per-frame) exactly."""

    def __init__(self, p):
        d = p.derived()
        self.l_s = f32(d["l_s"])
        self.l_r = f32(d["l_r"])
        self.l_m = f32(p.l_m)
        self.h = f32(p.dt)
        self.pp = f32(p.pole_pairs)
        self.r_s = f32(p.r_s)
        self.r_r = f32(p.r_r)
        self.i_guard = f32(d["i_guard"])
        self.cla1task7()

    def cla1task7(self):
        l_s, l_r, l_m, r_s, r_r = self.l_s, self.l_r, self.l_m, self.r_s, self.r_r
        inv_det = f32(1.0) / (l_s * l_r - l_m * l_m)

        self.m11r = -r_s * l_r * inv_det
        self.m12 = r_s * l_m * inv_det
        self.m21 = r_r * l_m * inv_det
        self.m22r = -r_r * l_s * inv_det
        self.inv_det_lm = inv_det

        self.psd = self.psq = self.prd = self.prq = f32(0.0)
        self.cla_started = 0
        self.cla_out_fault = FAULT_NONE

    def cla1task1(self, v_rd, v_rq, v_s, w_s, w_m):
        v_rd, v_rq, v_s, w_s, w_m = f32(v_rd), f32(v_rq), f32(v_s), f32(w_s), f32(w_m)
        l_s, l_r, l_m, h, pp = self.l_s, self.l_r, self.l_m, self.h, self.pp
        hh = f32(0.5) * h

        if self.cla_started == 0:
            r_s = self.r_s
            denom = r_s * r_s + (w_s * l_s) * (w_s * l_s)
            i_s_re = w_s * l_s * v_s / denom
            i_s_im = r_s * v_s / denom
            self.psd = l_s * i_s_re
            self.psq = l_s * i_s_im
            self.prd = l_m * i_s_re
            self.prq = l_m * i_s_im
            self.cla_started = 1

        psd, psq, prd, prq = self.psd, self.psq, self.prd, self.prq

        s = (w_s - w_m * pp) / w_s
        w_sl = s * w_s

        m11r, m12, m21, m22r = self.m11r, self.m12, self.m21, self.m22r

        a11r, a11i = f32(1.0) - hh * m11r, hh * w_s
        a12 = -hh * m12
        a21 = -hh * m21
        a22r, a22i = f32(1.0) - hh * m22r, hh * w_sl
        b11r, b11i = f32(1.0) + hh * m11r, -hh * w_s
        b22r, b22i = f32(1.0) + hh * m22r, -hh * w_sl

        r1r = b11r * psd - b11i * psq + hh * m12 * prd
        r1i = b11r * psq + b11i * psd + hh * m12 * prq + h * v_s
        r2r = b22r * prd - b22i * prq + hh * m21 * psd + h * v_rd
        r2i = b22r * prq + b22i * prd + hh * m21 * psq + h * v_rq

        dr = a11r * a22r - a11i * a22i - a12 * a21
        di = a11r * a22i + a11i * a22r
        k = f32(1.0) / (dr * dr + di * di)

        nr = r1r * a22r - r1i * a22i - a12 * r2r
        ni = r1r * a22i + r1i * a22r - a12 * r2i
        new_psd = (nr * dr + ni * di) * k
        new_psq = (ni * dr - nr * di) * k

        nr = a11r * r2r - a11i * r2i - a21 * r1r
        ni = a11r * r2i + a11i * r2r - a21 * r1i
        new_prd = (nr * dr + ni * di) * k
        new_prq = (ni * dr - nr * di) * k

        i_rd = (l_s * new_prd - l_m * new_psd) * self.inv_det_lm
        i_rq = (l_s * new_prq - l_m * new_psq) * self.inv_det_lm

        if (i_rd > self.i_guard or i_rd < -self.i_guard or
                i_rq > self.i_guard or i_rq < -self.i_guard):
            self.cla_out_fault = FAULT_CLAMP
        else:
            self.cla_out_fault = FAULT_NONE

        self.psd, self.psq, self.prd, self.prq = new_psd, new_psq, new_prd, new_prq

        k2 = self.inv_det_lm
        i_sd = (l_r * new_psd - l_m * new_prd) * k2
        i_sq = (l_r * new_psq - l_m * new_prq) * k2
        i_rd = (l_s * new_prd - l_m * new_psd) * k2
        i_rq = (l_s * new_prq - l_m * new_psq) * k2

        self.cla_out_i_rd = i_rd
        self.cla_out_i_rq = i_rq
        self.cla_out_p_s = f32(1.5) * v_s * i_sq
        self.cla_out_q_s = f32(1.5) * v_s * i_sd
        self.cla_out_t_e = f32(1.5) * pp * (new_psd * i_sq - new_psq * i_sd)

        return float(i_rd), float(i_rq)
