"""Reduced DFIG rotor-current model, exactly the PRD §5.2 equations.

Stator-flux oriented frame (psi_sd = psi_s, psi_sq = 0), stator flux taken
as V_s / w_s (R_s and stator-flux transients neglected, no saturation).
Two states, i_rd and i_rq (referred to the stator), integrated once per
100 us frame:

    di_rd/dt = ( v_rd - R_r i_rd + s w_s sL_r i_rq ) / sL_r
    di_rq/dt = ( v_rq - R_r i_rq - s w_s (sL_r i_rd + L_m psi_s / L_s) ) / sL_r

Option `stator_rs=True` (not in the PRD, see README Q2): keeps the stator
quasi-steady but includes R_s. psi_s is then solved each frame from
v_s = R_s i_s + j w_s psi_s with the present rotor current, giving a
complex psi_s (both d and q parts) in the same frame. Its steady state is
exactly the full machine's; with the PRD's R_s = 10 ohm the plain §5.2
model is about 4-5 % of rated off in P_s, Q_s, T_e, which fails A3.

This file is written to be ported line for line to `dfig_model.c` on the
CLA: scalar float arithmetic only, coefficients precomputed in __init__,
no allocation in step(), one 2x2 update per frame.
"""

import math

EULER = 0
TRAPEZOIDAL = 1

FAULT_NONE = 0
FAULT_CLAMP = 1
FAULT_NAN = 2


class DfigModel:
    def __init__(self, p, method=EULER, stator_rs=False):
        d = p.derived()
        self.p = p
        self.method = method
        self.stator_rs = stator_rs
        self.r_s = p.r_s
        self.dt = p.dt
        self.r_r = p.r_r
        self.l_m = p.l_m
        self.l_s = d["l_s"]
        self.sl_r = d["sigma_l_r"]
        self.inv_sl_r = 1.0 / d["sigma_l_r"]
        self.lm_ls = p.l_m / d["l_s"]
        self.pp = p.pole_pairs
        self.i_guard = d["i_guard"]

        self.i_rd = 0.0
        self.i_rq = 0.0
        self.fault = FAULT_NONE

        # outputs, refreshed every step
        self.slip = 0.0
        self.psi_s = 0.0
        self.psi_sd = self.psi_sq = 0.0
        self.i_sd = self.i_sq = 0.0
        self.p_s = self.q_s = self.t_e = 0.0
        self.p_r = self.p_r_approx = self.p_total = 0.0
        self.v_r_emf = 0.0   # open-circuit rotor EMF, referred peak phase

    def reset(self):
        """RESET is the only way out of a latched MODEL_FAULT (A6)."""
        self.i_rd = self.i_rq = 0.0
        self.fault = FAULT_NONE

    def step(self, v_rd, v_rq, v_s, w_s, w_m):
        """One frame. v_r* referred peak-phase dq volts, v_s peak phase stator
        volts, w_s grid rad/s, w_m mechanical rad/s."""
        s = (w_s - w_m * self.pp) / w_s
        sws = s * w_s
        psd, psq = self._stator_flux(self.i_rd, self.i_rq, v_s, w_s)
        # rotor back-EMF  j s w_s (L_m/L_s) psi_s, split into d and q
        emf_d = -sws * self.lm_ls * psq
        emf_q = sws * self.lm_ls * psd

        if self.fault:
            # converter disabled: rotor currents forced to zero
            self.i_rd = self.i_rq = 0.0
            v_rd = v_rq = 0.0
        else:
            h = self.dt
            a = self.r_r * self.inv_sl_r
            ud = (v_rd - emf_d) * self.inv_sl_r
            uq = (v_rq - emf_q) * self.inv_sl_r
            id0, iq0 = self.i_rd, self.i_rq
            if self.method == EULER:
                self.i_rd = id0 + h * (-a * id0 + sws * iq0 + ud)
                self.i_rq = iq0 + h * (-a * iq0 - sws * id0 + uq)
            else:
                # (I - h/2 A) x1 = (I + h/2 A) x0 + h u,  A = [[-a, w], [-w, -a]]
                # (the back-EMF input u is held over the frame)
                ha, hw = 0.5 * h * a, 0.5 * h * sws
                rd = (1.0 - ha) * id0 + hw * iq0 + h * ud
                rq = (1.0 - ha) * iq0 - hw * id0 + h * uq
                m = 1.0 + ha
                inv_det = 1.0 / (m * m + hw * hw)
                self.i_rd = (m * rd + hw * rq) * inv_det
                self.i_rq = (m * rq - hw * rd) * inv_det

            # stability guard (5.5): NaN/Inf or > 3x rated latches MODEL_FAULT
            if not (math.isfinite(self.i_rd) and math.isfinite(self.i_rq)):
                self.fault = FAULT_NAN
                self.i_rd = self.i_rq = 0.0
            elif abs(self.i_rd) > self.i_guard or abs(self.i_rq) > self.i_guard:
                self.fault = FAULT_CLAMP
                self.i_rd = min(max(self.i_rd, -self.i_guard), self.i_guard)
                self.i_rq = min(max(self.i_rq, -self.i_guard), self.i_guard)

        i_rd, i_rq = self.i_rd, self.i_rq
        psd, psq = self._stator_flux(i_rd, i_rq, v_s, w_s)
        self.slip = s
        self.psi_sd, self.psi_sq = psd, psq
        self.psi_s = math.hypot(psd, psq)
        if self.stator_rs:
            self.i_sd, self.i_sq = self._i_sd, self._i_sq
            self.p_s = 1.5 * v_s * self.i_sq
            self.q_s = 1.5 * v_s * self.i_sd
            self.t_e = 1.5 * self.pp * (psd * self.i_sq - psq * self.i_sd)
        else:
            # PRD §5.2 output equations, verbatim
            self.i_sd = (psd - self.l_m * i_rd) / self.l_s
            self.i_sq = -self.lm_ls * i_rq
            self.p_s = -1.5 * self.lm_ls * v_s * i_rq
            self.q_s = 1.5 * v_s * (v_s / (w_s * self.l_s) - self.lm_ls * i_rd)
            self.t_e = -1.5 * self.pp * self.lm_ls * psd * i_rq
        self.p_r_approx = -s * self.p_s
        self.p_r = 1.5 * (v_rd * i_rd + v_rq * i_rq)
        self.p_total = self.p_s + self.p_r_approx
        self.v_r_emf = abs(s * w_s) * self.lm_ls * self.psi_s
        return i_rd, i_rq

    def _stator_flux(self, i_rd, i_rq, v_s, w_s):
        """psi_s in the frame where v_s = (0, V_s)."""
        if not self.stator_rs:
            return v_s / w_s, 0.0
        # i_s = (j V_s - j w_s L_m i_r) / (R_s + j w_s L_s)
        nr = w_s * self.l_m * i_rq
        ni = v_s - w_s * self.l_m * i_rd
        dr, di = self.r_s, w_s * self.l_s
        k = 1.0 / (dr * dr + di * di)
        isd = (nr * dr + ni * di) * k
        isq = (ni * dr - nr * di) * k
        self._i_sd, self._i_sq = isd, isq
        return self.l_s * isd + self.l_m * i_rd, self.l_s * isq + self.l_m * i_rq
