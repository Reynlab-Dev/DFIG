"""Independent reference for validating the reduced model (PRD §5.6, A3).

The PRD validates against a Simulink/PLECS model. Until that exists, this
file is the stand-in: the full 4th-order dq DFIG (stator AND rotor flux
dynamics, R_s included, no assumption that stator flux is constant),
integrated with RK4 at a 10x finer step, plus closed-form steady-state
solutions of the same equations. It is validation-only code; nothing in
here is ported to the target.

Frame: synchronous, d axis at theta_v - 90 deg, so v_s = (0, V_s), which is
the frame the reduced model assumes. Motor convention, referred rotor.
"""

import math


class FullDfig:
    """psi_s' = v_s - R_s i_s - j w_s psi_s
       psi_r' = v_r - R_r i_r - j (w_s - p w_m) psi_r"""

    def __init__(self, p, substeps=10):
        d = p.derived()
        self.p = p
        self.l_s, self.l_r, self.l_m = d["l_s"], d["l_r"], p.l_m
        self.det = self.l_s * self.l_r - self.l_m ** 2
        self.h = p.dt / substeps
        self.n = substeps
        self.x = [0.0, 0.0, 0.0, 0.0]   # psi_sd, psi_sq, psi_rd, psi_rq
        self._out(0.0)

    def init_steady(self, v_s, w_s, w_m, v_rd, v_rq):
        """Start from the exact steady state for these inputs."""
        s = (w_s - self.p.pole_pairs * w_m) / w_s
        i_s, i_r = steady_full_from_vr(self.p, v_s, w_s, s, complex(v_rd, v_rq))
        ps = self.l_s * i_s + self.l_m * i_r
        pr = self.l_m * i_s + self.l_r * i_r
        self.x = [ps.real, ps.imag, pr.real, pr.imag]
        self._out(v_s)

    def _currents(self, x):
        psd, psq, prd, prq = x
        k = 1.0 / self.det
        isd = (self.l_r * psd - self.l_m * prd) * k
        isq = (self.l_r * psq - self.l_m * prq) * k
        ird = (self.l_s * prd - self.l_m * psd) * k
        irq = (self.l_s * prq - self.l_m * psq) * k
        return isd, isq, ird, irq

    def _f(self, x, v_s, w_s, w_slip, v_rd, v_rq):
        isd, isq, ird, irq = self._currents(x)
        psd, psq, prd, prq = x
        p = self.p
        return [
            0.0 - p.r_s * isd + w_s * psq,
            v_s - p.r_s * isq - w_s * psd,
            v_rd - p.r_r * ird + w_slip * prq,
            v_rq - p.r_r * irq - w_slip * prd,
        ]

    def step(self, v_rd, v_rq, v_s, w_s, w_m):
        w_slip = w_s - self.p.pole_pairs * w_m
        h = self.h
        x = self.x
        for _ in range(self.n):
            k1 = self._f(x, v_s, w_s, w_slip, v_rd, v_rq)
            k2 = self._f([a + 0.5 * h * b for a, b in zip(x, k1)], v_s, w_s, w_slip, v_rd, v_rq)
            k3 = self._f([a + 0.5 * h * b for a, b in zip(x, k2)], v_s, w_s, w_slip, v_rd, v_rq)
            k4 = self._f([a + h * b for a, b in zip(x, k3)], v_s, w_s, w_slip, v_rd, v_rq)
            x = [a + h / 6.0 * (b1 + 2 * b2 + 2 * b3 + b4)
                 for a, b1, b2, b3, b4 in zip(x, k1, k2, k3, k4)]
        self.x = x
        self._out(v_s)
        return self.i_rd, self.i_rq

    def _out(self, v_s):
        isd, isq, ird, irq = self._currents(self.x)
        psd, psq = self.x[0], self.x[1]
        self.i_sd, self.i_sq, self.i_rd, self.i_rq = isd, isq, ird, irq
        self.p_s = 1.5 * v_s * isq
        self.q_s = 1.5 * v_s * isd
        self.t_e = 1.5 * self.p.pole_pairs * (psd * isq - psq * isd)


# ---------------- closed-form steady states (complex phasors in dq) ----------------

def outputs_full(p, v_s, w_s, i_s, i_r):
    vs = complex(0.0, v_s)
    psi_s = p.l_m * i_r + (p.l_ls + p.l_m) * i_s
    s_cplx = 1.5 * vs * i_s.conjugate()
    t_e = 1.5 * p.pole_pairs * (psi_s.real * i_s.imag - psi_s.imag * i_s.real)
    return s_cplx.real, s_cplx.imag, t_e


def steady_full_from_ir(p, v_s, w_s, i_r):
    """Stator current for a given rotor current, full stator equation with R_s."""
    l_s = p.l_ls + p.l_m
    vs = complex(0.0, v_s)
    return (vs - 1j * w_s * p.l_m * i_r) / (p.r_s + 1j * w_s * l_s)


def steady_full_from_vr(p, v_s, w_s, s, v_r):
    l_s, l_r = p.l_ls + p.l_m, p.l_lr + p.l_m
    vs = complex(0.0, v_s)
    a11, a12 = p.r_s + 1j * w_s * l_s, 1j * w_s * p.l_m
    a21, a22 = 1j * s * w_s * p.l_m, p.r_r + 1j * s * w_s * l_r
    det = a11 * a22 - a12 * a21
    i_s = (vs * a22 - a12 * v_r) / det
    i_r = (a11 * v_r - a21 * vs) / det
    return i_s, i_r


def outputs_reduced(p, v_s, w_s, i_r):
    """PRD §5.2 output equations for a given rotor current."""
    l_s = p.l_ls + p.l_m
    k = p.l_m / l_s
    psi = v_s / w_s
    p_s = -1.5 * k * v_s * i_r.imag
    q_s = 1.5 * v_s * (v_s / (w_s * l_s) - k * i_r.real)
    t_e = -1.5 * p.pole_pairs * k * psi * i_r.imag
    return p_s, q_s, t_e


def steady_reduced_from_vr(p, v_s, w_s, s, v_r):
    """Equilibrium of the two §5.2 state equations."""
    d = p.derived()
    sl, r, sw = d["sigma_l_r"], p.r_r, s * w_s
    emf = sw * p.l_m / d["l_s"] * v_s / w_s
    # [ r  -sw*sl ] [id]   [v_rd]
    # [ sw*sl  r  ] [iq] = [v_rq - emf]
    bd, bq = v_r.real, v_r.imag - emf
    det = r * r + (sw * sl) ** 2
    i_d = (r * bd + sw * sl * bq) / det
    i_q = (r * bq - sw * sl * bd) / det
    return complex(i_d, i_q)


def vr_for_ir_reduced(p, v_s, w_s, s, i_r):
    """Rotor voltage that holds a given rotor current in the reduced model."""
    d = p.derived()
    sl, sw = d["sigma_l_r"], s * w_s
    emf = sw * p.l_m / d["l_s"] * v_s / w_s
    v_rd = p.r_r * i_r.real - sw * sl * i_r.imag
    v_rq = p.r_r * i_r.imag + sw * sl * i_r.real + emf
    return complex(v_rd, v_rq)
