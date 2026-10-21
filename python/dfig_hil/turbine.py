"""Wind turbine model (PRD §5.4), runs at 200 Hz and produces the DC drive
torque reference.

    lambda   = w_turbine R / v_wind
    1/lam_i  = 1/(lambda + 0.08 beta) - 0.035/(beta^3 + 1)
    Cp       = 0.5176 (116/lam_i - 0.4 beta - 5) e^(-21/lam_i) + 0.0068 lambda
    P_w      = 0.5 rho A v^3 Cp
    tau_ref  = P_w / w_turbine, referred to the generator shaft by the gear ratio

The lambda_i expression is the standard one that goes with these Cp
coefficients; the PRD uses lambda_i without defining it (README Q6).
"""

import math


def cp(lam, beta):
    if lam <= 0.0:
        return 0.0
    inv_li = 1.0 / (lam + 0.08 * beta) - 0.035 / (beta ** 3 + 1.0)
    if inv_li <= 0.0:
        return 0.0
    c = 0.5176 * (116.0 * inv_li - 0.4 * beta - 5.0) * math.exp(-21.0 * inv_li) + 0.0068 * lam
    return max(c, 0.0)


class Turbine:
    def __init__(self, p):
        self.p = p
        self.area = math.pi * p.turbine_radius ** 2
        self.v_wind = 0.0
        self.beta = 0.0
        self.lam = 0.0
        self.cp = 0.0
        self.p_w = 0.0
        self.t_gen = 0.0      # torque reference at the generator shaft, N m

    def step(self, w_m):
        """w_m: generator shaft speed rad/s. Returns torque reference (N m)."""
        p = self.p
        w_t = w_m / p.gear_ratio
        if self.v_wind <= 0.1 or w_t <= 1.0:
            self.lam = self.cp = self.p_w = 0.0
            # below cut-in / at standstill: a small torque that starts the rotor
            self.t_gen = 0.5 * p.air_density * self.area * self.v_wind ** 2 \
                * p.turbine_radius * 0.05 / p.gear_ratio
            return self.t_gen
        self.lam = w_t * p.turbine_radius / self.v_wind
        self.cp = cp(self.lam, self.beta)
        self.p_w = 0.5 * p.air_density * self.area * self.v_wind ** 3 * self.cp
        self.t_gen = self.p_w / w_m       # same power, at generator speed
        return self.t_gen
