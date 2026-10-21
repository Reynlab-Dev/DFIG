"""Plant parameters (PRD-DFIG-HIL-001 §5.3, §4.1, §5.4).

This module is the Python twin of the future `dfig_params.h`. Every value
is a plain float so the set can be written out as a C header or a
commissioning CSV without translation. Derived quantities are computed in
`derived()` so the base table stays the one that Exercise 1 overwrites.

Conventions used everywhere in this package
-------------------------------------------
* Motor convention for the machine (as the PRD's §5.2 equations are
  written): P_s, Q_s > 0 means the stator ABSORBS power from the grid,
  T_e > 0 is motoring torque. Generating therefore shows P_s < 0, T_e < 0.
* dq quantities are amplitude-invariant (peak phase values), so
  P = 1.5 * (v_d i_d + v_q i_q). V_s in the §5.2 equations is the peak
  phase stator voltage, sqrt(2/3) * V_LL_rms.
* Rotor quantities inside the model are referred to the stator. Use
  `rotor_to_actual_*` helpers at the interface (slip-ring voltages,
  rotor CT currents) where the 1:2 turns ratio applies.
"""

from dataclasses import dataclass, asdict, replace
import math


@dataclass(frozen=True)
class DfigParams:
    # --- Simulation frame (§5.5) ---
    dt: float = 100e-6            # s, 10 kHz model frame

    # --- Machine nameplate (§4.1) ---
    v_ll_rated: float = 415.0     # V rms line-line
    i_rated: float = 1.4          # A rms
    f_rated: float = 50.0         # Hz
    n_rated_rpm: float = 1410.0   # rpm
    p_rated: float = 746.0        # W (1 HP)
    pole_pairs: float = 2.0       # 4-pole
    turns_ratio: float = 2.0      # N_r / N_s (stator:rotor 1:2)

    # --- Equivalent circuit, star, per phase (§5.3 placeholders) ---
    r_s: float = 10.0             # ohm
    r_r: float = 12.0             # ohm, referred to stator
    l_ls: float = 0.025           # H, see README open question Q1
    l_lr: float = 0.025           # H, referred
    l_m: float = 0.60             # H

    # --- Mechanics (§5.3) ---
    j: float = 0.02               # kg m^2, motor + generator + coupling
    b: float = 0.001              # N m s

    # --- Simulated rotor-side converter DC link (not in PRD, see Q4) ---
    v_dc: float = 500.0           # V, actual rotor-side bus

    # --- DC prime mover / 4-quadrant drive (§4.1, E9) ---
    t_pm_max: float = 10.0        # N m, drive torque limit (2 HP @ 1750 rpm ~ 8.1 N m)
    tau_drive: float = 0.02       # s, torque response of the drive
    kp_speed: float = 0.6         # N m / (rad/s), drive speed loop (speed mode)
    ki_speed: float = 6.0         # N m / rad

    # --- Encoder (E2) ---
    encoder_ppr: float = 1024.0   # lines/rev, quadrature x4 in eQEP
    speed_window: float = 200.0   # frames for M-method speed estimate (20 ms)

    # --- Stability guard (§5.5) ---
    guard_multiple: float = 3.0   # states clamped to +/- 3x rated

    # --- Wind turbine, scaled to 1 HP (§5.4; values are ours, see Q6) ---
    turbine_radius: float = 0.68  # m
    air_density: float = 1.225    # kg/m^3
    gear_ratio: float = 1.2       # generator speed / turbine speed
    turbine_dt_frames: float = 50.0   # turbine model at 200 Hz

    def derived(self):
        """Quantities computed from the base table. Returned as a dict of
        floats so a C port can precompute them the same way."""
        w_s = 2.0 * math.pi * self.f_rated
        l_s = self.l_ls + self.l_m
        l_r = self.l_lr + self.l_m
        sigma = 1.0 - self.l_m ** 2 / (l_s * l_r)
        v_s_pk = math.sqrt(2.0 / 3.0) * self.v_ll_rated
        i_rated_pk = math.sqrt(2.0) * self.i_rated
        w_m_rated = self.n_rated_rpm * 2.0 * math.pi / 60.0
        return {
            "w_s": w_s,
            "l_s": l_s,
            "l_r": l_r,
            "sigma": sigma,
            "sigma_l_r": sigma * l_r,
            "tau_rotor": sigma * l_r / self.r_r,
            "v_s_pk_rated": v_s_pk,
            "psi_s_rated": v_s_pk / w_s,
            "i_rated_pk": i_rated_pk,
            "i_guard": self.guard_multiple * i_rated_pk,
            "z_base": self.v_ll_rated / (math.sqrt(3.0) * self.i_rated),
            "s_base": math.sqrt(3.0) * self.v_ll_rated * self.i_rated,
            "t_rated": self.p_rated / w_m_rated,
            "w_m_sync": w_s / self.pole_pairs,
            "counts_per_rev": 4.0 * self.encoder_ppr,
            "v_dc_referred": self.v_dc / self.turns_ratio,
        }

    def with_(self, **kw):
        return replace(self, **kw)

    def as_dict(self):
        return asdict(self)


DEFAULT = DfigParams()


def rotor_to_actual_voltage(v_referred, p=DEFAULT):
    return v_referred * p.turns_ratio


def rotor_to_actual_current(i_referred, p=DEFAULT):
    return i_referred / p.turns_ratio


def write_c_header(p=DEFAULT, path=None):
    """Emit the parameter set as `dfig_params.h` text (float literals)."""
    lines = [
        "/* Generated from python/dfig_hil/params.py. Do not edit by hand;",
        " * Exercise 1 replaces these with the unit's measured values. */",
        "#ifndef DFIG_PARAMS_H",
        "#define DFIG_PARAMS_H",
        "",
    ]
    for k, v in p.as_dict().items():
        lines.append(f"#define DFIG_{k.upper():<20s} ({float(v):.9g}f)")
    lines.append("")
    for k, v in p.derived().items():
        lines.append(f"#define DFIG_D_{k.upper():<18s} ({float(v):.9g}f)")
    lines += ["", "#endif"]
    text = "\n".join(lines) + "\n"
    if path:
        with open(path, "w") as f:
            f.write(text)
    return text
