"""One 100 us plant frame: grid, converter, DFIG, shaft, encoder, turbine.

On the simulation-only LaunchPad build the CLA owns all of this, and the
C28x control code sees only what a real controller would see:

  * measured side  -> encoder count, stator v_abc, stator i_abc
  * swap point     -> PlantFeedback, the Python mirror of plant_feedback_t (§6.5)

Frame order (fixed, so the C port reproduces it bit for bit):
  1. grid      -> V_s, w_s, theta_v
  2. converter -> duty[3] (from last control frame) to v_rd, v_rq
  3. DFIG      -> i_rd, i_rq, P_s, Q_s, T_e  (one of three models)
  4. turbine   -> drive torque reference (every 50th frame, 200 Hz)
  5. shaft     -> w_m, theta_m (uses this frame's T_e)
  6. encoder   -> count, speed estimate
"""

import math

from .converter import duties_to_vr_dq, inv_park, inv_clarke, rotor_angle, flux_angle
from .dfig import DfigModel, EULER
from .dfig_full import DfigFullModel
from .encoder import Encoder
from .grid import Grid
from .shaft import Shaft
from .turbine import Turbine

# src_flags bit positions, one per plant_feedback_t field (§6.5)
SRC_I_RA, SRC_I_RB, SRC_I_RC, SRC_V_DC, SRC_P_S, SRC_Q_S, SRC_T_E = range(7)
SRC_ALL_SIMULATED = (1 << 7) - 1


class PlantFeedback:
    __slots__ = ("i_r_abc", "v_dc", "P_s", "Q_s", "T_e", "src_flags")

    def __init__(self):
        self.i_r_abc = [0.0, 0.0, 0.0]
        self.v_dc = 0.0
        self.P_s = self.Q_s = self.T_e = 0.0
        self.src_flags = SRC_ALL_SIMULATED


class Plant:
    def __init__(self, p, rpm0=0.0, model="prd", method=EULER, v_ll=None):
        """model: "prd"    = §5.2 equations as written (default, the spec)
                  "prd_rs" = §5.2 with R_s kept in the stator equation
                  "full"   = 4th-order model with stator-flux dynamics"""
        self.p = p
        self.grid = Grid(p, v_ll=v_ll)
        if model == "full":
            self.dfig = DfigFullModel(p)
        else:
            self.dfig = DfigModel(p, method=method, stator_rs=(model == "prd_rs"))
        self.shaft = Shaft(p, rpm0=rpm0)
        self.encoder = Encoder(p)
        self.turbine = Turbine(p)
        self.wind_mode = False
        self.fb = PlantFeedback()
        self.frame = 0
        self.t = 0.0
        self.v_rd = self.v_rq = 0.0
        self.theta_r = 0.0
        self.i_s_abc = (0.0, 0.0, 0.0)

    def set_wind(self, v_wind, beta=0.0):
        self.wind_mode = True
        self.turbine.v_wind = v_wind
        self.turbine.beta = beta

    def step(self, duty=None, v_r_dq=None):
        """Advance one frame. Give either duty[3] (normal HIL path) or
        v_r_dq = (v_rd, v_rq) referred volts (Exercise 4 open-loop path).
        With neither, the rotor is driven with zero volts (shorted rotor)."""
        p = self.p
        v_s, w_s, theta_v = self.grid.step()
        theta_r = rotor_angle(theta_v, self.shaft.theta_m, p.pole_pairs)
        if duty is not None:
            v_rd, v_rq = duties_to_vr_dq(duty, p.v_dc, theta_r, p.turns_ratio)
        elif v_r_dq is not None:
            v_rd, v_rq = v_r_dq
        else:
            v_rd = v_rq = 0.0
        self.v_rd, self.v_rq, self.theta_r = v_rd, v_rq, theta_r

        m = self.dfig
        m.step(v_rd, v_rq, v_s, w_s, self.shaft.w_m)

        if self.wind_mode and self.frame % int(p.turbine_dt_frames) == 0:
            self.shaft.set_torque(self.turbine.step(self.shaft.w_m))
        self.shaft.step(m.t_e)
        self.encoder.step(self.shaft.theta_m)

        # plant_feedback_t: rotor currents in actual (slip-ring side) amps
        a, b = inv_park(m.i_rd, m.i_rq, theta_r)
        ia, ib, ic = inv_clarke(a, b)
        k = 1.0 / p.turns_ratio
        fb = self.fb
        fb.i_r_abc[0], fb.i_r_abc[1], fb.i_r_abc[2] = ia * k, ib * k, ic * k
        fb.v_dc = p.v_dc
        fb.P_s, fb.Q_s, fb.T_e = m.p_s, m.q_s, m.t_e

        # "measured" stator current, as the LA 25-NP would see it
        a, b = inv_park(m.i_sd, m.i_sq, flux_angle(theta_v))
        self.i_s_abc = inv_clarke(a, b)

        self.frame += 1
        self.t += p.dt
        return fb
