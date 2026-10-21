"""Validation tests for the P0 plant model. Run: python -m pytest -q"""

import math
import random

import numpy as np
import pytest

from dfig_hil.params import DEFAULT as P
from dfig_hil.dfig import DfigModel, EULER, TRAPEZOIDAL, FAULT_NONE, FAULT_CLAMP
from dfig_hil.dfig_full import DfigFullModel
from dfig_hil.plant import Plant
from dfig_hil.encoder import Encoder
from dfig_hil.converter import duties_to_vr_dq, vr_dq_to_duties
from dfig_hil.control_ref import RefCurrentController
from dfig_hil.turbine import cp
from dfig_hil import reference as ref

D = P.derived()
VS, WS = D["v_s_pk_rated"], D["w_s"]
S_BASE, T_BASE = D["s_base"], D["t_rated"]
SLIPS = [-0.3, -0.2, -0.1, -0.05, 0.05, 0.1, 0.2, 0.3]


def w_m_for(s):
    return WS * (1.0 - s) / P.pole_pairs


def run_fixed(model, v_r, s, seconds):
    for _ in range(int(seconds / P.dt)):
        model.step(v_r.real, v_r.imag, VS, WS, w_m_for(s))
    return complex(model.i_rd, model.i_rq)


# ---------------------------------------------------------------- parameters

def test_derived_matches_prd_numbers():
    assert D["z_base"] == pytest.approx(171, abs=0.5)                 # §5.3
    assert D["tau_rotor"] == pytest.approx(4e-3, rel=0.05)            # §5.5 "≈ 4 ms"
    assert D["tau_rotor"] / P.dt == pytest.approx(40, rel=0.05)       # "40 steps"


# --------------------------------------------------- integrator vs equilibrium

@pytest.mark.parametrize("method", [EULER, TRAPEZOIDAL])
@pytest.mark.parametrize("s", SLIPS)
def test_prd_model_settles_to_its_equilibrium(method, s):
    target = complex(0.3, 1.6)
    v_r = ref.vr_for_ir_reduced(P, VS, WS, s, target)
    i_r = run_fixed(DfigModel(P, method=method), v_r, s, 0.1)
    assert abs(i_r - target) < 1e-3 * D["i_rated_pk"]


@pytest.mark.parametrize("method", [EULER, TRAPEZOIDAL])
@pytest.mark.parametrize("s", SLIPS)
def test_rs_model_settles_to_full_machine_steady_state(method, s):
    v_r = ref.vr_for_ir_reduced(P, VS, WS, s, complex(0.3, 1.6))
    _, i_r_full = ref.steady_full_from_vr(P, VS, WS, s, v_r)
    i_r = run_fixed(DfigModel(P, method=method, stator_rs=True), v_r, s, 0.1)
    assert abs(i_r - i_r_full) < 1e-3 * D["i_rated_pk"]


# ----------------------------------------------------------- A3 cross-check

def a3_errors(stator_rs):
    """Worst steady-state error (% of rated) of P_s, Q_s, T_e against the full
    machine, same rotor voltage applied, s in [-0.3, 0.3], three operating
    points (generating, motoring, with and without magnetising i_rd)."""
    worst = [0.0, 0.0, 0.0]
    for s in SLIPS:
        for target in (complex(0.0, 1.6), complex(0.8, 1.6), complex(0.5, -1.6)):
            v_r = ref.vr_for_ir_reduced(P, VS, WS, s, target)
            m = DfigModel(P, stator_rs=stator_rs)
            run_fixed(m, v_r, s, 0.1)
            i_s, i_r = ref.steady_full_from_vr(P, VS, WS, s, v_r)
            f = ref.outputs_full(P, VS, WS, i_s, i_r)
            e = [abs(m.p_s - f[0]) / S_BASE, abs(m.q_s - f[1]) / S_BASE, abs(m.t_e - f[2]) / T_BASE]
            worst = [max(a, 100 * b) for a, b in zip(worst, e)]
    return worst


def test_a3_rs_model_within_3_percent():
    assert max(a3_errors(stator_rs=True)) < 3.0


def test_a3_prd_model_error_is_known_and_large():
    # Documents README Q2: the §5.2 equations as written neglect R_s, which
    # with R_s = 10 ohm costs far more than A3's 3 % when the rotor voltage
    # (not current) is the input, as it is in HIL.
    assert max(a3_errors(stator_rs=False)) > 3.0


def test_prd_outputs_vs_full_at_same_rotor_current():
    # Closed-loop view: if a current controller holds the same i_r, the
    # remaining error is from the stator equation alone (~4-5 % of rated).
    for i_r in (complex(0, 1.6), complex(0, -1.6), complex(0.8, 1.6)):
        r = ref.outputs_reduced(P, VS, WS, i_r)
        f = ref.outputs_full(P, VS, WS, ref.steady_full_from_ir(P, VS, WS, i_r), i_r)
        assert abs(r[0] - f[0]) / S_BASE < 0.06
        assert abs(r[2] - f[2]) / T_BASE < 0.06


# ------------------------------------------------------- physics sanity checks

@pytest.mark.parametrize("s", [0.05, 0.1, 0.2, 0.4])
def test_open_circuit_rotor_emf_a2(s):
    m = DfigModel(P)
    m.step(0.0, 0.0, VS, WS, w_m_for(s))
    v_r_actual = m.v_r_emf * P.turns_ratio
    assert v_r_actual == pytest.approx(s * P.turns_ratio * VS, rel=0.05)   # A2 ±5 %


@pytest.mark.parametrize("stator_rs", [False, True])
@pytest.mark.parametrize("s", SLIPS)
def test_power_balance(stator_rs, s):
    """Electrical in = mechanical out + copper losses (motor convention)."""
    v_r = ref.vr_for_ir_reduced(P, VS, WS, s, complex(0.4, 1.5))
    m = DfigModel(P, stator_rs=stator_rs)
    i_r = run_fixed(m, v_r, s, 0.1)
    p_mech = m.t_e * w_m_for(s)
    p_cu = 1.5 * P.r_r * abs(i_r) ** 2
    if stator_rs:
        p_cu += 1.5 * P.r_s * (m.i_sd ** 2 + m.i_sq ** 2)
    assert m.p_s + m.p_r == pytest.approx(p_mech + p_cu, abs=0.005 * S_BASE)


def test_p_r_reverses_at_synchronous_speed():
    """Exercise 6: fixed generating i_rq, sweep speed through 1500 rpm."""
    signs = []
    for rpm in (1350, 1650):
        s = 1.0 - rpm / 1500.0
        v_r = ref.vr_for_ir_reduced(P, VS, WS, s, complex(0.0, 1.6))
        m = DfigModel(P)
        run_fixed(m, v_r, s, 0.1)
        assert m.p_s < 0                          # stator generates either way
        signs.append(math.copysign(1, m.p_r_approx))
    assert signs == [1, -1]   # sub-sync rotor absorbs, super-sync rotor delivers


# --------------------------------------------------------- closed-loop dynamics

def step_response(model_factory, s=-0.1, i_ref=1.6, seconds=0.03):
    m = model_factory()
    c = RefCurrentController(P)
    w_m = w_m_for(s)
    iq = []
    for _ in range(int(seconds / P.dt)):
        vd, vq = c.step(0.0, i_ref, m.i_rd, m.i_rq, s, VS, WS)
        m.step(vd, vq, VS, WS, w_m)
        iq.append(m.i_rq)
    return np.array(iq)


def settling_time(y, target, band=0.02):
    outside = np.nonzero(np.abs(y - target) > band * abs(target))[0]
    return (outside[-1] + 1) * P.dt if len(outside) else 0.0


def test_reference_controller_meets_a4_on_prd_model():
    y = step_response(lambda: DfigModel(P))
    assert settling_time(y, 1.6) < 5e-3
    assert y.max() < 1.6 * 1.10


def full_model_factory(s=-0.1):
    """RK4 reference machine, started in the i_r = 0 steady state."""
    def make():
        f = ref.FullDfig(P)
        v0 = ref.vr_for_ir_reduced(P, VS, WS, s, 0j)
        f.init_steady(VS, WS, w_m_for(s), v0.real, v0.imag)
        return f
    return make


def full_cla_factory(s=-0.1):
    """The target-form full model, started in the same state."""
    def make():
        m = DfigFullModel(P)
        m.psd, m.psq, m.prd, m.prq = full_model_factory(s)().x
        m.started = True
        return m
    return make


def closed_loop(model_factory, s, bw, seconds, i_ref=1.6):
    m = model_factory()
    c = RefCurrentController(P, bandwidth_hz=bw)
    y = []
    for _ in range(int(seconds / P.dt)):
        vd, vq = c.step(0.0, i_ref, m.i_rd, m.i_rq, s, VS, WS)
        m.step(vd, vq, VS, WS, w_m_for(s))
        y.append(m.i_rq)
    return np.array(y)


@pytest.mark.parametrize("s", [-0.2, 0.2])
def test_full_cla_model_tracks_rk4_reference(s):
    """Trapezoidal at 100 us (target form) vs RK4 at 10 us, closed loop.
    The stator mode is lightly damped at s = -0.2, so small per-step
    differences add up; 2 % of rated over 150 ms."""
    a = closed_loop(full_cla_factory(s), s, 400.0, 0.15)
    b = closed_loop(full_model_factory(s), s, 400.0, 0.15)
    assert np.abs(a - b).max() < 0.02 * D["i_rated_pk"]


@pytest.mark.parametrize("s", SLIPS)
def test_full_cla_model_steady_state(s):
    v_r = ref.vr_for_ir_reduced(P, VS, WS, s, complex(0.3, 1.6))
    m = DfigFullModel(P)
    for _ in range(int(0.6 / P.dt)):          # stator mode needs ~10 L_s/R_s
        m.step(v_r.real, v_r.imag, VS, WS, w_m_for(s))
    i_s, i_r = ref.steady_full_from_vr(P, VS, WS, s, v_r)
    f = ref.outputs_full(P, VS, WS, i_s, i_r)
    assert abs(m.p_s - f[0]) < 0.005 * S_BASE
    assert abs(m.q_s - f[1]) < 0.005 * S_BASE
    assert abs(m.t_e - f[2]) < 0.005 * T_BASE


def test_prd_model_hides_stator_flux_mode():
    """README Q3. With 200 Hz current-loop gains at s = -0.1 the §5.2 model
    settles in about 2 ms, while the full machine is still ringing at grid
    frequency 0.2 s later. Gains tuned on the §5.2 model can be unstable on
    the real machine; at 400 Hz the full machine's ringing decays."""
    s = -0.1
    prd = closed_loop(lambda: DfigModel(P), s, 200.0, 0.3)
    full = closed_loop(full_cla_factory(s), s, 200.0, 0.3)
    tail = full[-1000:]
    assert settling_time(prd, 1.6) < 5e-3
    assert tail.max() - tail.min() > 0.1 * 1.6
    full400 = closed_loop(full_cla_factory(s), s, 400.0, 0.3)
    tail = full400[-1000:]
    assert tail.max() - tail.min() < 0.05 * 1.6


# ------------------------------------------------------------ interface pieces

def test_duty_round_trip():
    rnd = random.Random(1)
    for _ in range(200):
        vd, vq = rnd.uniform(-80, 80), rnd.uniform(-80, 80)
        th = rnd.uniform(0, 2 * math.pi)
        duty = vr_dq_to_duties(vd, vq, P.v_dc, th, P.turns_ratio)
        assert all(0.0 <= d <= 1.0 for d in duty)
        d2, q2 = duties_to_vr_dq(duty, P.v_dc, th, P.turns_ratio)
        assert (d2, q2) == pytest.approx((vd, vq), abs=1e-9)


@pytest.mark.parametrize("rpm", [37.0, 900.0, 1234.5, 1500.0, 1800.0, 1999.0, -600.0])
def test_encoder_speed_within_1_rpm_a1(rpm):
    enc = Encoder(P)
    w = rpm * 2 * math.pi / 60
    theta = 0.0
    for _ in range(3 * int(P.speed_window)):
        theta = (theta + w * P.dt) % (2 * math.pi)
        enc.step(theta)
    assert enc.rpm_est == pytest.approx(rpm, abs=1.0)
    assert 0 <= enc.count < 4096


def test_closed_loop_through_duties_and_rotor_angle():
    """Full HIL path: controller -> duty[3] -> converter -> model, with the
    rotor angle from grid angle and shaft angle, at a rotating speed."""
    plant = Plant(P, rpm0=1650.0)
    c = RefCurrentController(P)
    for _ in range(int(0.05 / P.dt)):
        m = plant.dfig
        vd, vq = c.step(0.5, 1.6, m.i_rd, m.i_rq, m.slip or -0.1, VS, WS)
        duty = vr_dq_to_duties(vd, vq, P.v_dc, plant.theta_r, P.turns_ratio)
        plant.step(duty=duty)
    assert (plant.dfig.i_rd, plant.dfig.i_rq) == pytest.approx((0.5, 1.6), abs=0.02)
    # rotor CT currents are slip-frequency sinusoids, amplitude |i_r| / turns ratio
    amp = math.hypot(0.5, 1.6) / P.turns_ratio
    assert max(abs(x) for x in plant.fb.i_r_abc) <= amp * 1.02


def test_guard_latches_and_reset_clears():
    m = DfigModel(P)
    for _ in range(200):
        m.step(400.0, 0.0, VS, WS, w_m_for(0.0))
    assert m.fault == FAULT_CLAMP
    for _ in range(10):
        m.step(0.0, 0.0, VS, WS, w_m_for(0.0))
    assert m.fault == FAULT_CLAMP and m.i_rd == 0.0      # latched, converter off
    m.reset()
    assert m.fault == FAULT_NONE


def test_single_precision_matches_double():
    """The CLA has a single-precision FPU only. Run the same Euler model on
    float32 scalars and compare after 0.2 s at s = -0.2."""
    s = -0.2
    v_r = ref.vr_for_ir_reduced(P, VS, WS, s, complex(0.3, 1.6))
    m64 = DfigModel(P)
    m32 = DfigModel(P)
    f32 = np.float32
    for k in ("dt", "r_r", "l_m", "l_s", "sl_r", "inv_sl_r", "lm_ls", "pp", "i_guard"):
        setattr(m32, k, f32(getattr(m32, k)))
    m32.i_rd = m32.i_rq = f32(0)
    a = (f32(v_r.real), f32(v_r.imag), f32(VS), f32(WS), f32(w_m_for(s)))
    for _ in range(2000):
        m64.step(v_r.real, v_r.imag, VS, WS, w_m_for(s))
        m32.step(*a)
    assert isinstance(m32.i_rq, np.float32)
    assert abs(m32.i_rq - m64.i_rq) < 1e-4 * D["i_rated_pk"]
    assert abs(m32.p_s - m64.p_s) < 1e-4 * S_BASE


def test_cp_curve_peak():
    lams = np.linspace(2, 14, 1201)
    c = np.array([cp(l, 0.0) for l in lams])
    assert c.max() == pytest.approx(0.48, abs=0.01)
    assert lams[c.argmax()] == pytest.approx(8.1, abs=0.2)


def test_grid_sag_depth_and_timing():
    plant = Plant(P, rpm0=1500.0)
    plant.grid.add_sag(0.01, 0.15, 0.30)
    v = []
    for _ in range(int(0.2 / P.dt)):
        plant.step()
        v.append(plant.grid.v_s)
    v = np.array(v)
    assert v[50] == pytest.approx(VS)
    assert v[1000] == pytest.approx(0.7 * VS)
    assert v[1700] == pytest.approx(VS)
