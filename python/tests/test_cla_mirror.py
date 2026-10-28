"""Frame-by-frame comparison: the CLA port (cla_mirror.py, a float32
transcription of firmware/dfig_hil_fw/model/dfig_model_cla.cla) against
the trusted float64 reference it was ported from (dfig_full.py).

This is the comparison harness named as the next step in
DFIG_HANDOFF.md. It catches a translation bug in the port -- a dropped
sign, a swapped term, a wrong variable -- which is the likely failure
mode of a hand port. It does NOT exercise the actual CLA instruction
set, the compiler, or the linker/memory-section wiring; that needs the
real board or a cycle-accurate simulator, which is the step after this
one (see DFIG_HANDOFF.md).
"""

import numpy as np
import pytest

from dfig_hil.params import DEFAULT as P
from dfig_hil.dfig_full import DfigFullModel
from dfig_hil.cla_mirror import ClaMirror

VS = P.derived()["v_s_pk_rated"]
WS = P.derived()["w_s"]


def w_m_for_slip(s):
    return (1.0 - s) * WS / P.pole_pairs


def run_pair(frames):
    """frames: list of (v_rd, v_rq, v_s, w_s, w_m). Returns (ref, mirror)
    arrays of (i_rd, i_rq) per frame."""
    ref = DfigFullModel(P)
    mirror = ClaMirror(P)
    ref_out, mirror_out = [], []
    for v_rd, v_rq, v_s, w_s, w_m in frames:
        ref_out.append(ref.step(v_rd, v_rq, v_s, w_s, w_m))
        mirror_out.append(mirror.cla1task1(v_rd, v_rq, v_s, w_s, w_m))
    return np.array(ref_out), np.array(mirror_out)


def max_rel_err(ref_out, mirror_out, scale):
    err = np.abs(ref_out - mirror_out)
    return float(np.max(err) / scale)


# A float32 port of a stiff ODE integrated for many frames will drift from
# a float64 reference by accumulated rounding error even with a perfectly
# correct port; this is not itself a bug. 1e-3 relative to rated current
# over a few thousand frames is a loose-but-meaningful bound for "the port
# is algebraically right," tightened once real hardware numbers are in.
REL_TOL = 1e-3
I_SCALE = P.derived()["i_rated_pk"]


def test_startup_transient_matches():
    """No excitation, synchronous speed: both models should settle to the
    same no-load steady state from the same _start()/Cla1Task7 init."""
    frames = [(0.0, 0.0, VS, WS, WS / P.pole_pairs) for _ in range(500)]
    ref_out, mirror_out = run_pair(frames)
    assert max_rel_err(ref_out, mirror_out, I_SCALE) < REL_TOL


def test_subsynchronous_step_matches():
    """Step rotor voltage command at a fixed sub-synchronous slip, the
    same scenario validate.py's fig_steps() exercises for A3."""
    s = -0.1
    w_m = w_m_for_slip(s)
    frames = [(0.0, 0.0, VS, WS, w_m) for _ in range(200)]
    frames += [(0.0, 1.6, VS, WS, w_m) for _ in range(1000)]
    ref_out, mirror_out = run_pair(frames)
    assert max_rel_err(ref_out, mirror_out, I_SCALE) < REL_TOL


def test_synchronous_crossing_matches():
    """Speed ramp straight through synchronous speed (Exercise 6) -- the
    case the handoff flagged as most sensitive to a wrong w_sl sign."""
    n = 4000
    frames = []
    for k in range(n):
        s = -0.1 + 0.2 * (k / n)          # sub -> super synchronous
        w_m = w_m_for_slip(s)
        frames.append((0.0, 1.6, VS, WS, w_m))
    ref_out, mirror_out = run_pair(frames)
    assert max_rel_err(ref_out, mirror_out, I_SCALE) < REL_TOL


def test_frequency_deviation_matches():
    """Grid frequency step, exercising cla_in_w_s -- the input this port
    was missing until this comparison harness caught it."""
    w_s_dev = WS * 0.98   # -2% frequency deviation
    frames = [(0.0, 0.0, VS, WS, WS / P.pole_pairs) for _ in range(300)]
    frames += [(0.0, 1.0, VS, w_s_dev, WS / P.pole_pairs) for _ in range(1000)]
    ref_out, mirror_out = run_pair(frames)
    assert max_rel_err(ref_out, mirror_out, I_SCALE) < REL_TOL


def test_clamp_fault_flag_matches():
    """Drive well past the current guard and check both models latch
    the same fault condition at the same time."""
    s = -0.1
    w_m = w_m_for_slip(s)
    ref = DfigFullModel(P)
    mirror = ClaMirror(P)
    ref_fault_frame = mirror_fault_frame = None
    for k in range(3000):
        v_rq = 50.0   # grossly over rated, forces the guard
        ref.step(0.0, v_rq, VS, WS, w_m)
        mirror.cla1task1(0.0, v_rq, VS, WS, w_m)
        if ref_fault_frame is None and ref.fault:
            ref_fault_frame = k
        if mirror_fault_frame is None and mirror.cla_out_fault:
            mirror_fault_frame = k
    assert ref_fault_frame is not None, "reference never latched a fault"
    assert mirror_fault_frame is not None, "CLA port never latched a fault"
    assert abs(ref_fault_frame - mirror_fault_frame) <= 2
