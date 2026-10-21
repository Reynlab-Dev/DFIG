"""Transforms and the simulated rotor-side converter (PRD §5.2 last line, §6.3).

The model does not take v_rd / v_rq from the controller directly. It takes
the three duty cycles the control code writes to the ePWM (the same numbers
the real converter would get), turns them into rotor phase voltages with an
ideal average-value bridge on a DC link V_dc, then into the stator-flux dq
frame. Conventions: amplitude-invariant Clarke, d axis on stator flux.
"""

import math

SQRT3 = math.sqrt(3.0)
TWO_PI = 2.0 * math.pi


def clarke(a, b, c):
    alpha = (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c)
    beta = (b - c) / SQRT3
    return alpha, beta


def inv_clarke(alpha, beta):
    a = alpha
    b = -0.5 * alpha + 0.5 * SQRT3 * beta
    c = -0.5 * alpha - 0.5 * SQRT3 * beta
    return a, b, c


def park(alpha, beta, theta):
    co, si = math.cos(theta), math.sin(theta)
    return alpha * co + beta * si, -alpha * si + beta * co


def inv_park(d, q, theta):
    co, si = math.cos(theta), math.sin(theta)
    return d * co - q * si, d * si + q * co


def flux_angle(theta_v):
    """Stator-flux angle: with R_s neglected the flux lags the voltage by 90 deg."""
    return (theta_v - 0.5 * math.pi) % TWO_PI


def rotor_angle(theta_v, theta_m, pole_pairs):
    """Angle of the stator-flux dq frame seen from the rotor windings
    (slip angle), theta_psi - p * theta_m."""
    return (flux_angle(theta_v) - pole_pairs * theta_m) % TWO_PI


def duties_to_vr_dq(duty, v_dc, theta_r, turns_ratio):
    """duty[3] in [0, 1] -> referred (v_rd, v_rq). Average-value bridge."""
    va, vb, vc = duty[0] * v_dc, duty[1] * v_dc, duty[2] * v_dc
    vn = (va + vb + vc) / 3.0             # floating star of the rotor winding
    alpha, beta = clarke(va - vn, vb - vn, vc - vn)
    d, q = park(alpha, beta, theta_r)
    return d / turns_ratio, q / turns_ratio


def vr_dq_to_duties(v_rd, v_rq, v_dc, theta_r, turns_ratio):
    """Referred (v_rd, v_rq) -> duty[3] with min-max zero-sequence injection
    (equivalent to SVPWM). Used by the reference controller in the tests;
    the student's own svpwm.c replaces it on the target."""
    alpha, beta = inv_park(v_rd * turns_ratio, v_rq * turns_ratio, theta_r)
    a, b, c = inv_clarke(alpha, beta)
    off = 0.5 * (max(a, b, c) + min(a, b, c))
    return [min(max((x - off) / v_dc + 0.5, 0.0), 1.0) for x in (a, b, c)]
