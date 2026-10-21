"""P0 validation run: prints the acceptance numbers and writes plots to figures/.

    python validate.py            # all figures, about 20 s
"""

import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from dfig_hil.params import DEFAULT as P
from dfig_hil.dfig import DfigModel
from dfig_hil.dfig_full import DfigFullModel
from dfig_hil.plant import Plant
from dfig_hil.converter import vr_dq_to_duties
from dfig_hil.control_ref import RefCurrentController
from dfig_hil.turbine import cp
from dfig_hil import reference as ref

D = P.derived()
VS, WS = D["v_s_pk_rated"], D["w_s"]
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures")

# reference palette, categorical slots 1-3 in fixed order
C_PRD, C_RS, C_FULL = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
LABEL = {"prd": "PRD §5.2 model", "prd_rs": "§5.2 + R_s", "full": "Full 4th-order"}
COLOR = {"prd": C_PRD, "prd_rs": C_RS, "full": C_FULL}

plt.rcParams.update({
    "figure.dpi": 110, "savefig.dpi": 130, "font.size": 9,
    "axes.edgecolor": INK2, "axes.labelcolor": INK2, "xtick.color": INK2,
    "ytick.color": INK2, "axes.grid": True, "grid.color": GRID,
    "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 2.0, "legend.frameon": False, "text.color": INK,
})


def w_m_for(s):
    return WS * (1.0 - s) / P.pole_pairs


def make(kind):
    if kind == "full":
        return DfigFullModel(P)
    return DfigModel(P, stator_rs=(kind == "prd_rs"))


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print("  wrote", os.path.relpath(path))


# ------------------------------------------------------------------- A2

def fig_rotor_emf():
    slips = np.linspace(-0.4, 0.4, 41)
    emf = []
    for s in slips:
        m = DfigModel(P)
        m.step(0.0, 0.0, VS, WS, w_m_for(s))
        emf.append(m.v_r_emf * P.turns_ratio / math.sqrt(2.0) * math.sqrt(3.0))  # V rms L-L
    ideal = np.abs(slips) * P.turns_ratio * P.v_ll_rated
    fig, ax = plt.subplots(figsize=(6, 3.4))
    ax.plot(slips, ideal, color=INK2, lw=1.2, ls="--", label="s·(N_r/N_s)·V_s")
    ax.plot(slips, emf, color=C_PRD, label="model, open rotor")
    ax.set_xlabel("slip s")
    ax.set_ylabel("slip-ring EMF (V rms L-L)")
    ax.set_title("Open-circuit rotor EMF at 415 V stator (A2)", loc="left")
    ax.legend()
    save(fig, "01_rotor_emf.png")
    err = max(abs(e - i) / i for e, i, s in zip(emf, ideal, slips) if 0.05 <= s <= 0.4)
    return 100 * err


# ------------------------------------------------------------------- A3

def fig_a3():
    slips = np.linspace(-0.3, 0.3, 25)
    target = complex(0.3, 1.6)                     # generating, slightly magnetising
    res = {k: [] for k in ("prd", "prd_rs", "full")}
    for s in slips:
        v_r = ref.vr_for_ir_reduced(P, VS, WS, s, target)
        for k in ("prd", "prd_rs"):
            m = make(k)
            for _ in range(1000):
                m.step(v_r.real, v_r.imag, VS, WS, w_m_for(s))
            res[k].append((m.p_s, m.q_s, m.t_e))
        i_s, i_r = ref.steady_full_from_vr(P, VS, WS, s, v_r)
        res["full"].append(ref.outputs_full(P, VS, WS, i_s, i_r))
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.3))
    names = ("P_s (W)", "Q_s (var)", "T_e (N·m)")
    for j, ax in enumerate(axes):
        for k in ("full", "prd", "prd_rs"):
            y = [r[j] for r in res[k]]
            ax.plot(slips, y, color=COLOR[k], label=LABEL[k] if j == 0 else None,
                    ls=":" if k == "prd_rs" else "-")
        ax.set_xlabel("slip s")
        ax.set_title(names[j], loc="left")
    fig.legend(loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.1))
    fig.suptitle("Steady state, same rotor voltage applied (A3: < 3 % of rated)", x=0.01, y=1.04, ha="left")
    save(fig, "02_a3_steady_state.png")
    base = (D["s_base"], D["s_base"], D["t_rated"])
    worst = {}
    for k in ("prd", "prd_rs"):
        worst[k] = [100 * max(abs(a[j] - b[j]) / base[j] for a, b in zip(res[k], res["full"]))
                    for j in range(3)]
    return worst


# ------------------------------------------------------- step responses

def closed_loop(kind, s, bw, seconds, i_ref=1.6):
    m = make(kind)
    if kind == "full":
        f = ref.FullDfig(P)
        v0 = ref.vr_for_ir_reduced(P, VS, WS, s, 0j)
        f.init_steady(VS, WS, w_m_for(s), v0.real, v0.imag)
        m.psd, m.psq, m.prd, m.prq = f.x
        m.started = True
    c = RefCurrentController(P, bandwidth_hz=bw)
    y = []
    for _ in range(int(seconds / P.dt)):
        vd, vq = c.step(0.0, i_ref, m.i_rd, m.i_rq, s, VS, WS)
        m.step(vd, vq, VS, WS, w_m_for(s))
        y.append(m.i_rq)
    return np.array(y)


def settle(y, target, band=0.02):
    out = np.nonzero(np.abs(y - target) > band * abs(target))[0]
    return (out[-1] + 1) * P.dt if len(out) else 0.0


def fig_steps():
    s = -0.1
    t = np.arange(int(0.12 / P.dt)) * P.dt * 1e3
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.3), sharey=True)
    stats = {}
    for ax, bw in zip(axes, (200.0, 400.0)):
        for k in ("full", "prd"):
            y = closed_loop(k, s, bw, 0.12)
            ax.plot(t, y, color=COLOR[k], label=LABEL[k], lw=1.6)
            stats[(k, bw)] = (settle(y, 1.6) * 1e3, 100 * (y.max() / 1.6 - 1))
        ax.axhline(1.6, color=INK2, lw=0.8, ls="--")
        ax.set_title(f"{bw:.0f} Hz current loop, s = {s}", loc="left")
        ax.set_xlabel("time (ms)")
    axes[0].set_ylabel("i_rq (A, referred)")
    axes[0].legend(loc="lower right")
    fig.suptitle("i_rq step 0 → 1.6 A with the reference PI controller", x=0.01, y=1.04, ha="left")
    save(fig, "03_step_response.png")
    return stats


# -------------------------------------------- synchronous crossing (Ex. 6)

def fig_sync_crossing():
    plant = Plant(P, rpm0=1300.0)
    c = RefCurrentController(P, bandwidth_hz=400.0)
    rpm, ps, pr, pt = [], [], [], []
    seconds = 4.0
    n = int(seconds / P.dt)
    for k in range(n):
        plant.shaft.set_speed_rpm(1300.0 + 400.0 * k / n)
        m = plant.dfig
        vd, vq = c.step(0.0, 1.6, m.i_rd, m.i_rq, m.slip, plant.grid.v_s, plant.grid.w_s)
        plant.step(duty=vr_dq_to_duties(vd, vq, P.v_dc, plant.theta_r, P.turns_ratio))
        if k % 50 == 0 and k > 2000:
            rpm.append(plant.encoder.rpm_est)
            ps.append(m.p_s)
            pr.append(m.p_r)
            pt.append(m.p_r_approx)
    fig, ax = plt.subplots(figsize=(6.5, 3.6))
    ax.plot(rpm, ps, color=C_PRD, label="P_s stator")
    ax.plot(rpm, pr, color=C_RS, label="P_r rotor, exact (incl. R_r loss)")
    ax.plot(rpm, pt, color=C_FULL, ls="--", label="P_r ≈ −s·P_s (PRD §5.2)")
    ax.axvline(1500, color=INK2, lw=0.8, ls="--")
    ax.axhline(0, color=INK2, lw=0.8)
    ax.text(1505, ax.get_ylim()[1] * 0.9, "synchronous", color=INK2)
    ax.set_xlabel("shaft speed from encoder (rpm)")
    ax.set_ylabel("power (W), motor convention")
    ax.set_title("Speed ramp through 1500 rpm at i_rq = 1.6 A (Exercise 6)", loc="left")
    ax.legend(loc="center left")
    save(fig, "04_sync_crossing.png")


# ------------------------------------------------------ grid sag (Ex. 9)

def fig_sag():
    t_ms, out = None, {}
    for kind in ("full", "prd"):
        plant = Plant(P, rpm0=1650.0, model=kind)
        plant.grid.add_sag(0.40, 0.15, 0.30)
        c = RefCurrentController(P, bandwidth_hz=400.0)
        y, tt = [], []
        for k in range(int(0.80 / P.dt)):
            m = plant.dfig
            vd, vq = c.step(0.3, 1.2, m.i_rd, m.i_rq, m.slip, plant.grid.v_s, plant.grid.w_s)
            plant.step(duty=vr_dq_to_duties(vd, vq, P.v_dc, plant.theta_r, P.turns_ratio))
            y.append(math.hypot(m.i_rd, m.i_rq))
            tt.append(plant.t * 1e3)
        keep = np.array(tt) >= 300.0          # skip energisation transient
        out[kind] = np.array(y)[keep]
        t_ms = np.array(tt)[keep]
    fig, ax = plt.subplots(figsize=(7.5, 3.4))
    for kind in ("full", "prd"):
        ax.plot(t_ms, out[kind], color=COLOR[kind], label=LABEL[kind], lw=1.4)
    ax.axvspan(400, 550, color=GRID, alpha=0.6, lw=0)
    ax.text(405, ax.get_ylim()[1] * 0.95, "30 % sag, 150 ms", color=INK2)
    ax.set_xlabel("time (ms)")
    ax.set_ylabel("|i_r| (A, referred)")
    ax.set_title("Rotor current through a grid sag, 1650 rpm, 400 Hz loop (Exercise 9)", loc="left")
    ax.legend(loc="upper right")
    save(fig, "05_grid_sag.png")
    return out["prd"].max(), out["full"].max()


# ------------------------------------------------------------ encoder A1

def fig_encoder():
    plant = Plant(P, rpm0=0.0)
    true, est, t = [], [], []
    for k in range(int(3.0 / P.dt)):
        plant.shaft.set_speed_rpm(min(1800.0, 900.0 * plant.t))
        plant.step()
        if k % 20 == 0:
            true.append(plant.shaft.rpm)
            est.append(plant.encoder.rpm_est)
            t.append(plant.t)
    true, est = np.array(true), np.array(est)
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.3))
    axes[0].plot(t, true, color=INK2, lw=1.2, ls="--", label="true")
    axes[0].plot(t, est, color=C_PRD, label="eQEP M-method, 20 ms")
    axes[0].set_xlabel("time (s)")
    axes[0].set_ylabel("rpm")
    axes[0].set_title("Shaft speed run-up", loc="left")
    axes[0].legend()
    steady = np.array(t) > 2.3
    axes[1].plot(np.array(t)[steady], (est - true)[steady], color=C_PRD)
    axes[1].axhline(1, color=INK2, lw=0.8, ls="--")
    axes[1].axhline(-1, color=INK2, lw=0.8, ls="--")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("error (rpm)")
    axes[1].set_title("Error at 1800 rpm (A1: ±1 rpm)", loc="left")
    save(fig, "06_encoder.png")
    return np.abs(est - true)[steady].max()


def fig_cp():
    lam = np.linspace(0.5, 14, 300)
    fig, ax = plt.subplots(figsize=(6, 3.3))
    for beta, col in ((0.0, C_PRD), (5.0, C_RS), (10.0, C_FULL)):
        ax.plot(lam, [cp(l, beta) for l in lam], color=col, label=f"β = {beta:.0f}°")
    ax.set_xlabel("tip-speed ratio λ")
    ax.set_ylabel("Cp")
    ax.set_title("Turbine Cp(λ, β), §5.4", loc="left")
    ax.legend()
    save(fig, "07_cp_curve.png")


def main():
    print("PRD-DFIG-HIL-001 phase P0 validation")
    e = fig_rotor_emf()
    print(f"A2 rotor EMF vs s*(Nr/Ns)*Vs, s in [0.05, 0.4]: worst {e:.1f} %  (limit 5 %)")
    w = fig_a3()
    for k, v in w.items():
        print(f"A3 {LABEL[k]:<16s} worst error, % of rated: P_s {v[0]:.1f}  Q_s {v[1]:.1f}  T_e {v[2]:.1f}  (limit 3)")
    st = fig_steps()
    for (k, bw), (ts, os_) in sorted(st.items()):
        settled = f"{ts:6.1f} ms" if ts < 119.0 else "not within 120 ms"
        print(f"Step {LABEL[k]:<16s} {bw:.0f} Hz: 2 % settling {settled}, overshoot {os_:5.1f} %")
    fig_sync_crossing()
    a, b = fig_sag()
    print(f"Sag peak rotor current: PRD model {a:.2f} A, full model {b:.2f} A (referred)")
    e = fig_encoder()
    print(f"A1 encoder error at 1800 rpm: {e:.2f} rpm (limit 1)")
    fig_cp()


if __name__ == "__main__":
    main()
