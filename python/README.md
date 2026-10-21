# DFIG HIL trainer: Python reference plant model (phase P0)

This is the P0 deliverable of `PRD_DFIG_HIL_Trainer.md` (PRD-DFIG-HIL-001 v0.9):
a reference model of the plant that will later run on the CLA of the
LAUNCHXL-F280049C, with tests and plots that validate it.

Where the repo's documents disagree, this follows the HIL PRD. The one
deliberate departure is the target. The PRD assumes a dual-core F28379D.
This build targets a single-core F280049C running simulation only, with
control on the C28x and the plant on the CLA at 10 kHz. On that build the
shaft, grid and encoder are simulated too, so they are modelled here.

## Quick start

```bash
cd python
pip install -r requirements.txt
python -m pytest -q        # 82 tests, about 1 s
python validate.py         # prints the acceptance numbers, writes figures/
```

```python
from dfig_hil import Plant, DEFAULT as P
from dfig_hil.converter import vr_dq_to_duties

plant = Plant(P, rpm0=1650, model="prd")   # "prd" | "prd_rs" | "full"
plant.grid.add_sag(t_start=0.5, duration=0.15, depth=0.30)
for _ in range(10000):                     # 1 s of 100 us frames
    duty = vr_dq_to_duties(0.0, 60.0, P.v_dc, plant.theta_r, P.turns_ratio)
    fb = plant.step(duty=duty)             # fb mirrors plant_feedback_t (§6.5)
print(fb.P_s, fb.Q_s, fb.T_e, plant.encoder.rpm_est)
```

## What is in the package

| File | PRD | What it does |
|---|---|---|
| `params.py` | §5.3, §4.1 | One parameter table, the twin of `dfig_params.h`. `write_c_header()` writes the header. |
| `dfig.py` | §5.2, §5.5 | The §5.2 two-state rotor-current model, verbatim. Euler or trapezoidal, ±3× rated clamp, NaN check, latched MODEL_FAULT, `reset()`. Option `stator_rs=True` adds R_s (see Q2). |
| `dfig_full.py` | (none) | A full 4th-order model in target form: trapezoidal at 100 µs, explicit 2×2 complex solve, no loops (see Q3). |
| `grid.py` | §7.2, `grid_events.c` | Stator supply: V, f, angle, and sag/swell/frequency events from a fixed table. |
| `shaft.py` | §4.1 E9, §5.3 | J·dω/dt = T_pm + T_e − Bω. The 4-quadrant DC drive is a first-order torque lag with a torque limit, in speed mode or torque mode. |
| `encoder.py` | E2, A1 | 1024 PPR ×4 eQEP counter (0..4095, wraps) and an M-method speed estimate over a 20 ms ring buffer. |
| `converter.py` | §5.2, §6.3 | Duty[3] → rotor phase voltages (average-value bridge on V_dc) → stator-flux dq. Transforms. |
| `turbine.py` | §5.4 | Cp(λ, β), wind power, and the drive torque reference at 200 Hz. |
| `plant.py` | §6.3, §6.5 | One 100 µs frame in a fixed order. Returns `PlantFeedback` with `src_flags`. |
| `reference.py` | §5.6 | Validation only: an RK4 4th-order machine at 10 µs and closed-form steady states. It stands in for the Simulink/PLECS reference until one exists. |
| `control_ref.py` | (none) | A plain PI rotor-current controller used only to drive the plant in tests. It is not the students' controller. |

The model code uses scalar float arithmetic only, with coefficients
precomputed and no allocation in `step()`, so it ports to CLA C line by
line. `test_single_precision_matches_double` runs the model in float32,
which is what the CLA has, and it agrees with float64 to 1e-4 of rated.

### Conventions

* **Motor convention**, the one the PRD's §5.2 equations are written in.
  P_s, Q_s > 0 means the stator absorbs power and T_e > 0 is motoring
  torque, so generating shows P_s < 0 and T_e < 0. The HMI will probably
  want to flip the sign for display.
* dq values are amplitude-invariant (peak phase). V_s in §5.2 is peak phase
  voltage, √(2/3)·V_LL. The model's d axis sits 90° behind the grid voltage.
* Rotor quantities inside the model are referred to the stator.
  `PlantFeedback.i_r_abc` and the slip-ring EMF are actual values, using the
  1:2 turns ratio.

## Validation results

From `python validate.py` with the PRD's placeholder parameters:

| Check | Result | Limit | |
|---|---|---|---|
| A1 encoder speed error at 1800 rpm | 0.62 rpm | ±1 rpm | pass |
| A2 rotor EMF vs s·(N_r/N_s)·V_s, s ∈ [0.05, 0.4] | 4.0 % | 5 % | pass (the gap is L_m/L_s) |
| A3 §5.2 model vs full machine, same rotor voltage, s ∈ [−0.3, 0.3] | P_s 20 %, Q_s 31 %, T_e 20 % of rated | 3 % | **fail**, see Q2 |
| A3 §5.2 + R_s vs full machine | 0.0 % | 3 % | pass |
| A4 reference PI at 200 Hz on the §5.2 model | 2.7 ms, no overshoot | < 5 ms, < 10 % | pass |
| §5.6 step settling vs full machine | §5.2 model settles in ~3 ms; the full machine rings for > 100 ms | within 10 % | **fail**, see Q3 |
| Full model in target form vs RK4 reference | within 2 % of rated, closed loop | | pass |
| Power balance P_s + P_r = T_e·ω_m + copper loss | within 0.5 % of rated | | pass |
| P_r changes sign across synchronous speed | yes | | pass |

Figures are in `figures/`:

1. `01_rotor_emf.png`: open-circuit slip-ring EMF against slip (Exercise 2, A2).
2. `02_a3_steady_state.png`: P_s, Q_s and T_e against slip for the three models.
3. `03_step_response.png`: i_rq step at s = −0.1 with 200 Hz and 400 Hz loops.
4. `04_sync_crossing.png`: speed ramp through 1500 rpm (Exercise 6).
5. `05_grid_sag.png`: a 30 % / 150 ms sag at 1650 rpm (Exercise 9).
6. `06_encoder.png`: speed estimate on a run-up, and its error.
7. `07_cp_curve.png`: the turbine Cp(λ, β) curves.

## Open questions

These came up while building the model. Q2 and Q3 change the design. The
rest are parameter values to confirm.

**Q1. Leakage inductance: is it 0.05 H per side, or 0.05 H in total?**
§5.3 says "L_ls, L_lr = 0.05 H, split 50/50". If that is 0.05 H each,
σL_r/R_r = 8 ms. That contradicts §5.5, which says "≈ 4 ms, 40 steps".
The model uses **0.025 H each** (0.05 H total), which gives 4.08 ms and
matches §5.5. Exercise 1's blocked-rotor test settles this.

**Q2. §5.2 neglects R_s, and with R_s = 10 Ω this fails A3.** R_s is about
6 % of base impedance, which is not negligible on a 1 HP machine. The
error is 4–5 % of rated when a controller holds the same rotor current,
and 20–31 % when the same rotor voltage is applied, which is how the HIL
plant is driven. The `stator_rs=True` variant keeps the stator algebraic
but solves the flux from v_s = R_s·i_s + jω_s·ψ_s each frame. It matches
the full machine exactly in steady state, for the cost of one complex
divide per frame.

**Q3. The §5.2 model has no stator-flux dynamics. Should the CLA run the
full model instead?** When a controller holds the rotor current, the real
machine's stator flux rings at grid frequency with a time constant of
about L_s/R_s = 62 ms. The §5.2 model cannot show this. In practice:

* PI gains tuned on the §5.2 model (200 Hz bandwidth) leave the full machine
  with an oscillation at super-synchronous speed that grows instead of
  decaying, to about 2 A peak to peak after 1 s at s = −0.1 (fig. 3).
  This breaks the PRD's promise that HIL-tuned firmware drives the real
  converter unchanged (G5, A9).
* A grid sag gives a 1.93 A rotor-current peak and a ringing transient on
  the full model, but 1.28 A and almost nothing on the §5.2 model (fig. 5).
  Exercise 9 is about exactly that transient.
* The §5.6 settling-time check against any full reference (Simulink or
  PLECS included) cannot pass.

`dfig_full.py` runs the full model in target form: about 80 flops per
frame, trapezoidal, and within 2 % of the RK4 reference. That fits easily
in the 25 µs CLA budget. The recommendation is to run `"full"` on the CLA
and keep §5.2 as the equations students derive and compare against
(Exercise 3).

**Q4. What is the rotor-side DC link voltage?** The HIL PRD has no DC bus.
The model uses V_dc = 500 V actual (250 V referred), which covers
|s| ≤ 0.3 at 415 V with SVPWM. At the 110–150 V stator voltage in §9.2(a),
a proportionally lower V_dc gives a more realistic duty-cycle resolution.

**Q5. Is L_m = 0.6 H realistic?** It gives a no-load current of about
1.27 A, which is 91 % of the 1.4 A rating. That is high even for a 1 HP
machine, and it makes Q_s large (≈ 880 var magnetising). Check this in
Exercise 1.

**Q6. What are the turbine parameters?** §5.4 does not give R, ρ or the gear
ratio, and it does not define λ_i. The model uses the standard λ_i
expression, R = 0.68 m and a 1.2 gear ratio. With those, Cp_max = 0.48 at
λ = 8.1, and 12 m/s gives about 740 W near 1640 rpm (s ≈ −0.09).

**Q7. Should Exercise 6's table use the exact rotor power?** The PRD's
P_r ≈ −s·P_s ignores rotor copper loss. At i_rq = 1.6 A the exact rotor
power crosses zero near 1590 rpm, not 1500 rpm (fig. 4), so a student's
P_s + P_r = P_mech table will not close using the approximation.
`DfigModel` exposes both `p_r` (exact) and `p_r_approx`.

**Q8. Which speed method does the control side use?** A per-frame count
difference is useless (146 rpm per count). The 20 ms M-method meets A1
but adds about 10 ms of delay. That is fine for slip and display, but the
rotor angle for the Park transform should come from the count directly,
as `Encoder.theta_m()` does. The eQEP capture unit (T-method) is the
alternative if a faster speed signal is needed.

**Q9. Is 415 V the right default stator voltage?** The model defaults to
415 V for validation. For the reduced voltage in §9.2(a), use
`Plant(P, v_ll=150)`. On a simulation-only build there is no shock hazard,
but matching the shipped exercise conditions keeps the numbers comparable.

## Next steps (P2 on the F280049C)

* Port `dfig_full.py` (or `dfig.py`) and `encoder.py` to CLA C. Then run the
  same input vectors through both the C and Python versions and compare
  outputs frame by frame.
* Use `params.write_c_header()` to generate `dfig_params.h`.
* sin/cos in `converter.py` map to `CLAsin`/`CLAcos` from the CLA math library.
