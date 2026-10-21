# DFIG HIL Trainer: handoff

Status as of 21 October 2026. This file is self-contained. It sums up the decisions, the work done, the results and the next steps, so that work can continue in Claude Code on the Windows laptop without the earlier chat.

---

## 1. Goal and the decision taken

The goal is a **DFIG (doubly-fed induction generator) wind energy trainer**. The **HIL (hardware-in-the-loop) variant** was chosen on 21 Oct 2026. In this variant the rotor-side converter is simulated in real time by a plant model running at 10 kHz, and the control code is real firmware running on a TI C2000 microcontroller.

- The governing spec is `PRD_DFIG_HIL_Trainer.md` (PRD-DFIG-HIL-001 v0.9). **Where the repo's documents disagree, follow the HIL PRD.**
- The work starts with PRD phase **P0**: a Python reference plant model, validated before any firmware is written. P0 is done (section 5).

## 2. Repository

- GitHub: https://github.com/Reynlab-Dev/DFIG (public). The local copy is at `D:\DFIG`.
- Before this work the repo held one commit with three documents and no code. The documents describe **three different trainers**:

| File | Variant | Key specs |
|---|---|---|
| `PRD_DFIG_HIL_Trainer.md` | **HIL (chosen)** | 1 HP 415 V slip-ring machine + 2 HP DC motor; rotor converter simulated at 10 kHz; F28379D dual core; 1024 PPR encoder |
| `DFIG_Wind_Energy_Conversion_System_Manual.pdf` | Full-power | STM32F407, 1200 V / 30 A IGBT rotor converter |
| `DFIG-LV-FAB-01_Fabrication_Booklet.md.pdf` | Low-voltage | 110 V stator, 48 V battery DC link, BOOSTXL-3PHGANINV, VFD prime mover, 20 kHz loop |

The three disagree on machine rating, prime mover, loop rate and encoder PPR. Ignore the other two except as background.

## 3. Hardware and platform

| Item | Detail |
|---|---|
| Controller on hand | **TI LAUNCHXL-F280049C**: one C28x core at 100 MHz with FPU, plus a **CLA** co-processor; on-board XDS110 debug probe with a virtual COM port; no native USB device. **2 DACs** on the LaunchPad, where the PRD assumed 3 (scope channels must be reduced to 2). |
| PRD's assumed controller | LAUNCHXL-F28379D, dual C28x + 2 CLAs. The plan below adapts the PRD to the single-core part. |
| PC | Windows. Suggested machine: HP small-form-factor PC, Intel i5 8th gen, 16 GB RAM, Windows 11. |
| Inverter board | Not needed for the simulation-only build. A BOOSTXL-3PHGANINV pairs with this LaunchPad later if real power stages are added. |

**Simulation-only plan: what runs where**

| Where | What | Rate |
|---|---|---|
| C28x | Control code: ADC/feedback read, Clarke/Park, 2× PI, inverse Park, SVPWM, outer P/Q or MPPT loop (1 kHz), protection state machine, telemetry (100 Hz) | 10 kHz ISR |
| CLA | Plant model: grid, converter (duty → rotor voltage), DFIG machine, shaft and DC drive, encoder emulation, turbine (200 Hz) | 10 kHz |
| PC | Dashboard/HMI, scenarios (wind profiles, grid sags), CSV logging, the Python reference model for cross-checking | — |

The C28x and the CLA talk only through a shared-RAM struct, `plant_feedback_t` (PRD §6.5). On the F28379D, the PRD's two cores enforced that boundary. Here the CLA enforces it, and it is the "swap point" for adding a real converter in Phase 2.

## 4. Staged test plan

1. **Python (done):** model validated against an independent reference and against the PRD acceptance criteria.
2. **Firmware on the LaunchPad:** port the model to the CLA and the control code to the C28x. Measure the execution budget: model ≤ 25 µs and control ≤ 20 µs of each 100 µs frame (PRD §5.5).
3. **Compare against Python:** feed identical input vectors (duty cycles, speed, grid events) to the C build and to the Python model, and compare their outputs frame by frame over UART/CSV. The comparison should agree to float32 precision.

## 5. The P0 Python model

### Where it is

- **Branch** `p0-python-plant-model`, one commit on top of `main`. It is **not on GitHub yet** (see section 8).
- **Files**, next to this handoff:
  - `dfig-p0-python-model.zip`: the `python/` folder and `.gitignore`.
  - `0001-p0-python-plant-model.patch`: the same commit as a git patch.
  - `run-2026-10-21/`: output of a clean run and all 7 figures.

### Layout of `python/`

```
python/
  dfig_hil/
    params.py       parameter table, twin of dfig_params.h; write_c_header() emits it
    dfig.py         PRD §5.2 two-state rotor-current model (Euler or trapezoidal),
                    ±3x rated clamp, NaN check, latched MODEL_FAULT, reset();
                    option stator_rs=True keeps R_s in the stator equation
    dfig_full.py    full 4th-order model (stator + rotor flux), CLA-portable:
                    trapezoidal at 100 us, explicit 2x2 complex solve
    converter.py    duty[3] -> rotor phase voltages on V_dc -> Clarke/Park at slip angle
    grid.py         stator supply V/f/angle + sag, swell, frequency-step event table
    shaft.py        J dw/dt = T_pm + T_e - B w; DC drive as torque lag, speed or torque mode
    encoder.py      1024 PPR x4 eQEP count (0..4095), M-method speed over 20 ms ring buffer
    turbine.py      Cp(lambda, beta), wind power, torque reference at 200 Hz
    plant.py        one 100 us frame in fixed order; PlantFeedback mirrors plant_feedback_t
                    Plant(P, model="prd" | "prd_rs" | "full")
    reference.py    validation only: RK4 4th-order machine at 10 us + closed-form steady states
    control_ref.py  plain PI current controller used only by the tests (not student code)
  tests/test_model.py   82 tests
  validate.py           prints the acceptance numbers, writes figures/*.png
  requirements.txt      numpy, matplotlib, pytest
  README.md             full write-up and open questions
```

**Frame order** (the C port must keep it): grid → converter → DFIG → turbine (every 50th frame) → shaft → encoder.

**Conventions:**
- **Motor convention** throughout, as in the PRD equations: P_s, Q_s > 0 means the stator absorbs power, and T_e > 0 is motoring torque, so generation shows as negative values.
- dq quantities are amplitude-invariant (peak phase values).
- Rotor quantities are referred to the stator. The turns ratio N_r/N_s = 2 applies only at the interface: `PlantFeedback.i_r_abc` holds actual slip-ring amps.

### Run it on Windows

Install Python 3.10 or newer from python.org and tick "Add to PATH". Then, in a Command Prompt:

```bat
cd D:\DFIG\python
py -m pip install -r requirements.txt
py -m pytest -q
py validate.py
```

The figures are written to `D:\DFIG\python\figures\`.

### Results from a clean run (21 Oct 2026)

These numbers come from running the shipped zip in a fresh virtual environment: 82 tests passed in 0.8 s.

| Check | Result | Limit | |
|---|---|---|---|
| A1 encoder speed error at 1800 rpm | 0.62 rpm | ±1 rpm | pass |
| A2 rotor EMF vs s·(N_r/N_s)·V_s, s 0.05–0.4 | 4.0 % | 5 % | pass |
| A3 PRD §5.2 model vs full machine, same rotor voltage, s ±0.3 | P_s 20.1 %, Q_s 31.1 %, T_e 19.7 % of rated | 3 % | **fail** |
| A3 §5.2 + R_s vs full machine | 0.0 % | 3 % | pass |
| A4 reference PI (200 Hz) on §5.2 model | 2.7 ms settling, 0 % overshoot | < 5 ms, < 10 % | pass |
| §5.6 step settling vs full machine | §5.2 settles in ~3 ms; full machine rings > 100 ms | within 10 % | **fail** |
| Full model (target form) vs RK4 reference, closed loop | within 2 % of rated | — | pass |
| 30 % / 150 ms grid sag, peak rotor current | 1.28 A on the §5.2 model, 1.93 A on the full model | — | — |

## 6. Findings that change the design

1. **PRD §5.2 neglects stator resistance.** With R_s = 10 Ω, about 6 % of base impedance, the §5.2 model fails A3 by 20–31 % of rated. The `stator_rs=True` variant matches the full machine exactly in steady state, for one extra complex divide per frame.
2. **PRD §5.2 has no stator-flux dynamics.** Under current control, the real machine's stator flux rings at grid frequency with a time constant of about L_s/R_s ≈ 62 ms. As a result:
   - PI gains tuned at 200 Hz on the §5.2 model go **unstable on the full machine above synchronous speed**, with the oscillation growing to about 2 A peak-to-peak. Gains at 400 Hz are stable.
   - The §5.2 model hides the grid-sag transient, which is the subject of Exercise 9.
   - The §5.6 settling check cannot pass against any full reference model.
3. **Recommendation: run the full 4th-order model (`dfig_full.py`) on the CLA.** It costs about 80 floating-point operations per frame, which fits easily in the 25 µs budget. Keep the §5.2 equations as the ones students derive and compare against in Exercise 3. **This needs the user's confirmation.**
4. **Exact vs approximate rotor power.** The PRD's P_r ≈ −s·P_s ignores rotor copper loss. At i_rq = 1.6 A the exact P_r crosses zero near 1590 rpm, not 1500 rpm. The model exposes both values (`p_r`, `p_r_approx`).
5. **Encoder speed.** A per-frame count difference resolves only 146 rpm, so the 20 ms M-method window is used (0.73 rpm resolution, about 10 ms delay). The rotor angle for the Park transform must come from the raw count, not from the speed estimate.

### Parameter choices that need confirming

| Parameter | Value used | Why |
|---|---|---|
| L_ls, L_lr | 0.025 H each | PRD says "0.05 H, split 50/50". Reading that as 0.025 H each matches the PRD's own ≈4 ms rotor time constant; 0.05 H each would give 8 ms |
| L_m | 0.60 H (PRD value) | Implies a no-load current of about 1.27 A, which is 91 % of the 1.4 A rating and suspiciously high. Exercise 1 should measure it |
| Rotor DC link V_dc | 500 V actual (250 V referred) | Not in the HIL PRD; covers \|s\| ≤ 0.3 at 415 V with SVPWM |
| Turbine | R = 0.68 m, gear ratio 1.2, ρ = 1.225, standard λ_i formula | Not in the PRD; gives Cp_max 0.48 at λ 8.1 and about 740 W at 12 m/s near 1640 rpm |
| Default stator voltage | 415 V | Use `Plant(P, v_ll=150)` for the reduced-voltage exercises in PRD §9.2(a) |

Other parameters follow PRD §5.3 (placeholders until Exercise 1): R_s 10 Ω, R_r 12 Ω, L_m 0.6 H, J 0.02 kg·m², B 0.001 N·m·s, p = 2, 50 Hz, 1410 rpm rated, turns ratio 1:2.

## 7. Artifact links (claude.ai)

- Project overview: https://claude.ai/artifact/RvY6V4Us48DymitZcGs57H
- Repo walkthrough: https://claude.ai/artifact/EppS71vKjkEPSvf9CDQUDK
- Plant model preview (diagram, how each part works, charts from real runs): https://claude.ai/artifact/CeWASDaDNiCbDVtLwjb6Zn

## 8. Status and blockers

- **GitHub push is blocked.** Claude's cloud session cannot push to Reynlab-Dev/DFIG because the Claude GitHub App is not installed on the Reynlab-Dev organization. Fix it at https://claude.ai/connect-github, or have an org owner install it from https://github.com/apps/claude/installations/select_target. Pushing from the laptop with your own git credentials works regardless.
- **To get the code into `D:\DFIG` with history**, apply the patch in Git Bash or PowerShell:

```bash
cd /d/DFIG                     # PowerShell: cd D:\DFIG
git checkout main && git pull
git checkout -b p0-python-plant-model
git am path/to/0001-p0-python-plant-model.patch
git push -u origin p0-python-plant-model
```

  Then open a PR into `main`. Alternatively, unzip `dfig-p0-python-model.zip` into `D:\DFIG` and commit it.

## 9. Next steps, in priority order

1. **Get P0 onto GitHub.** Apply the patch, push the branch and open a PR (section 8).
2. **Decide on the CLA model.** The recommendation is the full 4th-order model (finding 3). Also confirm the parameter choices in section 6.
3. **Set up the firmware skeleton in Code Composer Studio** for the F280049C. Install CCS and C2000Ware, then start from a C2000Ware F28004x example with a CLA task. Create the PRD §6.4 layout, adapted to one core:
   - `src/main_cpu1.c` (C28x control)
   - `src/cla_tasks.cla` (the plant model)
   - `src/control/` (park_clarke, pi_dcl, svpwm, outer_loop: the student-editable files)
   - `src/model/` (dfig_model, turbine_model, grid_events, `dfig_params.h` generated by `params.write_c_header()`)
   - `src/platform/` (plant_iface, protection, telemetry)
4. **Set up the 10 kHz timing.** An ePWM time base triggers the C28x ISR. The CLA task is forced each frame, and the duty[3] → model → feedback handshake goes through shared message RAM. Toggle a GPIO to measure the budget: model ≤ 25 µs, control ≤ 20 µs. Keep an overrun counter.
5. **Port the model to CLA C**, following `dfig_full.py` (or `dfig.py`), `converter.py`, `encoder.py`, `grid.py` and `shaft.py` line by line. Use `CLAsin`/`CLAcos` from the CLA math library. Keep the frame order.
6. **Build the frame-by-frame comparison harness.** Stream inputs and outputs over SCI (XDS110 virtual COM) and compare them with the Python model on the PC.
7. **Port the control loop** (Park, PI with anti-windup, SVPWM) and check A4 on the LaunchPad. Use at least a 400 Hz bandwidth, or add stator-flux damping (finding 2).
8. **Add telemetry** (PRD Appendix B frame, 100 Hz) and a minimal PC dashboard with M/S provenance badges (PRD §7.4).
9. **Add the protection state machine** (latched faults, RESET only), grid-event injection and the turbine/MPPT loop.
10. **Map the 2 DAC channels** to the PRD's scope test points. The PRD assumed 3.
