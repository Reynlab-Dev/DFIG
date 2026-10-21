# Product Requirements Document
## DFIG Wind Energy Conversion System — Hardware-in-the-Loop (HIL) Trainer

**Document ID:** PRD-DFIG-HIL-001
**Version:** 0.9 (Draft for internal review)
**Status:** Draft — open questions in §14 must be closed before BOM freeze
**Supersedes:** Nothing. Companion to *DFIG Wind Energy Conversion System Manual v1.0* (full-power variant)
**Date:** 08 September 2026

---

## 1. Summary

### 1.1 What we are building

A DFIG wind energy conversion trainer in which the **electromechanical plant is physically real** and the **rotor-side power conversion is executed in real time by a plant model**, running on the same class of microcontroller that would run the production converter.

The student operates a real 2 HP DC machine driving a real 1 HP slip-ring induction machine. Real shaft speed, real stator voltage, real slip, real rotor EMF at the slip rings. The rotor currents, converter behaviour, and stator power flow are computed by a validated DFIG model at 10 kHz, driven by the real measurements and by the student's own control code.

This is the same architecture used by Typhoon HIL, OPAL-RT and dSPACE, positioned at laboratory-trainer scale and price.

### 1.2 Why

| Driver | Detail |
|---|---|
| Schedule | The 1200 V / 30 A IGBT bridge, isolated gate drive, desaturation protection and DC bus for the full-power variant is the long pole. HIL removes it from the critical path. |
| Risk | A student with a badly tuned PI loop cannot destroy a HIL plant. On the full-power variant they can destroy the IGBT module. |
| Pedagogy | Grid faults, LVRT, wind gusts and converter failures can be exercised on demand. On the full-power unit most of these are unsafe or impossible. |
| Upgrade path | The control firmware written for the HIL unit is the *same firmware* that drives the real converter later. Nothing is thrown away. |

### 1.3 Explicit product-integrity requirement

This unit **must not be delivered or documented as the full-power converter trainer.** It is a distinct SKU with its own manual. Every displayed quantity is tagged as measured or simulated, in hardware, permanently. See §7.4 and §12. This is a hard requirement, not a preference — a college that discovers undeclared simulation loses trust in the entire product line.

### 1.4 Goals

- **G1** — Preserve genuine physics: slip, rotor EMF, synchronous crossing and mechanical dynamics are measured, not modelled.
- **G2** — Preserve genuine control engineering: real MCU, real ADC quantisation, real sample-and-compute delay, real discrete PI, real SVPWM.
- **G3** — Deliver 10 laboratory exercises with assessable outcomes at Bloom *Apply* level or above.
- **G4** — Provide unambiguous measured-vs-simulated signal disclosure at the panel, in firmware telemetry, and in the manual.
- **G5** — Firmware and hardware architected so that the real rotor-side converter can be substituted with a documented, bounded change set.

### 1.5 Non-goals

- Not a substitute for the full-power DFIG trainer where the syllabus requires physical power conversion.
- No grid-side converter (GSC) hardware in v1. GSC is modelled only.
- No actual power export to a utility grid. The "grid" is the lab supply through an isolation/variable transformer.
- No real-time code generation toolchain (Simulink Coder / Embedded Coder) as a delivered dependency in v1. Hand-written C.

---

## 2. Users and context

| User | Needs | Success looks like |
|---|---|---|
| UG student (EEE/EIE, 6th–8th sem) | Understand DFIG behaviour, tune a controller, produce a lab record | Completes exercise, gets plots, can explain why the power flow reversed |
| Lab instructor | Run 4 batches/day safely, reset quickly between batches, mark objectively | Zero incidents, one-button reset, auto-generated CSV per student |
| PG / project student | Modify control code, try alternative algorithms | Can flash own firmware, has documented API and model source |
| Faculty / HOD (buyer) | Syllabus mapping, NBA/accreditation evidence, defensible purchase | CO/PO mapping table, sample lab record, honest spec sheet |
| Our service engineer | Install and commission in ≤1 day, diagnose remotely | Documented commissioning procedure with pass/fail gates |

---

## 3. System architecture

### 3.1 Layer model

```
┌─ L5  DISPLAY & HMI ────────────────────────────────────────────┐
│  Panel meters (MEASURED, blue)  │  HMI screen (SIMULATED, amber)│
│  DAC → oscilloscope test points │  PC dashboard + CSV logging   │
└────────────────────────────────────────────────────────────────┘
                    ▲ telemetry 100 Hz (UART/USB)
┌─ L4  CONTROL + MODEL (TI C2000 LaunchPad) ─────────────────────┐
│  CPU1: DQ current control, SVPWM, protection state machine     │
│  CPU2/CLA: DFIG plant model + turbine model @ 10 kHz            │
│  Shared RAM interface struct  (the swap point — see §6.5)       │
└────────────────────────────────────────────────────────────────┘
     ▲ ADC: ω_m, v_s(abc), i_s(abc), v_r(abc)   ▼ DAC/PWM: τ_ref, gate pulses
┌─ L3  SENSING & INTERFACE BOARD ────────────────────────────────┐
│  Encoder input │ Voltage xducers │ Current xducers │ Signal cond│
└────────────────────────────────────────────────────────────────┘
                    ▲ physical quantities
┌─ L2  ELECTRICAL PLANT (REAL) ──────────────────────────────────┐
│  Variac + isolation xfmr → Stator 3φ │ Rotor → slip rings →     │
│  rotor termination (resistor bank / guarded open) │ TI inverter │
│  board present, gate pulses live, output disconnected in HIL    │
└────────────────────────────────────────────────────────────────┘
┌─ L1  MECHANICAL PLANT (REAL) ──────────────────────────────────┐
│  2 HP DC motor + 4-quadrant DC drive ── coupling ── 1 HP SRIM   │
│  Encoder on free shaft end                                      │
└────────────────────────────────────────────────────────────────┘
```

### 3.2 The measured / simulated boundary

This table is the single most important design decision in the product. It goes in the PRD, in the manual, and on the panel.

| Quantity | Source | Rationale |
|---|---|---|
| Shaft speed ω_m | **MEASURED** (encoder) | Drives slip. Everything follows from it. |
| Stator voltage v_s(abc), f_s | **MEASURED** (LV 25-P) | Real grid reference frame, real synchronisation |
| Rotor open-circuit voltage v_r(abc) | **MEASURED** | Proves s·(N_r/N_s)·V_s directly; strongest physical demo |
| Stator current i_s(abc) | **MEASURED** | Real magnetising current, real machine loading |
| Slip s, rotor frequency f_r | **DERIVED from measured** | ω_s from f_s, ω_m from encoder |
| DC motor armature V, I, torque | **MEASURED** | Real prime mover, real mechanical power |
| **Rotor current i_r(abc), i_rd, i_rq** | **SIMULATED** | Requires the converter we are deferring |
| **Stator active/reactive power P_s, Q_s** | **SIMULATED** | Function of rotor current |
| **Electromagnetic torque T_e** | **SIMULATED** | Function of rotor current |
| **Rotor power P_r, DC bus V_dc, I_dc** | **SIMULATED** | No DC bus in HIL configuration |
| **Converter modulation index, duty** | REAL (computed by control code), applied to model | Genuine control output |

**Design principle:** a student who changes *anything physical* — speed, stator voltage, mechanical load — sees the simulated numbers move, because the simulated numbers are downstream of real measurements. If a knob turn does not move the display, we have built a video, not a trainer.

---

## 4. Equipment / Bill of Materials

### 4.1 Retained from the existing full-power design

| Item | Spec | Notes |
|---|---|---|
| DC motor | 2 HP, 220 V armature, 220 V field, 1750 RPM, double-side shaft extension | Unchanged |
| Slip-ring induction machine | 1 HP, 3φ, 415 V / 1.4 A, 50 Hz, 1410 RPM, turns ratio 1:2 | Unchanged. 4-pole ⇒ N_s = 1500 RPM, rated slip 6% |
| Base frame, coupling, guards | Existing | Coupling guard mandatory |
| TI inverter board | Present in-house | Retained, powered, gate pulses live, **power output disconnected** in HIL mode. Keeps gate-pulse scoping real. |

### 4.2 New / changed for HIL variant

| Ref | Item | Specification | Qty | Purpose |
|---|---|---|---|---|
| E1 | Controller | TI LAUNCHXL-F28379D (C2000 Delfino, 200 MHz, dual C28x + 2× CLA, 4× 16-bit ADC, 3× 12-bit DAC) | 1 | Control + model. See §6.1 for alternative. |
| E2 | Incremental encoder | 1024 PPR, 5 V line driver, shaft or hollow-bore | 1 | ω_m to eQEP. **Critical path item.** |
| E3 | Voltage transducer | LEM LV 25-P × 3 (or ZMPT101B × 3 for cost variant) | 3 | Stator v_ab, v_bc, v_ca |
| E4 | Voltage transducer | LEM LV 25-P × 3, rated to 600 V primary | 3 | Rotor slip-ring voltage |
| E5 | Current transducer | LEM LA 25-NP × 3 | 3 | Stator i_a, i_b, i_c |
| E6 | Signal conditioning board | Custom 2-layer, ±15 V rail, differential → 0–3.0 V level shift, 2nd-order anti-alias @ 3 kHz, 3.3 V clamp on every ADC line | 1 | Protects MCU. Non-negotiable clamps. |
| E7 | Variable transformer | 3φ, 0–470 V, 6 A, with isolation transformer | 1 | Stator supply. **Enables reduced-voltage safe operation — see §9.2** |
| E8 | Rotor termination unit | 3φ resistor bank, 0–200 Ω/ph, 500 W, star point switchable; OR interlocked guarded open-circuit terminal | 1 | See §9.2 |
| E9 | DC drive | 4-quadrant thyristor/IGBT DC drive, 0–220 V armature, with **0–10 V analog speed/torque reference input** | 1 | Analog input is mandatory — enables turbine-torque emulation from the model (§5.4) |
| E10 | HMI display | 10.1" capacitive touch, 1280×800, HDMI + USB, panel-mount | 1 | Simulated-quantity display |
| E11 | Display compute | Raspberry Pi 4 (4 GB) or equivalent SBC, panel-mount | 1 | Dashboard, logging, CSV export |
| E12 | Panel meters | 3½-digit digital panel meters, blue bezel: V_s, I_s, V_rotor, N (RPM) | 4 | **MEASURED quantities only** |
| E13 | Test point bank | 8× BNC, buffered, 1 MΩ, from MCU DAC + gate drive taps | 1 | Oscilloscope observation |
| E14 | Mode selector | Key-operated 2-position: HIL / CONVERTER, with position readback to MCU | 1 | Prevents mixed-mode operation. Key held by instructor. |
| E15 | Emergency stop | Latching mushroom, dual-channel, drops DC drive contactor and stator contactor | 1 | Mandatory |
| E16 | Enclosure & wiring | 19" panel, labelled, IP2X on all live terminals | 1 | |

### 4.3 Deferred to Phase 2 (full-power variant)

1200 V/30 A IGBT module, Infineon isolated gate driver with desat, snubber, DC bus capacitor bank, precharge circuit, brake chopper, grid-side converter.

---

## 5. Plant model specification

### 5.1 Reference frame and assumptions

Stator-flux-oriented, ψ_sd = ψ_s, ψ_sq = 0. Stator resistance neglected in flux estimation (valid for f_s = 50 Hz). Magnetic saturation neglected in v1 (v2 may add a saturation curve as an advanced exercise).

### 5.2 Governing equations

Stator currents as a function of rotor currents:

```
i_sd = (ψ_s − L_m · i_rd) / L_s
i_sq = −(L_m / L_s) · i_rq
ψ_s  = V_s / ω_s          (from MEASURED stator voltage and frequency)
```

Rotor circuit — the state equations integrated at 10 kHz:

```
di_rd/dt = ( v_rd − R_r·i_rd + s·ω_s·σL_r·i_rq ) / σL_r
di_rq/dt = ( v_rq − R_r·i_rq − s·ω_s·(σL_r·i_rd + L_m·ψ_s/L_s) ) / σL_r

σ = 1 − L_m² /(L_s·L_r)
s = (ω_s − ω_m·p) / ω_s        (ω_m MEASURED, p = 2 pole pairs)
```

Outputs:

```
P_s = −(3/2)·(L_m/L_s)·V_s·i_rq
Q_s =  (3/2)·V_s·( V_s/(ω_s·L_s) − (L_m/L_s)·i_rd )
T_e = −(3/2)·p·(L_m/L_s)·ψ_s·i_rq
P_r ≈ −s·P_s
P_total = P_s + P_r
```

v_rd, v_rq come from the **student's control code** via the inverse of the modulation stage — the model receives the same duty cycles the real converter would receive.

### 5.3 Parameters

Initial estimates for the 1 HP, 415 V, 4-pole machine (star, per phase). **These are placeholders. Exercise 1 replaces them with values measured from the actual machine in the actual unit shipped.**

| Parameter | Estimate | Source |
|---|---|---|
| R_s | 10 Ω | DC resistance test |
| R_r (referred) | 12 Ω | Blocked rotor |
| L_ls, L_lr | 0.05 H | Blocked rotor, split 50/50 |
| L_m | 0.60 H | No-load |
| J (total, motor+gen+coupling) | 0.02 kg·m² | Retardation test |
| B (viscous) | 0.001 N·m·s | Retardation test |
| Base impedance | 171 Ω | 415/(√3 × 1.4) |

Parameters live in a single header (`dfig_params.h`) and in a commissioning CSV. Each shipped unit gets its own measured set, recorded in the acceptance test report.

### 5.4 Wind turbine model (drives the DC prime mover)

```
λ    = ω_turbine · R / v_wind
Cp   = 0.5176·(116/λ_i − 0.4β − 5)·e^(−21/λ_i) + 0.0068·λ
P_w  = 0.5·ρ·A·v_wind³·Cp(λ, β)
τ_ref = P_w / ω_turbine   →  scaled → DAC → DC drive analog reference
```

Scaled to the 1 HP machine. This closes the loop the other way: the *model commands the real DC motor*, so a simulated wind gust produces a real speed change on a real shaft, which the encoder measures, which changes the real slip. That bidirectional coupling is what makes this a twin rather than a display.

### 5.5 Solver and numerics

- **Method:** forward Euler at T_s = 100 µs. Rotor electrical time constant σL_r/R_r ≈ 4 ms ⇒ 40 steps per time constant. Adequate. Trapezoidal available as a compile-time option for the numerics exercise.
- **Arithmetic:** IEEE-754 single-precision float (FPU present on C28x).
- **Execution budget:** model ≤ 25 µs, control ≤ 20 µs of the 100 µs frame. Overrun sets a fault flag and is displayed — students see real-time violation as a first-class concept.
- **Stability guard:** model states clamped to ±3× rated; NaN/Inf check every frame; either condition trips MODEL_FAULT and latches.

### 5.6 Model validation (gate before shipment)

Cross-validated against a Simulink/PLECS reference model of the same machine. Acceptance: steady-state error < 3% on P_s, Q_s, T_e across slip ∈ [−0.3, +0.3]; step response settling time within 10% of reference. Results are attached to each unit's acceptance report.

---

## 6. Microcontroller platform

### 6.1 Controller selection

**Primary: TI LAUNCHXL-F28379D.**

| Requirement | F28379D | STM32F407 (existing) |
|---|---|---|
| Cores | 2× 200 MHz C28x + 2× CLA | 1× 168 MHz M4F |
| Control + model concurrently | Yes — physical core separation | Tight; needs careful budgeting |
| PWM | 12× ePWM, HRPWM, trip zones | 12× TIM channels |
| ADC | 4× 16-bit, 1.1 MSPS, hardware-triggered | 3× 12-bit |
| DAC | 3× 12-bit on-chip | 2× 12-bit |
| Encoder | 3× eQEP | TIM encoder mode |
| Ecosystem | C2000Ware DCL, SFRA, motor control SDK | CubeMX, MC SDK |

The dual-core split is the deciding factor: **CPU1 runs control, CPU2 runs the model, and they can only talk through a defined shared-memory struct.** That enforces the interface discipline that makes the Phase-2 swap safe. It also lets us tell students, truthfully, that the controller cannot see inside the plant.

**Retain the STM32F407 board as a second controller option.** Firmware is written against a thin HAL so both are buildable. This preserves continuity with the existing manual and gives ARM-familiar departments a path.

### 6.2 Task structure

| Task | Rate | Core | Duty |
|---|---|---|---|
| ADC ISR / current loop | 10 kHz | CPU1 | Read ADC → Clarke → Park → 2× PI → inverse Park → SVPWM → write ePWM + model duties |
| Plant model | 10 kHz | CPU2 | Integrate rotor states, compute P_s/Q_s/T_e, write feedback struct |
| Outer power loop | 1 kHz | CPU1 | P/Q or MPPT → i_rd_ref, i_rq_ref |
| Turbine model | 200 Hz | CPU2 | Cp-λ, τ_ref → DAC → DC drive |
| Protection & state machine | 10 kHz | CPU1 | Overcurrent, overspeed, model fault, overrun, E-stop, latch |
| Telemetry | 100 Hz | CPU1 | Pack frame → SCI/USB → SBC |
| Scope DAC | 10 kHz | CPU1 | 3 user-selectable internal variables → BNC |

### 6.3 Data flow per 100 µs frame

```
encoder ─┐
ADC ─────┼→ [CPU1: measure] → i_rd/i_rq feedback ← [shared struct] ←┐
         │        ↓                                                  │
         │   PI(d), PI(q) → v_rd*, v_rq* → inv Park → SVPWM          │
         │        ↓                    ↓                             │
         │   ePWM pins (real,      duty[3] → [shared struct] → [CPU2: model]
         │   scopeable)                                          integrate
         │                                                       ↓
         └──────────────────────────────────────────── i_r, P_s, Q_s, T_e ┘
```

### 6.4 Firmware module list

```
/src
  main_cpu1.c            control application, state machine
  main_cpu2.c            model application
  /control
    park_clarke.c        transforms (student-editable)
    pi_dcl.c             discrete PI with anti-windup (student-editable)
    svpwm.c              space vector modulation (student-editable)
    outer_loop.c         P/Q, MPPT (student-editable)
  /model
    dfig_model.c         §5.2 equations
    turbine_model.c      §5.4 Cp-λ
    dfig_params.h        per-unit measured parameters
    grid_events.c        sag, swell, freq deviation injection
  /platform
    adc_iface.c          → replaced at Phase 2? NO. Already real.
    plant_iface.c        ← THE SWAP POINT (§6.5)
    protection.c
    telemetry.c
  /hal
    hal_c2000.c / hal_stm32.c
```

Student-editable files are marked in a header banner and are the only files touched in exercises 4–8. A `make restore-golden` target reverts them.

### 6.5 The swap point (Phase 2 upgrade path)

All plant feedback enters the control code through one struct:

```c
typedef struct {
    float i_r_abc[3];      /* rotor currents        */
    float v_dc;            /* DC bus voltage        */
    float P_s, Q_s, T_e;   /* stator power, torque  */
    uint16_t src_flags;    /* bit n = 1 → simulated */
} plant_feedback_t;
```

In HIL mode it is populated by `plant_iface_model()`. In converter mode by `plant_iface_adc()`. **One function pointer assignment, selected by the E14 key switch position.** `src_flags` propagates all the way to the display, so the M/S tagging is driven by firmware truth rather than by a label someone might forget to change.

Estimated Phase-2 change set: 1 new file (`plant_iface_adc.c`, ~200 lines), 0 changes to `/control`, calibration constants only in `dfig_params.h`.

### 6.6 Development schedule mitigation

Build the model **in Python/Simulink on the SBC first**, with the MCU streaming measurements up at 1 kHz and receiving feedback down. Debugging a stiff ODE with a scope and a breakpoint is painful; debugging it in NumPy is not. Port to CPU2 once the equations are validated. The `plant_iface` abstraction means this is also just a different implementation behind the same struct. Budget: 2 weeks Python, 1 week port.

---

## 7. Display and HMI

### 7.1 Three display surfaces

| Surface | Shows | Why separate |
|---|---|---|
| **Panel meters (blue bezel)** | V_s, I_s, V_rotor, N (RPM) | Physically measured. A meter reading a real transducer is the most trustworthy object on the panel. |
| **HMI touchscreen (amber accents)** | i_rd, i_rq, P_s, Q_s, T_e, P_r, V_dc, slip, Cp, λ, wind speed | Every value carries an **S** badge |
| **BNC test points → oscilloscope** | Gate pulses (real), DAC-selected internals | Real gate pulses on real pins are a genuine measurement even in HIL |

### 7.2 HMI screens

1. **Overview** — single-line diagram with animated power-flow arrows that reverse at synchronous speed, live P_s / P_r / P_total, slip, mode banner (SUB-SYNCHRONOUS / SYNCHRONOUS / SUPER-SYNCHRONOUS)
2. **Control** — i_rd_ref / i_rq_ref setpoints, K_p / K_i entry, live step-response plot, tracking error
3. **Waveforms** — 4-channel soft scope, 2 s rolling window, trigger, freeze, PNG export
4. **Machine** — parameter table (from commissioning), editable in instructor mode
5. **Scenarios** — wind profile selection (step / ramp / turbulent / gust), grid event injection (sag depth & duration, frequency deviation), fault injection
6. **Log** — event and fault history with timestamps, CSV export per student ID

### 7.3 Layout rules

- Numeric values in tabular figures, ≥ 28 px, fixed decimal places — no jitter from digit-width changes
- Units always displayed, never inferred
- Power-flow arrow direction is the primary visual; the number is secondary
- Colour is never the sole carrier of meaning (the M/S badge is a letter, not a hue)

### 7.4 Signal provenance disclosure — mandatory

1. Every simulated value on the HMI carries a persistent **S** badge, driven by `src_flags` from firmware.
2. Every measured value carries **M**.
3. A permanent header bar reads: `HIL MODE — rotor-side converter simulated`.
4. A silkscreened legend on the physical panel explains M and S.
5. Exported CSV column headers are suffixed `_meas` or `_sim`.
6. The manual's §2 contains the §3.2 table verbatim.

---

## 8. Learning outcomes

Coded per the CO/MO hierarchy, Bloom level noted. Target is *Apply* or above throughout.

### Course Outcomes

| ID | Code | Outcome | Bloom |
|---|---|---|---|
| co-1 | DFIG-CO-1 | **Explain** DFIG variable-speed operation and **predict** power-flow direction from measured slip | Understand → Apply |
| co-2 | DFIG-CO-2 | **Determine** equivalent-circuit parameters of a wound-rotor machine by standard test and **apply** them to a model | Apply |
| co-3 | DFIG-CO-3 | **Implement and tune** a discrete DQ current controller on a real microcontroller to a specified dynamic performance | Apply → Create |
| co-4 | DFIG-CO-4 | **Analyse** decoupled active/reactive power control and **demonstrate** independent P–Q regulation | Analyse |
| co-5 | DFIG-CO-5 | **Evaluate** converter protection and grid-disturbance response, and **justify** the protection settings chosen | Evaluate |
| co-6 | DFIG-CO-6 | **Distinguish** measured from modelled quantities in a HIL system and **assess** model validity against measurement | Evaluate |

### Module Outcomes

| ID | Parent | Outcome |
|---|---|---|
| mo-1-1 | co-1 | Measure slip and rotor EMF across speed and verify V_r = s·(N_r/N_s)·V_s |
| mo-1-2 | co-1 | Identify the synchronous crossing point experimentally and state the sign change in P_r |
| mo-2-1 | co-2 | Conduct no-load, blocked-rotor and retardation tests; compute R_s, R_r, L_ls, L_lr, L_m, J |
| mo-2-2 | co-2 | Load measured parameters into the plant model and validate against a second method |
| mo-3-1 | co-3 | Derive and implement Clarke/Park transforms in fixed sample time |
| mo-3-2 | co-3 | Tune PI gains to < 5 ms settling, < 10% overshoot, zero steady-state error |
| mo-3-3 | co-3 | Quantify the effect of sample rate and computation delay on loop stability |
| mo-4-1 | co-4 | Vary i_rq at constant i_rd; show P_s changes and Q_s does not |
| mo-4-2 | co-4 | Operate at a commanded power factor and verify with stator measurements |
| mo-5-1 | co-5 | Trigger and interpret overcurrent, overspeed and model-fault trips; execute latch/reset |
| mo-5-2 | co-5 | Apply a grid voltage sag and characterise the rotor current transient |
| mo-6-1 | co-6 | Classify all panel quantities as measured or simulated and justify each |
| mo-6-2 | co-6 | Compare model output against an independent calculation and report the discrepancy |

**CO→PO mapping table for NBA accreditation to be produced as a separate annexe.**

---

## 9. Safety requirements

### 9.1 Standard items

Coupling guard interlocked to the DC drive. Dual-channel E-stop dropping both contactors. Earth continuity test in commissioning. IP2X on all terminals ≥ 50 V. All ADC inputs clamped to 3.3 V independent of the conditioning op-amps.

### 9.2 Rotor terminal hazard — HIGH PRIORITY

With 415 V on the stator and a 1:2 turns ratio, **standstill rotor voltage at the slip rings approaches 830 V line-to-line.** In the full-power design this is terminated into the converter. In HIL the rotor may be left open, which puts a lethal, exposed, unterminated potential on the slip ring assembly in a student laboratory.

Three acceptable mitigations. **The unit ships with at least (a) and (c):**

| | Mitigation | Effect |
|---|---|---|
| **(a)** | Stator energised through variac at **110–150 V L-L** for all standard exercises | Rotor standstill V drops to ~220–300 V; at s = 0.2, ~44–60 V. Slip/EMF physics identical, just scaled. |
| **(b)** | Rotor shorted through the E8 resistor bank (0–200 Ω/ph) | Terminates the winding; also produces real rotor current and real torque |
| **(c)** | Interlocked, guarded, key-locked slip-ring terminal enclosure; access drops the stator contactor | Physical prevention |

Full 415 V operation is instructor-key-gated only, and only with the rotor terminated.

### 9.3 Zero-torque advisory

With the rotor open-circuited, the machine develops no electromagnetic torque; the DC motor spins it freely. The manual must not imply otherwise. The HMI displays `T_e (S)` — instructors must be able to explain that the real shaft is unloaded while the model shows torque. **This is precisely why mitigation (b) is valuable: with the resistor bank in, real rotor current and real torque exist, and the model's T_e can be validated against a real retardation or torque-arm measurement.** Strongly recommended for the shipped configuration.

---

## 10. Laboratory exercises

Sequenced by backward design from §8. Ten exercises, ~26 contact hours.

| # | Title | Hrs | Level | Outcomes | Physical / Simulated |
|---|---|---|---|---|---|
| 1 | **Machine parameter determination** — DC resistance, no-load, blocked-rotor, retardation tests; compute the full equivalent circuit; load results into the model | 3 | Apply | mo-2-1, mo-2-2 | 100% physical |
| 2 | **Slip and rotor EMF characterisation** — sweep DC motor speed 900–1800 RPM, record V_rotor and f_rotor, plot against slip, verify the turns-ratio relation, locate the synchronous point | 3 | Apply | mo-1-1, mo-1-2 | 100% physical |
| 3 | **Model validation** — compare model-predicted rotor EMF and stator current against exercise 1–2 measurements; compute % error; identify where and why the model deviates | 2 | Evaluate | mo-2-2, mo-6-2 | Both — this is the exercise that teaches the HIL concept itself |
| 4 | **Open-loop rotor voltage injection** — command v_rd/v_rq directly, bypassing the PI loops; observe the resulting i_rd/i_rq and cross-coupling terms | 2 | Apply | mo-3-1 | Control real, plant simulated |
| 5 | **DQ current controller implementation and tuning** — write/complete the Park transform and PI code, flash it, apply step references, tune to spec, capture step responses on the soft scope | 4 | Create | mo-3-1, mo-3-2 | Control real, plant simulated |
| 6 | **Sub- and super-synchronous power flow** — drive across synchronous speed at fixed i_rq; observe the sign reversal of P_r; construct the power-balance table P_s + P_r = P_mech | 3 | Analyse | mo-1-2, mo-4-1 | Speed/slip real, power simulated |
| 7 | **Decoupled P–Q control** — step i_rq with i_rd fixed, then the reverse; demonstrate independence; operate at 0.9 lagging and 0.9 leading power factor | 3 | Analyse | mo-4-1, mo-4-2 | Control real, plant simulated |
| 8 | **Wind profile tracking / MPPT** — load a turbulent wind profile; the turbine model drives the real DC motor; implement optimal-λ tracking; compute captured energy vs theoretical maximum | 3 | Create | co-3, co-4 | Prime mover real, driven by model |
| 9 | **Protection and grid disturbance** — trigger overcurrent and overspeed trips; execute latch and RESET; inject a 30% / 150 ms voltage sag; characterise the rotor current transient and discuss LVRT | 2 | Evaluate | mo-5-1, mo-5-2 | Fault logic real, disturbance simulated |
| 10 | **Real-time and numerical effects** — vary the control sample rate and observe stability degradation; switch Euler → trapezoidal; force a frame overrun and observe the fault; discuss discretisation | 2 | Evaluate | mo-3-3, mo-6-1 | Real-time behaviour genuinely real |

Exercises 1, 2, 9 and 10 are wholly or substantially real measurement — deliberately, so the lab record contains genuine experimental data and the student learns where the boundary sits.

---

## 11. Deliverables

| # | Deliverable | Format |
|---|---|---|
| D1 | HIL trainer unit, commissioned | Hardware |
| D2 | Technical manual (distinct from the full-power manual, with §3.2 verbatim in §2) | PDF + printed |
| D3 | Laboratory manual — 10 exercises with procedure, observation tables, sample results | PDF + printed |
| D4 | Instructor guide with model answers, expected value ranges, common failure modes | PDF |
| D5 | Firmware source, MIT-licensed, with student-editable files marked | Git repo + USB |
| D6 | Plant model source (C + Python reference implementation) | Git repo + USB |
| D7 | HMI application source | Git repo + USB |
| D8 | Per-unit commissioning report: measured machine parameters, model validation results, calibration constants | PDF, signed |
| D9 | CO→PO mapping annexe for accreditation | DOCX |
| D10 | Phase-2 upgrade specification | PDF |

---

## 12. Acceptance criteria

| ID | Criterion | Method |
|---|---|---|
| A1 | Encoder speed within ±1 RPM of a reference tachometer, 0–1800 RPM | Comparison |
| A2 | Measured rotor EMF within ±5% of s·(N_r/N_s)·V_s across s ∈ [0.05, 0.4] | Measurement |
| A3 | Model steady-state P_s, Q_s, T_e within 3% of the Simulink reference, s ∈ [−0.3, 0.3] | Cross-validation |
| A4 | Current loop achieves < 5 ms settling, < 10% overshoot with default gains | Step response capture |
| A5 | Frame overrun count = 0 over 1 hour continuous | Telemetry counter |
| A6 | All faults latch; RESET is the only clearing path; auto-restart impossible | Fault injection × 6 types |
| A7 | Every HMI value carries an M or S badge sourced from `src_flags` | Visual + code review |
| A8 | No accessible conductor above 50 V without deliberate key-gated action | Physical inspection |
| A9 | Phase-2 swap demonstrated on the bench: `plant_iface` pointer switched, control code unmodified | Engineering demo |
| A10 | All 10 exercises executed end-to-end by a person who did not build the unit | Dry run |

---

## 13. Phasing

| Phase | Scope | Duration | Exit |
|---|---|---|---|
| **P0** | Machine characterisation, Python model, validation against Simulink | 2 wk | A3 on desktop |
| **P1** | Sensing board, encoder, ADC chain, calibration | 2 wk | A1, A2 |
| **P2** | Control firmware on F28379D, model on CPU2, protection state machine | 3 wk | A4, A5, A6 |
| **P3** | HMI, telemetry, logging, provenance badging | 2 wk | A7 |
| **P4** | Panel build, safety, key interlocks | 2 wk | A8 |
| **P5** | Lab manual authoring, dry run, documentation | 2 wk | A10 |
| **P6** | *(later)* Real converter integration | — | A9 in hardware |

Critical path: encoder procurement (P1) and model validation (P0) run in parallel. Total ~13 weeks to shippable v1.

---

## 14. Open questions — close before BOM freeze

1. **Which TI inverter board is in hand?** LAUNCHXL-F28379D + BOOSTXL-DRV8305? A TIDA reference design? This determines whether §6.1 is a procurement item or a shelf item, and whether the gate-pulse test points in §7.1 are usable as-is.
2. **Rotor termination decision:** open (guarded) or resistor bank? Recommendation is **resistor bank**, because it produces real torque, makes T_e validatable, and removes the open-terminal hazard. Costs ~1 week and one BOM item.
3. **Stator voltage for standard exercises:** confirm 110–150 V via variac as the default. Anything else requires re-doing the §9.2 risk assessment.
4. **Does the existing DC drive have an analog reference input?** If not, exercise 8 (turbine emulation) is not possible and the drive must be replaced. This is the single highest-impact unknown.
5. **Does this SKU replace the full-power trainer or sit alongside it?** Affects pricing, positioning and whether the existing manual is retired or retained.
6. **Encoder lead time** — 1024 PPR, hollow bore. Confirm availability before committing to the 13-week schedule.
7. **Model on CPU2 or on the SBC in the shipped unit?** SBC is faster to build; CPU2 is the better product and the better teaching story. Recommendation: SBC for the prototype, CPU2 for production.

---

## 15. Risks

| Risk | Impact | Likelihood | Mitigation |
|---|---|---|---|
| Model diverges from reality; students lose trust | High | Medium | Exercise 3 makes validation an explicit, graded activity — turn the risk into a learning outcome |
| Perceived as "fake equipment" by faculty | High | Medium | §7.4 disclosure; position as HIL, an *advanced* category; lead the demo with exercises 1–2, which are purely physical |
| Slip-ring open-circuit voltage incident | Severe | Low | §9.2 (a) + (c) mandatory, (b) recommended |
| CPU2 model port overruns schedule | Medium | Medium | SBC fallback already architected via `plant_iface` |
| DC drive lacks analog input | Medium | Medium | Open question 4 — resolve in week 1 |
| Phase-2 swap turns out harder than scoped | Medium | Low | A9 demonstrated on the bench during P2, not deferred |

---

## Appendix A — Signal list

| Signal | Type | Range | Sensor | ADC ch | Rate | Provenance |
|---|---|---|---|---|---|---|
| N_shaft | Speed | 0–2000 RPM | 1024 PPR encoder | eQEP1 | 10 kHz | M |
| v_s_ab, v_s_bc | Voltage | ±650 V | LV 25-P | A0, A1 | 10 kHz | M |
| i_s_a, i_s_b, i_s_c | Current | ±5 A | LA 25-NP | A2–A4 | 10 kHz | M |
| v_r_ab, v_r_bc | Voltage | ±900 V | LV 25-P | B0, B1 | 10 kHz | M |
| V_dc_motor, I_dc_motor | DC | 0–250 V, 0–12 A | Shunt + isolator | B2, B3 | 1 kHz | M |
| i_rd, i_rq | Model state | ±5 A | — | — | 10 kHz | S |
| P_s, Q_s, T_e, P_r, V_dc | Model output | — | — | — | 10 kHz | S |
| τ_ref → DC drive | Command | 0–10 V | — | DAC-A | 200 Hz | Model output → real actuator |
| Scope ch 1–3 | User-selected | 0–3 V | — | DAC-A/B/C | 10 kHz | Per selection |

## Appendix B — Telemetry frame

100 Hz, binary, little-endian, over SCI @ 921600 or USB CDC.

```
uint16  sync          0xA55A
uint16  seq
uint32  timestamp_us
float   n_shaft, v_s_rms, f_s, slip           /* measured  */
float   i_s_rms, v_r_rms                       /* measured  */
float   i_rd, i_rq, i_rd_ref, i_rq_ref         /* simulated */
float   P_s, Q_s, T_e, P_r, v_dc               /* simulated */
float   v_wind, cp, lambda                     /* simulated */
uint16  state, fault_flags, src_flags
uint16  overrun_count
uint16  crc16
```

`src_flags` bit n = 1 means field n is simulated. The HMI badge renders directly from this — provenance is asserted by the firmware, not by the UI.

---

*Specifications are indicative and subject to component availability and supply-chain constraints. Functionally equivalent or superior substitutions may be made in consultation with the customer.*
