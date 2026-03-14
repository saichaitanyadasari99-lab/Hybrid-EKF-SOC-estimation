# Simulink Implementation Guide
## Hybrid EKF + Coulomb Counting SOC Estimator

This document maps every section of `ekf_cc_hybrid_v3.m` to a concrete Simulink block architecture.  
Target toolboxes: **Simulink**, **Stateflow**, **Simscape Electrical** (optional), **Embedded Coder** (for code gen).

---

## Recommended Model Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                    SOC_Estimator.slx  (top level)             │
│                                                               │
│  Inports:  I_pack, V_pack, T_avg                              │
│  Outports: SOC_estimated, SOC_cc, SOC_ekf, V1_state           │
│                                                               │
│  ┌──────────────┐  ┌─────────────────┐  ┌──────────────────┐ │
│  │  CC_Subsystem│  │  EKF_Subsystem  │  │  Fusion_Subsystem│ │
│  └──────────────┘  └─────────────────┘  └──────────────────┘ │
└───────────────────────────────────────────────────────────────┘
```

Use **Atomic Subsystems** for each block so they can be unit-tested independently and exported to C via Embedded Coder.

---

## Block 1 — Coulomb Counting Subsystem

**Inputs:** `I` [A], `dt` [s]  
**Output:** `SOC_cc`  
**State:** `SOC_cc` (persists between steps → use Unit Delay)

```
Simulink blocks:
  I ──► [Gain: dt/Q_capacity] ──► [Sum: +] ──► [Unit Delay (IC = SOC_init)] ──► SOC_cc
                                       ▲
                                  feedback loop
```

**Key settings:**
- `Q_capacity` = from data (`net_Ah / ΔSOC × 3600`) — set as workspace variable `Q_cap`
- Unit Delay initial condition: `SOC_init` (from workspace)
- Add **Saturation block** (0 to 1) after Unit Delay
- Sample time: match your BMS CAN message rate (0.1s in this dataset)

**Stateflow alternative:** Not needed here — pure Simulink is sufficient for CC.

---

## Block 2 — OCV Lookup (2D Table)

**Inputs:** `SOC_pred`, `T_avg`  
**Output:** `OCV`  
**Block:** `Lookup Table (2-D)` from Simulink / Signal Routing

```
SOC_pred ──► [Prelookup SOC] ──┐
                               ├──► [Interpolation (2D)] ──► OCV
T_avg    ──► [Prelookup T]   ──┘
```

**Settings:**
- Breakpoints 1 (SOC): `[0 0.05 0.10 ... 1.0]`
- Breakpoints 2 (T):   `[0 10 25 45 60]`
- Table data: `ocv_pack` matrix (21×5), scaled for `n_series`
- Interpolation method: Linear
- Extrapolation method: Clamp (never extrapolate outside 0–100% SOC)

**Tip:** Right-click the 2D Lookup block → "View table" to visually verify the LFP plateau shape before running.

---

## Block 3 — EKF Subsystem (MATLAB Function block)

This is cleanest as a single **MATLAB Function block** inside an Atomic Subsystem.  
It preserves the persistent state variables `x` and `P` cleanly.

```matlab
% Inside MATLAB Function block
function [SOC_ekf, V1] = ekf_update(I, Vt, T, OCV, dOCV, R0, R1, C1, dt, Qk, Rk_eff)

persistent x P
if isempty(x)
    x = [0.98; 0];          % initial SOC, V1 — override from workspace in real use
    P = diag([1e-4, 1e-4]);
end

%% Prediction
A = [1, 0; 0, 1 - dt/(R1*C1)];
x_pred = [x(1) + (I*dt)/Q_cap;
          x(2) + dt*(-x(2)/(R1*C1) + I/C1)];
P = A*P*A' + Qk;

%% Update
H      = [dOCV, 1];
V_pred = OCV + I*R0 + x_pred(2);
S      = H*P*H' + Rk_eff;
K      = P*H'/S;
x      = x_pred + K*(Vt - V_pred);
x(1)   = max(0, min(1, x(1)));
P      = (eye(2) - K*H)*P;

SOC_ekf = x(1);
V1      = x(2);
```

**Why MATLAB Function block and not pure Simulink?**  
The matrix operations (`P*H'`, Kalman gain) are cumbersome with pure Simulink blocks. MATLAB Function blocks generate the same C code via Embedded Coder, so there's no penalty for code generation.

**Important for code generation:**
- All persistent variables must be declared with `persistent`
- Matrix sizes must be fixed (no dynamic sizing)
- Avoid `interp2` inside — pass in `OCV` and `dOCV` from the lookup block outside

---

## Block 4 — dOCV/dSOC Computation

Compute the OCV gradient separately (needed for H Jacobian and adaptive Rk switching).

```
SOC_pred ──► [Lookup Table 1D: dOCV_vs_SOC @ T=T_avg]  ──► dOCV
```

Use a **1D lookup table** per temperature column, or a second **2D lookup table** with the pre-computed gradient matrix `dOCV_matrix`.

**Why not numerical difference inside EKF?**  
Calling `interp2` twice per step inside a MATLAB Function block adds two more table lookups per timestep. Cleaner to pre-compute once and pass it as an input signal.

---

## Block 5 — Adaptive Rk Switch

**Input:** `dOCV`  
**Output:** `Rk_eff`

```
dOCV ──► [Abs] ──► [Compare: < 1.0] ──► [Switch]
                                            │── true  (plateau): Rk_plateau = 25
                                            └── false (slope):   Rk_active  = 1
```

Use a **Switch block** with threshold 1.0 on `|dOCV|`.

---

## Block 6 — CC / EKF Fusion

**Inputs:** `SOC_cc`, `SOC_ekf`  
**Output:** `SOC_est`

```
SOC_cc  ──► [Gain: CC_WEIGHT]  ──┐
                                  ├──► [Sum] ──► Saturation(0,1) ──► SOC_est
SOC_ekf ──► [Gain: 1-CC_WEIGHT]──┘
```

For the adaptive version (dynamic CC weight based on `|dOCV|`):

```
dOCV ──► [Abs] ──► [Gain: 1/10] ──► [MinMax: min with 0.9] ──► [MaxMin: max with 0.1]
       ──► cc_weight
```

---

## Top-Level Wiring

```
I     ──►─────────────────────────────►── CC_Subsystem ──► SOC_cc ──►─┐
      │                                                                │
      ├──►─────────────────────────────►── EKF_Subsystem ──► SOC_ekf ─┤──► Fusion ──► SOC_est
      │                               ▲                               │
Vt   ──►───────────────────────────────┤                               │
T    ──►── OCV_Lookup ──► OCV ─────────┤                               │
           dOCV_Lookup ──► dOCV ───────┤──► Adaptive_Rk ──► Rk_eff ──►┘
```

---

## Simulation Settings

| Setting | Value | Reason |
|---------|-------|--------|
| Solver | Fixed-step, Discrete | BMS runs on fixed-rate interrupt |
| Step size | 0.1 s | Match CAN message rate in this dataset |
| Stop time | `length(time)*dt` | Full discharge cycle |

**Workspace variables to define before simulation:**
```matlab
SOC_init   = 0.98;
Q_cap      = 88.7 * 3600;   % from data
n_series   = 108;
R0         = 0.06;           % from optimization
R1         = 0.04;
C1         = 1200;
CC_WEIGHT  = 0.70;
Rk_active  = 1.0;
Rk_plateau = 25.0;
Qk         = diag([1e-7, 1e-4]);
```

---

## Stateflow: SOC State Manager (Optional but Recommended)

A Stateflow chart can manage operating modes and guard against invalid states:

```
States:
  INIT        → wait for valid current/voltage signal
  CHARGING    → I > 0 threshold; switch OCV bias correction if needed
  DISCHARGING → I < 0; main EKF+CC active
  REST        → |I| < threshold; use voltage for SOC correction (OCV reset)
  FAULT       → SOC < 0.05 or V < V_cutoff; freeze SOC, raise flag

Transitions:
  INIT → DISCHARGING     [I < -2A for 5 consecutive samples]
  DISCHARGING → REST     [|I| < 1A for 30 consecutive samples]
  REST → DISCHARGING     [I < -2A]
  REST → CHARGING        [I > 2A]
  any → FAULT            [Vt < V_min or Vt > V_max]
```

The REST state is particularly important — during a genuine rest (>30s), the terminal voltage converges toward OCV. You can use this as a **hard SOC reset**: look up the rested voltage in the OCV table and override the EKF state. This eliminates cumulative CC drift on long trips.

---

## Code Generation with Embedded Coder

To generate deployable C code:

1. Mark the top-level model as an **Atomic Subsystem** with a function interface
2. Set **System Target File** to `ert.tlc` (Embedded Real-Time)
3. In the MATLAB Function block, ensure:
   - No dynamic memory allocation
   - All arrays are fixed-size
   - No `persistent` variables accessed outside the function (use `State` ports instead for code gen compatibility)
4. Run **Code Generation Advisor** → fix all errors flagged under "Efficiency"
5. Generated code will produce a single `soc_estimator.c` + `soc_estimator.h`

**Tip:** For AUTOSAR compatibility (if targeting a production BMS MCU), use the AUTOSAR System Target File and map each subsystem to an AUTOSAR Software Component (SwC).

---

## Validation Workflow

```
1. Run MATLAB script (ekf_cc_hybrid_v3.m) → get reference SOC trace
2. Feed same CSV data into Simulink model via "From Workspace" block
3. Compare SOC_est from both → difference should be < 0.01% (numerical only)
4. If mismatch: check Unit Delay initial conditions and dt alignment
5. SIL test: generate C code, compile, run on same input → compare to MATLAB output
6. HIL test: deploy to dSPACE/Speedgoat, inject real CAN data, log output
```

---

## File Checklist for Simulink Submission

```
simulink/
├── README_simulink.md          ← this file
├── SOC_Estimator.slx           ← main model (TODO: build from this guide)
├── SOC_Estimator_init.m        ← workspace init script (run before simulation)
├── validate_vs_matlab.m        ← comparison script (Simulink vs MATLAB output)
└── codegen/
    ├── soc_estimator.c         ← generated after Embedded Coder run
    └── soc_estimator.h
```
