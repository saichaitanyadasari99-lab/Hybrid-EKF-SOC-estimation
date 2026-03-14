# Hybrid EKF + Coulomb Counting SOC Estimator

> Real-world battery SOC estimation for commercial electric vehicles using a fused Extended Kalman Filter and Coulomb Counting algorithm, validated on live vehicle discharge data from a 108S LFP pack.

---

## Overview

This project implements a **production-grade hybrid SOC estimator** combining:

- **Extended Kalman Filter (EKF)** — corrects for sensor drift and initial SOC error using a first-order RC equivalent circuit model (Thevenin model) and a 2D temperature-dependent OCV lookup table
- **Coulomb Counting (CC)** — integrates signed current for real-time SOC tracking; dominates in the flat LFP plateau where voltage carries negligible SOC information
- **Adaptive fusion** — the CC weight increases in the plateau region where `dOCV/dSOC` is near zero; EKF voltage corrections dominate on the slopes

Developed and validated against field data from a **VECV commercial electric bus** (108S1P LFP, ~89 Ah usable capacity at 84% SOH).

---

## Results

| Metric | v1 (broken) | v2 (OCV fix) | v3 (full fix) |
|--------|------------|--------------|---------------|
| RMSE | — | 3.82% | < 1.5% |
| MAE  | — | 3.31% | < 1.2% |
| Positive bias | ~40% flat | +3–4% drift | < 1% |
| Runtime (optimization) | — | ~2 hrs ❌ | ~3 min ✅ |

> **Note:** The BMS reference SOC is an integer-resolution staircase (1% steps), not a ground truth. The EKF continuous estimate is arguably more physically accurate.

---

## Bug History — What Was Wrong and Why

This estimator was developed iteratively on real vehicle data. The diagnostic journey is documented here as it contains lessons relevant to any real-world BMS implementation.

### v1 Bugs Fixed (sign convention and OCV)
| # | Bug | Root Cause | Effect |
|---|-----|-----------|--------|
| 1 | Sign convention | Discharge current is **negative** in this dataset. `SOC -= u*dt/Q` was incrementing SOC during discharge | SOC increased to 1.0 and stuck |
| 2 | Voltage model sign | `V_pred = OCV - I*R0 - V1` predicted Vt **above** OCV during discharge — physically impossible | Every Kalman innovation was massively wrong |
| 3 | H Jacobian sign | `H = [dOCV, -1]` didn't match corrected voltage model | Kalman gain direction inverted |
| 4 | OCV table wrong | Table was from a different cell spec; values extracted from non-equilibrium stops | 17V pack-level error in OCV prediction |

### v2 → v3 Bugs Fixed (hardware configuration)
| # | Bug | Root Cause | Effect |
|---|-----|-----------|--------|
| A | n_series = 120, should be 108 | `pack_V / avg_cell_V = 397/3.683 = 107.8` | OCV table scaled 11% too high → systematic +3% SOC bias |
| B | Q = 105 Ah, actual ~89 Ah | Pack at ~84% SOH; CC underestimated discharge by 18% | SOC drifted high throughout |
| C | Optimization over 72k pts | `fminsearch` with `interp2` inside loop = ~10M calls | 2+ hr runtime, optimizer never converged, bad R0/R1/C1 |

**Key lesson:** Always verify `n_series` from `V_pack / V_cell_avg` before coding the OCV table. Always compute `Q_capacity` from `∫I·dt / ΔSOC` on real data before assuming nameplate.

---

## Repository Structure

```
hybrid-ekf-cc-soc-estimator/
│
├── matlab/
│   └── ekf_cc_hybrid_v3.m          # Production MATLAB script (final version)
│
├── simulink/
│   └── README_simulink.md           # Block-by-block Simulink architecture guide
│
├── docs/
│   ├── algorithm_overview.md        # EKF + CC theory and design decisions
│   └── ocv_table_notes.md           # OCV data sourcing and temperature effects
│
├── data/
│   └── README_data.md               # Data format description (raw CSV not included)
│
└── README.md
```

---

## Algorithm Architecture

```
                    ┌─────────────────────────────────────┐
                    │         INPUTS (per timestep)        │
                    │   I(k): pack current [A]             │
                    │   Vt(k): pack voltage [V]            │
                    │   T(k): avg cell temperature [°C]    │
                    └──────────────┬──────────────────────┘
                                   │
              ┌────────────────────┴────────────────────┐
              │                                         │
    ┌─────────▼──────────┐                  ┌──────────▼────────────┐
    │   COULOMB COUNTING  │                  │   EKF (Thevenin RC)   │
    │                     │                  │                       │
    │  SOC_cc += I·dt/Q   │                  │  Predict: x̂⁻ = A·x   │
    │                     │                  │  OCV = f(SOC, T) 2D   │
    │  Dominant in flat   │                  │  V_pred = OCV+I·R0+V1 │
    │  LFP plateau where  │                  │  Update: K, x, P      │
    │  dOCV/dSOC ≈ 0      │                  │                       │
    └─────────┬──────────┘                  └──────────┬────────────┘
              │                                         │
              └──────────────┬──────────────────────────┘
                             │
              ┌──────────────▼──────────────────┐
              │     ADAPTIVE FUSION              │
              │                                  │
              │  α = f(|dOCV/dSOC|)              │
              │  SOC = α·SOC_cc + (1-α)·SOC_ekf  │
              │                                  │
              │  Plateau: α → 0.9  (trust CC)    │
              │  Slope:   α → 0.3  (trust EKF)   │
              └──────────────┬──────────────────┘
                             │
              ┌──────────────▼──────────────────┐
              │        SOC_estimated(k)          │
              └─────────────────────────────────┘
```

---

## Battery Model

First-order Thevenin equivalent circuit (1RC):

```
    R0          R1
───┤├───┬───┤├────┤├───
   │    │              │
  OCV  C1            Vt(k)
   │    │              │
───────────────────────
```

**State vector:** `x = [SOC; V1]` where V1 is the voltage across the RC branch.

**State equations:**
```
SOC(k+1) = SOC(k) + (I(k)·dt) / Q
V1(k+1)  = V1(k) · (1 - dt/τ) + (I(k)·dt) / C1
Vt(k)    = OCV(SOC,T) + I(k)·R0 + V1(k)
```

**Sign convention (this dataset):** Discharge current is **negative**. Regen/charge is positive.

---

## Setup & Usage

### Requirements
- MATLAB R2021a or later
- No additional toolboxes required for the base script
- Optimization Toolbox (for `fminsearch`) — can be disabled with `RUN_OPTIMIZATION = false`

### Quick Start

```matlab
% 1. Place your CSV in the same folder as the script
% 2. Update the filename variable:
filename = 'your_data_file.csv';

% 3. Verify your CSV has these columns (names are auto-detected):
%    Time (abs), BMS_PackCurrent, BMS_PackVoltage, BMS_SOC,
%    BMS_MaxCellTemp, BMS_MinCellTemp,
%    BMS_PackMaxCellVoltage, BMS_PackMinCellVoltage

% 4. Run
ekf_cc_hybrid_v3

% 5. To skip the 3-min optimization and use fixed parameters:
RUN_OPTIMIZATION = false;
```

### Data Format

| Column | Unit | Notes |
|--------|------|-------|
| `Time (abs)` | s | Absolute time from start |
| `BMS_PackCurrent` | A | Negative = discharge |
| `BMS_PackVoltage` | V | Pack terminal voltage |
| `BMS_SOC` | % (0–100) | BMS reference (staircase) |
| `BMS_MaxCellTemp` | °C | Used for 2D OCV interpolation |
| `BMS_MinCellTemp` | °C | Averaged with max for T estimate |
| `BMS_PackMaxCellVoltage` | V | Used to detect n_series |
| `BMS_PackMinCellVoltage` | V | Used to detect n_series |

---

## Tuning Guide

| Parameter | Default | Effect | Tune if... |
|-----------|---------|--------|------------|
| `Q_capacity` | auto from data | CC rate | Persistent drift up/down |
| `n_series` | auto from data | OCV scale | Systematic constant bias |
| `CC_WEIGHT` | 0.70 | CC vs EKF blend | Noisy EKF → increase; CC drift → decrease |
| `Qk(1,1)` | 1e-7 | SOC process noise | Increase if EKF is too slow to correct |
| `Rk_active` | 1.0 | Measurement trust on slope | Increase if Kalman is over-correcting |
| `Rk_plateau` | 25.0 | Measurement trust in plateau | Increase for smoother plateau tracking |
| `SS_FACTOR` | 15 | Subsampling for optimization | Reduce for higher accuracy; increase for speed |

---

## Simulink Implementation

See [`simulink/README_simulink.md`](simulink/README_simulink.md) for the complete block-by-block architecture to replicate this algorithm in Simulink with Stateflow and Simscape Electrical.

---

## Roadmap

- [ ] Simulink model (`slx`) with Stateflow state machine
- [ ] SOH estimation via online resistance tracking (RLS)
- [ ] SOP estimation (power limit prediction)
- [ ] Multi-temperature OCV characterisation workflow
- [ ] Python port (NumPy/SciPy)

---

## Author

**Chaitanya** — Deputy Manager, Battery Systems Engineering  
Volvo Eicher Commercial Vehicles (VECV)  
M.Tech Design Engineering, BITS Pilani

---

## License

MIT License. See [LICENSE](LICENSE).
