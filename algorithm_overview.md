# Algorithm Overview — Hybrid EKF + Coulomb Counting SOC Estimator

## Why Hybrid?

Neither Coulomb Counting nor EKF alone is sufficient for a commercial EV:

| Method | Strength | Weakness |
|--------|----------|----------|
| Coulomb Counting | Simple, fast, works in plateau | Accumulates error; needs accurate Q and I |
| EKF (voltage-based) | Self-correcting; no drift | Blind in LFP plateau (dOCV/dSOC ≈ 0); needs accurate OCV |
| **Hybrid (this work)** | Corrects drift + accurate in plateau | Needs tuned fusion weight |

LFP chemistry makes EKF-only estimation unusually hard. The OCV is nearly flat between ~20–90% SOC (dV ≈ 40 mV across 70% SOC range on a single cell). The voltage innovation `Vt − V_pred` carries almost no SOC signal in this range — the Kalman gain collapses, and the EKF essentially degenerates to Coulomb Counting anyway. The hybrid makes this explicit and controllable.

## State Space Model

1RC Thevenin equivalent circuit:

**State vector:** `x = [SOC, V1]ᵀ`

**Discrete state equations (Euler, dt = 0.1s):**
```
x(k+1) = A·x(k) + B·u(k)

A = [1,           0        ]
    [0,  1 - dt/(R1·C1)   ]

B = [dt/Q_capacity]
    [dt/C1        ]
```

**Output equation:**
```
Vt(k) = OCV(SOC(k), T(k)) + I(k)·R0 + V1(k)
```

**Measurement Jacobian:**
```
H = [dOCV/dSOC,  1]
```

## EKF Steps

**Predict:**
```
x̂⁻(k) = A·x̂(k-1) + B·u(k)
P⁻(k)  = A·P(k-1)·Aᵀ + Q
```

**Update:**
```
S   = H·P⁻·Hᵀ + R
K   = P⁻·Hᵀ·S⁻¹
x̂   = x̂⁻ + K·(Vt - V_pred)
P   = (I - K·H)·P⁻
```

## OCV Table Design

- Source: datasheet Table 3-2-1 (cell-level, 5 temperatures)
- Scale: `OCV_pack = OCV_cell × n_series`  ← **n_series must be verified from data**
- n_series detection: `round(V_pack_rest / V_cell_avg_rest)`
- Interpolation: `interp2(temp_bp, soc_bp, ocv_pack, T, SOC, 'linear')`
- Extrapolation: clamped (never extrapolate outside [0,1] SOC)

## Capacity Estimation

Do not use nameplate capacity blindly. Compute from data:

```matlab
net_Ah  = -sum(I) * dt / 3600;       % integrated discharge
dSOC    = SOC_true(1) - SOC_true(end);
Q_Ah    = net_Ah / dSOC;             % actual usable capacity
```

This implicitly captures SOH degradation. Updating this periodically (per full cycle) gives a simple SOH tracker.

## Noise Tuning

**Q matrix (process noise):**
- `Q(1,1)` = SOC uncertainty per step from current sensor noise
  - For 0.1A noise at dt=0.1s: `σ_soc = (0.1×0.1)/(Q_Ah×3600) ≈ 3e-8` → `Q(1,1) ≈ 1e-7`
- `Q(2,2)` = V1 uncertainty; larger value = filter tracks fast RC dynamics

**R (measurement noise):**
- Active slope: ~0.5V RMS sensor noise → `R ≈ 0.25`; use `1.0` for safety margin
- Plateau: inflate to 25 so CC dominates — voltage gives no SOC information here

## Known Limitations

1. **Initial SOC uncertainty**: Pack may not be at equilibrium at t=0 (cells charged and not rested → OCV inflated). A proper initialisation uses the first genuine rest window.
2. **Single RC branch**: A 2RC model would better capture diffusion at high C-rates. For bus discharge (typically < 1C), 1RC is sufficient.
3. **Temperature averaging**: Using `(Tmax + Tmin)/2` is an approximation. For wide spread (>10°C delta), use the weakest cell temperature.
4. **No current sensor offset correction**: A production implementation should estimate and subtract the current sensor DC bias using rest periods.
