# RESULTS — Voice-Controlled Quadrotor Navigation System

**Run date:** 2026-10-19 | **Platform:** Windows / Python 3.13 / MuJoCo  
**Test suite:** 96/96 passed | **Gauntlet:** 67/67 PASS (100%)

---

## Pipeline Execution Summary

| Stage | Command | Result |
|---|---|:---:|
| **Tests** | `pytest tests/` | ✅ 96/96 PASS |
| **Stage A** — Excitation | `run_excitation.py` | ✅ PASS |
| **Stage B** — Identification | `run_identification.py` | ✅ PASS |
| **Stage C** — Validation | `run_validation.py` | ✅ PASS |
| **Stage D Inner** — Attitude loop | `run_inner_loop.py` | ✅ PASS |
| **Stage D Cascade** — Full cascade | `run_cascade.py` | ✅ PASS |
| **Stage E Trials** — 24 isolated + 5 chained | `run_trials.py` | ✅ PASS |
| **Stage E Preemption** — Mid-flight override | `run_preemption.py` | ✅ PASS |
| **Render Nav GIF** — Offscreen 3D city tour | `render_offscreen.py` | ✅ PASS |
| **Render Preempt GIF** — Mid-flight reversal | `render_offscreen.py --preempt` | ✅ PASS |
| **Maneuver Gauntlet** — 67 maneuvers | `run_gauntlet.py` | ✅ 67/67 PASS |

---

## Key Numbers Table

| Metric | Value |
|---|---|
| **SINDy held-out NRMSE** | 0.0786 |
| **SINDy held-out R²** | 0.9946 |
| **DMDc held-out NRMSE** | 0.0825 |
| **DMDc held-out R²** | 0.9926 |
| **SINDy active terms** | 44 / 156 |
| **Stage C diverged rollouts** | 0 / 7 (SINDy full envelope) |
| **Stage D inner settling time** | 0.56 – 0.86 s |
| **Stage D cascade settling time** | ~1.868 s |
| **Stage D cascade final error** | 0.0006 m |
| **Stage E isolated trials** | 24 / 24 (100%) |
| **Stage E chained legs** | 5 / 5 (100%), final error 0.0000 m, max 0.022 m |
| **Preemption peak excursion** | 0.259 m (tower face at 2.30 m → 2.041 m clearance) |
| **Preemption reversal latency** | 0.422 s (tol 0.5 s) |
| **Latency: classify** | 0.421 ms |
| **Latency: trajectory gen** | 34.797 ms (quintic Bezier + collision check) |
| **Latency: control dispatch** | 0.860 ms |
| **Latency: total** | 36.078 ms |
| **Robustness (see README)** | gyro noise to 2 rad/s, 1 N wind, ±20% payload: all < 0.06 m error (original cascade failed each) |
| **Autonomous patrol (see README)** | avoiding: 5/5 dev seeds + 29/30 never-seen seeds collision-free; straight-line baseline 0/5 (min clearance -0.757 m) |

---

## Maneuver Gauntlet Results

### Section 1 — Single Moves (8 commands × 3 runs = 24 trials)

| Command | Mean Err (m) | Settle (s) | Overshoot (m) | Att Err (deg) | Pass |
|---|---|---|---|---|:---:|
| forward | 0.0216 | 2.95 | 0.022 | 0.08 | 3/3 ✅ |
| back    | 0.0216 | 2.95 | 0.022 | 0.08 | 3/3 ✅ |
| left    | 0.0216 | 2.95 | 0.022 | 0.08 | 3/3 ✅ |
| right   | 0.0216 | 2.95 | 0.022 | 0.08 | 3/3 ✅ |
| up      | 0.0166 | 1.752 | 0.026 | 0.00 | 3/3 ✅ |
| down    | 0.0166 | 1.752 | 0.026 | 0.00 | 3/3 ✅ |
| hover   | 0.0000 | 0.000 | 0.000 | 0.00 | 3/3 ✅ |
| stop    | 0.0000 | 0.000 | 0.000 | 0.00 | 3/3 ✅ |

**24/24 PASS** — tolerance 0.15 m, zero divergence, zero collisions.

### Section 2 — Chained Sequences (3 sequences × 2 runs = 38 legs)

| Sequence | Legs | Runs | All Pass |
|---|---|---|:---:|
| Square: fwd→left→back→right→hover | 5 | 2 | ✅ 10/10 |
| Stairs: up→fwd→up→fwd→down→back→hover | 7 | 2 | ✅ 14/14 |
| Full mix: left→up→right→down→fwd→back→hover | 7 | 2 | ✅ 14/14 |

**38/38 legs PASS** — no resets between commands, final error ≤ 0.0003 m on every sequence.

### Section 3 — Edge Cases (5/5 PASS)

| Edge Case | Result | Notes |
|---|---|---|
| Rapid-fire (fwd 0.3s → left 6s) | ✅ PASS | Final err 0.0217 m |
| Mid-move preemption (fwd 1s → back 6s) | ✅ PASS | Final err 0.0198 m |
| Stop mid-move then resume | ✅ PASS | Final err 0.0157 m |
| Down at lowest altitude (z=1.0→0.5m) | ✅ PASS | z_min=0.474 m, above floor |
| Up at highest altitude (z=1.0→1.5m) | ✅ PASS | z_max=1.526 m, no runaway |

### Gauntlet Total: **67/67 PASS (100%)**

---

## Visual Evidence

### 3D City Navigation GIF (Chase + Bird's-Eye)
`videos/navigation.gif` — forward→left→back→right→up→down→hover over 3D city

### Full Gauntlet GIF (All 8 Commands + Square Chain)
`videos/gauntlet_full.gif` — 780 frames, chase view left / bird's-eye right, every direction visible

### Command Preemption GIF
`videos/robustness_preemption.gif` — forward overridden by back mid-flight, 1.911 m clearance to tower

### 2D Telemetry
`plots_2d/telemetry_2d.png` — position error, attitude error, latency per trial  
`plots_2d/cascade_forward.png` — Stage D cascade step response  
`plots_2d/rollout_sindy_long.png` — 11s open-loop SINDy rollout  
`plots_2d/robustness_preemption.png` — x(t) vs counterfactual + reversal window

### 3D Plots
`plots_3d/flight_3d_trajectory.png` — isometric + top-down flight paths over city  
`plots_3d/rollout_sindy_long_3d_phase.png` — ωx/ωy/ωz phase portrait (GT vs SINDy)

---

## File Index

```
results/
├── plots_2d/          20 PNG files (stage A/C/D/E plots + preemption)
├── plots_3d/          3 PNG files (3D trajectory + phase portrait)
├── videos/            3 GIF files (navigation, preemption, gauntlet)
├── maneuvers/         67 CSV + 67 PNG per-maneuver + 3 summary CSVs + gauntlet_full.gif
├── data/              sindy_fitted_model.npz + 5 CSV/parquet logs (README.md explains each)
└── logs/              raw console output per pipeline stage (README.md explains each)
```

**`data/` and `logs/` are raw backing files, not meant to be read directly —
each has its own `README.md` explaining what's in it, if you need to check.
This file (`RESULTS.md`) is the actual human-readable summary.**
