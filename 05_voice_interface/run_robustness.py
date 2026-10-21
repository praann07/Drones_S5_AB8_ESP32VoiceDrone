"""Closed-loop robustness sweep: how much sensor noise, IMU bias, wind and
payload mismatch can the cascade tolerate before a `forward` leg misses
its waypoint?

Stage C's noise ablation only stressed the IDENTIFICATION (does SINDy
still fit if the training omega is noisy). Every flight in the gauntlet
and demo used ideal sensors, no wind and the exact identified mass, so
nothing yet said how the CONTROLLER behaves off the happy path. This
sweeps each disturbance on its own, from a clean hover, and reports the
level at which the leg stops meeting the 0.15 m tolerance - failures are
reported, not hidden.

Each level is flown twice: robust=False is the original gated cascade,
robust=True adds integral action + yaw-priority desaturation.

Pass  = final truth position error < POSITION_TOLERANCE_M and no divergence.
Diverged = tilt > 60 deg or position error > 2 m at any step.

Run: .venv\\Scripts\\python.exe 05_voice_interface\\run_robustness.py
Writes data/processed/robustness_plots/robustness_results.csv + robustness.png
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
for _p in ("05_voice_interface", "01_simulation", "01_simulation/models",
           "04_control", "02_identification", "03_validation"):
    sys.path.insert(0, str(ROOT / _p))

from flight import DT, FlightController, IDENTITY_Q  # noqa: E402
from trajectory import POSITION_TOLERANCE_M  # noqa: E402

OUT = ROOT / "data" / "processed" / "robustness_plots"
FLIGHT_S = 8.0
SEEDS = (0, 1, 2)
DIVERGE_TILT_DEG = 60.0
DIVERGE_ERR_M = 2.0


def fly(noise=None, seed=0, wind=None, wind_fn=None, payload=1.0, robust=True) -> dict:
    fc = FlightController(sensor_noise=noise, seed=seed, robust=robust)
    if payload != 1.0:
        fc.set_payload_scale(payload)
    if wind is not None:
        fc.wind_force = np.asarray(wind, dtype=float)
    fc.wind_fn = wind_fn
    target = fc.dispatch("forward")["target"]
    max_tilt, diverged = 0.0, False
    for _ in range(int(round(FLIGHT_S / DT))):
        fc.step()
        tilt = fc.attitude_error_deg(IDENTITY_Q)
        err = float(np.linalg.norm(fc.position() - target))
        max_tilt = max(max_tilt, tilt)
        if tilt > DIVERGE_TILT_DEG or err > DIVERGE_ERR_M or not np.isfinite(err):
            diverged = True
            break
    final_err = float(np.linalg.norm(fc.position() - target))
    return {"final_err_m": final_err, "max_tilt_deg": max_tilt,
            "diverged": diverged,
            "pass": (not diverged) and final_err < POSITION_TOLERANCE_M}


def sweep(name, unit, levels, build, seeds=(0,)):
    """Each level is flown with the original gated cascade (robust=False)
    and with integral action + yaw-priority desaturation (robust=True)."""
    rows = []
    for lv in levels:
        row = {"disturbance": name, "unit": unit, "level": lv, "n": len(seeds)}
        for tag, robust in (("base", False), ("robust", True)):
            rs = [fly(seed=s, robust=robust, **build(lv)) for s in seeds]
            row[f"err_{tag}_m"] = float(np.mean([r["final_err_m"] for r in rs]))
            row[f"tilt_{tag}_deg"] = float(np.max([r["max_tilt_deg"] for r in rs]))
            row[f"pass_{tag}"] = float(np.mean([r["pass"] for r in rs]))
        rows.append(row)
        print(f"  {name:<14s} {lv:>7g} {unit:<6s} "
              f"base err={row['err_base_m']:.4f} m pass={row['pass_base']:.0%} | "
              f"robust err={row['err_robust_m']:.4f} m pass={row['pass_robust']:.0%} ({len(seeds)})",
              flush=True)
    return rows


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    print("Nominal (ideal sensors, no disturbance) ...")
    for robust in (False, True):
        b = fly(robust=robust)
        print(f"  robust={robust}: err={b['final_err_m']:.4f} m  pass={b['pass']}")

    rows += sweep("gyro noise", "rad/s", [0.0, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0],
                  lambda x: {"noise": {"gyro_std": x}}, SEEDS)
    rows += sweep("gyro bias", "rad/s", [0.0, 0.01, 0.05, 0.1, 0.2, 0.5],
                  lambda x: {"noise": {"gyro_bias": [x, 0.0, 0.0]}})
    rows += sweep("attitude noise", "rad", [0.0, 0.002, 0.01, 0.02, 0.05, 0.1],
                  lambda x: {"noise": {"att_std": x}}, SEEDS)
    rows += sweep("position noise", "m", [0.0, 0.005, 0.02, 0.05, 0.1],
                  lambda x: {"noise": {"pos_std": x}}, SEEDS)
    rows += sweep("constant wind", "N", [0.0, 0.25, 0.5, 1.0, 2.0, 3.0],
                  lambda x: {"wind": [0.0, x, 0.0]})
    rows += sweep("wind gust", "N", [0.0, 1.0, 2.0, 4.0],
                  lambda x: {"wind_fn": (lambda t, x=x:
                             np.array([0.0, x * np.exp(-((t - 2.0) / 0.4) ** 2), 0.0]))})
    rows += sweep("payload scale", "x", [0.8, 0.9, 1.0, 1.1, 1.2, 1.5],
                  lambda x: {"payload": x})

    df = pd.DataFrame(rows)
    df.to_csv(OUT / "robustness_results.csv", index=False)

    names = list(dict.fromkeys(df["disturbance"]))
    fig, axes = plt.subplots(2, 4, figsize=(18, 7))
    for ax, nm in zip(axes.flat, names):
        d = df[df["disturbance"] == nm]
        ax.plot(d["level"], d["err_base_m"], "o--", color="tab:gray", label="original cascade")
        ax.plot(d["level"], d["err_robust_m"], "o-", color="tab:green", label="+ integral + yaw-priority")
        ax.axhline(POSITION_TOLERANCE_M, color="r", ls=":", lw=1, label="0.15 m tolerance")
        ax.set_title(nm); ax.set_xlabel(d["unit"].iloc[0]); ax.set_ylabel("final error (m)")
        ax.set_yscale("log"); ax.set_ylim(1e-4, 3.0); ax.grid(alpha=0.3, which="both")
    axes.flat[0].legend()
    for ax in list(axes.flat)[len(names):]:
        ax.axis("off")
    fig.suptitle("Closed-loop robustness: forward leg, final position error vs disturbance")
    fig.tight_layout()
    fig.savefig(OUT / "robustness.png", dpi=110)
    print(f"\nWrote {OUT / 'robustness_results.csv'} and robustness.png")


if __name__ == "__main__":
    main()
