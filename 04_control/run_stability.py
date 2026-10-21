"""Closed-loop stability analysis of the as-built cascade, from the
IDENTIFIED model (frozen SINDy artifact), checked against MuJoCo.

What it answers, all measured:
  1. Where are the closed-loop poles really? (continuous time, per axis,
     inner loop alone and inner+outer cascade)
  2. How fast must the controller run? The controller is digital (zero-
     order hold, ZOH): torque is computed from a sample and held for one
     period. Exact sampled-data spectral radius rho vs control rate, with
     and without one sample of computation delay (what real hardware has).
  3. How much gain margin is left at the 500 Hz rate actually used?
  4. Do the predictions hold on MuJoCo? (MuJoCo step response vs the
     linear prediction, and the predicted instability rate reproduced by
     holding the control for N physics steps).

Run for two gain sets: "fixed" (controller.gains_from_identified_model,
current) and "legacy" (the formula used until 2026-10-20, which left out
the 1/b factor, with its 1.5 s settling target) - so the finding that
motivated the fix stays reproducible.

Linearization (small angle, hover): qe_vec ~ e/2, so per axis
    e_dot = omega,  omega_dot = a*omega + b*tau,
    tau   = -(Kq/2)*e - Komega*omega
with a = identified drag term (A_sindy diagonal) and b = identified
control effectiveness (SINDy u-coefficients mapped through the mixer).

    .venv\\Scripts\\python.exe 04_control\\run_stability.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import pandas as pd
from scipy.linalg import expm

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "01_simulation"))
from controller import (DAMPING_RATIO, SETTLING_TIME_S,  # noqa: E402
                        QuaternionController, gains_from_identified_model)
from mixer import HOVER_THRUST_PER_ROTOR, mixer_matrix  # noqa: E402
from position_controller import GRAVITY, KD_POS, KP_POS, MASS_KG  # noqa: E402
from sim_driver import C_ROT, C_TRANS, MODEL_PATH  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parent.parent
FROZEN_MODEL_PATH = PROJECT_ROOT / "data" / "processed" / "sindy_fitted_model.npz"
OUT_DIR = PROJECT_ROOT / "data" / "processed" / "stability_plots"
CONTROL_RATE_HZ = 500.0
AXES = ("roll", "pitch", "yaw")
TRUE_INERTIA = np.array([0.003375, 0.003375, 0.006750])  # quad.xml diaginertia


# --- identified per-axis plant -------------------------------------------
def identified_plant() -> dict:
    """a_i (1/s) and b_i (rad/s^2 per N*m) per axis from the frozen SINDy
    artifact. b comes from the u1..u4 coefficients mapped through the
    inverse mixer, i.e. d(omega_dot)/d(tau) - nothing from ground truth."""
    # allow_pickle: our own artifact (03_validation freeze_selected_model),
    # never an external file - same reasoning as flight.load_frozen_sindy.
    d = np.load(FROZEN_MODEL_PATH, allow_pickle=True)
    names = [str(n) for n in d["feature_names"]]
    coef = d["coef"]
    u_cols = [names.index(f"u{k}") for k in range(1, 5)]
    B_u = coef[3:6, u_cols]                                  # omega_dot per rotor thrust
    B_tau = B_u @ np.linalg.inv(mixer_matrix())[:, 1:]       # omega_dot per body torque
    A = d["A_sindy"]
    return {"a": np.diag(A)[3:6].copy(), "b": np.diag(B_tau).copy(),
            "B_tau": B_tau, "B_u": B_u, "A_sindy": A}


def true_plant() -> dict:
    return {"a": -C_ROT / TRUE_INERTIA, "b": 1.0 / TRUE_INERTIA}


# --- continuous / sampled-data closed loops ------------------------------
def axis_ol(a: float, b: float) -> tuple[np.ndarray, np.ndarray]:
    return np.array([[0.0, 1.0], [0.0, a]]), np.array([[0.0], [b]])


def axis_gain(Kq: float, Ko: float) -> np.ndarray:
    return np.array([[Kq / 2.0, Ko]])


def cascade_ol(a: float, b: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Lateral x-axis via pitch: states [x, v, theta, omega], input tau.
    x_ddot = g*theta - (c_trans/m)*v. Returns (A, B, K-builder input map)."""
    A = np.array([[0, 1, 0, 0],
                  [0, -C_TRANS / MASS_KG, GRAVITY, 0],
                  [0, 0, 0, 1],
                  [0, 0, 0, a]], dtype=float)
    B = np.array([[0], [0], [0], [b]], dtype=float)
    return A, B


def cascade_gain(Kq: float, Ko: float) -> np.ndarray:
    """theta_d = (-Kp*x - Kd*v)/g;  tau = -(Kq/2)(theta - theta_d) - Ko*omega."""
    kq = Kq / 2.0
    return np.array([[kq * KP_POS / GRAVITY, kq * KD_POS / GRAVITY, kq, Ko]])


def zoh(A: np.ndarray, B: np.ndarray, T: float) -> tuple[np.ndarray, np.ndarray]:
    n, m = B.shape
    M = np.zeros((n + m, n + m))
    M[:n, :n], M[:n, n:] = A, B
    E = expm(M * T)
    return E[:n, :n], E[:n, n:]


def sampled_rho(A, B, K, T: float, delay: bool = False) -> float:
    """Spectral radius of the sampled closed loop. delay=True: the torque
    computed from sample k is applied during period k+1 (one-sample
    computation delay), state augmented with the held torque."""
    Phi, Gam = zoh(A, B, T)
    if not delay:
        Acl = Phi - Gam @ K
    else:
        n, m = B.shape
        Acl = np.zeros((n + m, n + m))
        Acl[:n, :n], Acl[:n, n:] = Phi, Gam
        Acl[n:, :n] = -K
    return float(np.max(np.abs(np.linalg.eigvals(Acl))))


def min_stable_rate(A, B, K, delay: bool, lo=1.0, hi=1e6) -> float:
    """Bisection (log space) on the control rate where rho crosses 1."""
    if sampled_rho(A, B, K, 1.0 / hi, delay) >= 1.0:
        return float("inf")
    for _ in range(80):
        mid = np.sqrt(lo * hi)
        if sampled_rho(A, B, K, 1.0 / mid, delay) < 1.0:
            hi = mid
        else:
            lo = mid
    return hi


def gain_margin(A, B, K, T: float, delay: bool) -> float:
    """Largest multiplier k on K keeping rho < 1 at period T."""
    lo, hi = 0.01, 1.0
    while sampled_rho(A, B, hi * K, T, delay) < 1.0 and hi < 1e4:
        lo, hi = hi, hi * 2.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if sampled_rho(A, B, mid * K, T, delay) < 1.0:
            lo = mid
        else:
            hi = mid
    return lo


def design_intent_poles(settling_s: float = SETTLING_TIME_S) -> np.ndarray:
    wn = 4.0 / (DAMPING_RATIO * settling_s)
    return np.roots([1.0, 2 * DAMPING_RATIO * wn, wn ** 2])


LEGACY_SETTLING_TIME_S = 1.5


def legacy_gains(A_sindy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The pre-2026-10-20 gain formula, in effect: Kq = 2*wn^2 and
    Komega = 2*zeta*wn + a, i.e. Kp/I and Kd/I used directly as torque
    gains (missing the 1/b = I factor). Kept only to reproduce the finding."""
    wn = 4.0 / (DAMPING_RATIO * LEGACY_SETTLING_TIME_S)
    a = np.diag(A_sindy)[3:6]
    return np.full(3, 2 * wn ** 2), np.maximum(2 * DAMPING_RATIO * wn + a, 0.0)


# --- MuJoCo checks -------------------------------------------------------
def mujoco_roll_response(Kq, Ko, hold_steps: int, physics_dt: float,
                          e0: float = 0.05, duration: float = 1.5) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Roll error from e0 rad, controller re-evaluated every `hold_steps`
    physics steps (control period = hold_steps*physics_dt). Returns
    (t, e, saturated) - saturated per physics step."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    model.opt.timestep = physics_dt
    data = mujoco.MjData(model)
    data.qpos[2] = 1.0
    data.qpos[3:7] = [np.cos(e0 / 2), np.sin(e0 / 2), 0, 0]
    mujoco.mj_forward(model, data)
    ctrl = QuaternionController(Kq, Ko)
    n = int(round(duration / physics_dt))
    t, e, sat = np.zeros(n), np.zeros(n), np.zeros(n, dtype=bool)
    q_d, u, s = np.array([1.0, 0, 0, 0]), np.full(4, HOVER_THRUST_PER_ROTOR), False
    for k in range(n):
        q, w = data.qpos[3:7].copy(), data.qvel[3:6].copy()
        if k % hold_steps == 0:
            u, s = ctrl.compute(q, w, q_d, np.zeros(3), 4 * HOVER_THRUST_PER_ROTOR)
        t[k], sat[k] = k * physics_dt, s
        e[k] = 2 * np.arctan2(q[1], q[0])
        if not np.isfinite(e[k]) or abs(e[k]) > 3.0:
            return t[:k + 1], e[:k + 1], sat[:k + 1]
        data.ctrl[:] = u
        data.qfrc_applied[3:6] = -C_ROT * data.qvel[3:6]
        mujoco.mj_step(model, data)
    return t, e, sat


def linear_prediction(a, b, Kq, Ko, t, e0=0.05) -> np.ndarray:
    """e(t) of the linear closed loop from e(0)=e0, omega(0)=0."""
    A, B = axis_ol(a, b)
    Acl = A - B @ axis_gain(Kq, Ko)
    return np.array([(expm(Acl * tk) @ np.array([e0, 0.0]))[0] for tk in t])


def unstable(e: np.ndarray, sat: np.ndarray) -> bool:
    """Linear instability here shows up as a SATURATION-BOUNDED limit
    cycle, not a blow-up: the fast mode grows until rotors clip at 0/6 N
    and then oscillates there, so |e| stays small. A stable loop from a
    0.05 rad offset never saturates at all, so the test is: still clipping
    in the second half of the run (or non-finite)."""
    if not np.all(np.isfinite(e)) or abs(e[-1]) > 3.0:
        return True
    return bool(np.mean(sat[len(sat) // 2:]) > 0.1)


# --- analysis ------------------------------------------------------------
def _metrics(A, B, K, T) -> dict:
    return {
        "rho_500Hz": sampled_rho(A, B, K, T, False),
        "rho_500Hz_delay": sampled_rho(A, B, K, T, True),
        "min_rate_hz": min_stable_rate(A, B, K, False),
        "min_rate_hz_delay": min_stable_rate(A, B, K, True),
        "gain_margin_500Hz": gain_margin(A, B, K, T, False),
        "gain_margin_500Hz_delay": gain_margin(A, B, K, T, True),
    }


def analyse() -> dict:
    idp, tp = identified_plant(), true_plant()
    gain_sets = {"fixed": gains_from_identified_model(idp["A_sindy"], idp["B_u"]),
                 "legacy": legacy_gains(idp["A_sindy"])}
    T = 1.0 / CONTROL_RATE_HZ
    rows = []
    for gname, (Kq, Ko) in gain_sets.items():
        for i, ax in enumerate(AXES):
            for src, P in (("identified", idp), ("truth", tp)):
                A, B = axis_ol(P["a"][i], P["b"][i])
                K = axis_gain(Kq[i], Ko[i])
                poles = np.linalg.eigvals(A - B @ K)
                wn2 = P["b"][i] * Kq[i] / 2
                c = -(P["a"][i] - P["b"][i] * Ko[i])
                rows.append({"gains": gname, "loop": "inner", "axis": ax, "plant": src,
                             "Kq": Kq[i], "Komega": Ko[i],
                             "a_per_s": P["a"][i], "b_per_Nm_s2": P["b"][i],
                             "pole_1": poles[0], "pole_2": poles[1],
                             "zeta": c / (2 * np.sqrt(wn2)), "wn_rad_s": np.sqrt(wn2),
                             **_metrics(A, B, K, T)})
            if ax == "pitch":
                for src, P in (("identified", idp), ("truth", tp)):
                    A, B = cascade_ol(P["a"][i], P["b"][i])
                    K = cascade_gain(Kq[i], Ko[i])
                    poles = np.sort_complex(np.linalg.eigvals(A - B @ K))
                    rows.append({"gains": gname, "loop": "cascade_x", "axis": ax, "plant": src,
                                 "Kq": Kq[i], "Komega": Ko[i],
                                 "a_per_s": P["a"][i], "b_per_Nm_s2": P["b"][i],
                                 "pole_1": poles[0], "pole_2": poles[-1],
                                 "zeta": np.nan, "wn_rad_s": np.nan,
                                 "poles_all": np.array2string(poles, precision=2),
                                 **_metrics(A, B, K, T)})
    return {"df": pd.DataFrame(rows), "gains": gain_sets, "idp": idp, "tp": tp}


SWEEP_HOLD_STEPS = (10, 20, 30, 36, 40, 42, 44, 45, 46, 47, 48, 50, 56, 66, 100, 200, 400)
SWEEP_DT = 5e-5


def mujoco_checks(res: dict) -> dict:
    idp, out = res["idp"], {}
    for gname, (Kq, Ko) in res["gains"].items():
        t, e, _ = mujoco_roll_response(Kq, Ko, hold_steps=1, physics_dt=0.002)
        pred = linear_prediction(idp["a"][0], idp["b"][0], Kq[0], Ko[0], t)
        sweep = []
        for N in SWEEP_HOLD_STEPS:
            _, ee, ss = mujoco_roll_response(Kq, Ko, hold_steps=N, physics_dt=SWEEP_DT, duration=1.0)
            sweep.append({"gains": gname, "control_rate_hz": 1.0 / (N * SWEEP_DT),
                          "unstable": unstable(ee, ss)})
        out[gname] = {"t": t, "e": e, "pred": pred,
                      "max_dev_rad": float(np.max(np.abs(e - pred))),
                      "sweep": pd.DataFrame(sweep)}
    return out


# --- plot ----------------------------------------------------------------
COLORS = {"fixed": "#1f77b4", "legacy": "#d62728"}


def plot(res: dict, chk: dict) -> Path:
    df, idp = res["df"], res["idp"]
    fig, axs = plt.subplots(2, 2, figsize=(13, 9))

    ax = axs[0, 0]
    for gname, marker in (("legacy", "o"), ("fixed", "s")):
        sub = df[(df.gains == gname) & (df.loop == "inner") & (df.plant == "identified")]
        for _, r in sub.iterrows():
            ps = np.array([r.pole_1, r.pole_2])
            ax.scatter(ps.real, ps.imag, marker=marker, s=55, c=COLORS[gname], alpha=0.8, zorder=3,
                       label=f"{gname} gains" if r.axis == "roll" else None)
    for ts, lab in ((LEGACY_SETTLING_TIME_S, "legacy design intent"), (SETTLING_TIME_S, "fixed design intent")):
        dp = design_intent_poles(ts)
        ax.scatter(dp.real, dp.imag, marker="x", s=90, c="k" if ts == SETTLING_TIME_S else "grey",
                   label=f"{lab} (Ts={ts} s, zeta={DAMPING_RATIO})")
    ax.set_xscale("symlog", linthresh=1.0)
    ax.axvline(0, c="grey", lw=0.8)
    ax.set_xlabel("Re(s) [1/s]  (symlog)")
    ax.set_ylabel("Im(s) [rad/s]")
    ax.set_title("Inner-loop poles (roll, pitch, yaw), identified model")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    ax = axs[0, 1]
    rates = np.logspace(1.0, 4, 220)
    for gname, (Kq, Ko) in res["gains"].items():
        A, B = axis_ol(idp["a"][0], idp["b"][0])
        K = axis_gain(Kq[0], Ko[0])
        ax.plot(rates, [sampled_rho(A, B, K, 1 / f) for f in rates], c=COLORS[gname], label=f"{gname}")
        ax.plot(rates, [sampled_rho(A, B, K, 1 / f, True) for f in rates], "--", c=COLORS[gname],
                label=f"{gname} + 1-sample delay")
        sw = chk[gname]["sweep"]
        y = 1.8 if gname == "legacy" else 0.3
        ax.scatter(sw.control_rate_hz, np.full(len(sw), y), c=np.where(sw.unstable, "r", "g"),
                   marker="s", s=22, edgecolors=COLORS[gname],
                   label=f"MuJoCo, {gname} (red=limit cycle, green=stable)")
    ax.axhline(1.0, c="k", lw=1)
    ax.axvline(CONTROL_RATE_HZ, c="purple", ls=":", label="500 Hz (used)")
    ax.set_xscale("log")
    ax.set_ylim(0, 2.5)
    ax.set_xlabel("control rate [Hz]")
    ax.set_ylabel("spectral radius rho (stable < 1)")
    ax.set_title("Roll axis: sampled-data stability vs control rate")
    ax.legend(fontsize=6.5)
    ax.grid(alpha=0.3, which="both")

    ax = axs[1, 0]
    for gname in ("legacy", "fixed"):
        c = chk[gname]
        ax.plot(c["t"], np.degrees(c["e"]), c=COLORS[gname], label=f"MuJoCo, {gname}")
        ax.plot(c["t"], np.degrees(c["pred"]), "--", c="k", lw=1)
    ax.plot([], [], "--", c="k", lw=1, label="linear prediction (identified model)")
    ax.set_title(f"Roll recovery from 2.9 deg: max |MuJoCo - prediction| = "
                 f"{np.degrees(chk['fixed']['max_dev_rad']):.3f} deg (fixed)")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("roll error [deg]")
    ax.legend()
    ax.grid(alpha=0.3)

    ax = axs[1, 1]
    x = np.arange(3)
    for k, (gname, col) in enumerate((("legacy", "gain_margin_500Hz"), ("legacy", "gain_margin_500Hz_delay"),
                                      ("fixed", "gain_margin_500Hz"), ("fixed", "gain_margin_500Hz_delay"))):
        sub = df[(df.gains == gname) & (df.loop == "inner") & (df.plant == "identified")]
        ax.bar(x + (k - 1.5) * 0.2, sub[col], 0.2, color=COLORS[gname],
               alpha=1.0 if "delay" not in col else 0.5,
               label=f"{gname}{' + 1-sample delay' if 'delay' in col else ''}")
    ax.axhline(1.0, c="k", lw=1)
    ax.set_yscale("log")
    ax.set_xticks(x, list(AXES))
    ax.set_ylabel("gain multiplier before instability (log)")
    ax.set_title("Gain margin at 500 Hz (below 1 = unstable)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis="y", which="both")

    fig.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / "stability_analysis.png"
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def main() -> None:
    res = analyse()
    chk = mujoco_checks(res)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    res["df"].to_csv(OUT_DIR / "stability_results.csv", index=False)
    pd.concat([c["sweep"] for c in chk.values()]).to_csv(OUT_DIR / "mujoco_rate_sweep.csv", index=False)
    path = plot(res, chk)
    pd.set_option("display.width", 240)
    cols = ["gains", "loop", "axis", "plant", "pole_1", "pole_2", "zeta", "rho_500Hz_delay",
            "min_rate_hz", "min_rate_hz_delay", "gain_margin_500Hz", "gain_margin_500Hz_delay"]
    print(res["df"][cols].to_string(index=False, float_format=lambda v: f"{v:.3g}"))
    for gname, (Kq, Ko) in res["gains"].items():
        sw = chk[gname]["sweep"]
        bad = sw[sw.unstable].control_rate_hz
        print(f"{gname}: Kq={np.round(Kq, 4)} Komega={np.round(Ko, 4)}  "
              f"MuJoCo vs linear max dev {np.degrees(chk[gname]['max_dev_rad']):.4f} deg  "
              f"MuJoCo highest unstable rate: {bad.max() if len(bad) else 'none'} Hz")
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
