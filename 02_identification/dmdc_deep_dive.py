"""Every number quoted in DMDC_DEEP_DIVE.md, computed from the project's data.

    .venv\\Scripts\\python.exe 02_identification\\dmdc_deep_dive.py

Read-only analysis: it fits nothing that is saved and writes no files. It
reuses the exact preprocessing and the exact DMDc fit of the Stage B gate
(run_identification.py / dmdc_fit.py), then adds the linear-systems analysis
those scripts do not do: SVD anatomy of the data matrix, parameter standard
errors, the physics prediction of A and B, the continuous-time model,
controllability, rollout error growth, the errors-in-variables bias and the
size of the terms a linear model leaves out. Section letters match the
document.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.linalg import expm, logm
from scipy.signal import savgol_coeffs

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from dmdc_fit import HOVER_THRUST_PER_ROTOR, DMDcModel  # noqa: E402
from preprocessing import CONTROL_COLS, DERIV_COLS, SG_POLYORDER, SG_WINDOW, STATE_COLS  # noqa: E402
from run_identification import DT, evaluate_one_step, load_processed, load_split  # noqa: E402

np.set_printoptions(precision=5, suppress=False, linewidth=140)

# Physical constants of the simulated drone (quad.xml, sim_driver.py)
IXX = IYY = 0.003375
IZZ = 0.00675
ARM = 0.106066          # rotor offset along x and y (m)
KM = 0.02               # yaw torque per newton of rotor thrust (m)
C_ROT = 0.008           # N m s / rad, torque = -C_ROT * omega


def banner(title: str) -> None:
    print("\n" + "=" * 78 + f"\n{title}\n" + "=" * 78)


def segments_of(frames: dict) -> list[tuple[np.ndarray, np.ndarray]]:
    return [(seg[STATE_COLS].to_numpy(), seg[CONTROL_COLS].to_numpy())
            for df in frames.values() for _, seg in df.groupby("segment_id", sort=True)
            if len(seg) >= 2]


def stack_pairs(segs, with_prev: bool = False):
    Z, Zp, dU, Zm = [], [], [], []
    for state, control in segs:
        if with_prev:
            if len(state) < 3:
                continue
            Zm.append(state[:-2].T); Z.append(state[1:-1].T); Zp.append(state[2:].T)
            dU.append((control[1:-1] - HOVER_THRUST_PER_ROTOR).T)
        else:
            Z.append(state[:-1].T); Zp.append(state[1:].T)
            dU.append((control[:-1] - HOVER_THRUST_PER_ROTOR).T)
    out = (np.hstack(Z), np.hstack(Zp), np.hstack(dU))
    return out + (np.hstack(Zm),) if with_prev else out


def nrmse(A, B, segs) -> float:
    err = chg = 0.0
    for state, control in segs:
        pred = state[:-1] @ A.T + (control[:-1] - HOVER_THRUST_PER_ROTOR) @ B.T
        err += np.sum((pred - state[1:]) ** 2)
        chg += np.sum((state[1:] - state[:-1]) ** 2)
    return float(np.sqrt(err / chg))


def main() -> None:
    split = load_split()
    ids = lambda reg, sp: split[(split.regime == reg) & (split.split == sp)].trial_id  # noqa: E731
    hover_fit = load_processed(ids("near_hover", "fit"))
    hover_held = load_processed(ids("near_hover", "held_out"))
    full_held = load_processed(ids("full_envelope", "held_out"))
    fit_segs, held_segs, full_segs = segments_of(hover_fit), segments_of(hover_held), segments_of(full_held)

    Z, Zp, dU = stack_pairs(fit_segs)
    n, m, N = Z.shape[0], dU.shape[0], Z.shape[1]
    Om = np.vstack([Z, dU])

    # ------------------------------------------------------------------ A
    banner("A. Data: sizes")
    print(f"near-hover fit trials {len(hover_fit)}, segments {len(fit_segs)}, snapshot pairs N = {N}")
    print(f"held-out near-hover trials {len(hover_held)}, segments {len(held_segs)}; "
          f"full-envelope held-out trials {len(full_held)}")
    print(f"n = {n} states, m = {m} inputs, Omega is {Om.shape}, dt = {DT} s")
    peak = max(np.linalg.norm(s[:, :3], axis=1).max() for s, _ in fit_segs)
    print(f"largest |theta| in the fit data: {np.degrees(peak):.2f} deg")
    print("per-column RMS of Z:", np.sqrt((Z ** 2).mean(axis=1)))
    print("per-column RMS of dU (N):", np.sqrt((dU ** 2).mean(axis=1)))

    # ------------------------------------------------------------------ B
    banner("B. SVD anatomy of Omega = [Z; dU]")
    U_, s_, Vt_ = np.linalg.svd(Om, full_matrices=False)
    print("singular values of Omega:", s_)
    print(f"sigma_max / sigma_min (condition number) = {s_[0] / s_[-1]:.3e}")
    for tol in (1e-6, 1e-8, 1e-12):
        print(f"  numerical rank at rtol {tol:g}: {int(np.sum(s_ > s_[0] * tol))}")
    sZ = np.linalg.svd(Z, compute_uv=False)
    sU = np.linalg.svd(dU, compute_uv=False)
    print("singular values of Z :", sZ, f"  cond {sZ[0] / sZ[-1]:.3e}")
    print("singular values of dU:", sU)
    null_vec = U_[m * 0 + n:, -1]            # input-block of the smallest left singular vector
    print("left singular vector of sigma_min (state block, input block):")
    print("   state :", U_[:n, -1])
    print("   input :", U_[n:, -1], " (uniform-thrust direction is +-0.5 each)")
    sum_du = dU.sum(axis=0)
    print(f"sum over rotors of dU: max |.| = {np.abs(sum_du).max():.3e} N, rms = {np.sqrt((sum_du ** 2).mean()):.3e} N")
    print(f"for scale, rms of one rotor's dU = {np.sqrt((dU ** 2).mean()):.4f} N")

    # the Stage B estimate and two truncations
    G_pinv = Zp @ np.linalg.pinv(Om)
    A_hat, B_hat = G_pinv[:, :n], G_pinv[:, n:]
    print("\nStage B estimate (np.linalg.pinv, default rcond):")
    print("A =\n", A_hat)
    print("B =\n", B_hat)
    ones = np.ones(m) / np.sqrt(m)
    print("component of B along the uniform-thrust direction, B @ (1/2)(1,1,1,1):", B_hat @ ones)
    P_perp = np.eye(m) - np.outer(ones, ones)
    B_perp = B_hat @ P_perp
    print("B with that direction removed (B @ P_perp) =\n", B_perp)

    def svd_solve(rtol):
        r = int(np.sum(s_ > s_[0] * rtol))
        G = Zp @ (Vt_[:r].T @ np.diag(1.0 / s_[:r]) @ U_[:, :r].T)
        return G, r
    for rtol in (1e-8, 1e-6):
        G_t, r = svd_solve(rtol)
        print(f"\ntruncated SVD solve, rtol {rtol:g}: rank {r}; max|A_trunc - A_pinv| = "
              f"{np.abs(G_t[:, :n] - A_hat).max():.3e}; max|B_trunc - B_perp| = {np.abs(G_t[:, n:] - B_perp).max():.3e}; "
              f"max|B_trunc| = {np.abs(G_t[:, n:]).max():.3e}")
    G_t, _ = svd_solve(1e-8)
    A_t, B_t = G_t[:, :n], G_t[:, n:]
    print("manual SVD solve with full rank vs np.linalg.pinv: max diff =",
          np.abs(Zp @ (Vt_.T @ np.diag(1.0 / s_) @ U_.T) - G_pinv).max())
    print(f"\none-step NRMSE held-out near hover : pinv {nrmse(A_hat, B_hat, held_segs):.5f}   "
          f"truncated {nrmse(A_t, B_t, held_segs):.5f}")
    print(f"one-step NRMSE held-out full envelope: pinv {nrmse(A_hat, B_hat, full_segs):.5f}   "
          f"truncated {nrmse(A_t, B_t, full_segs):.5f}")
    held_dU = np.hstack([(c[:-1] - HOVER_THRUST_PER_ROTOR).T for _, c in held_segs])
    print(f"held-out near hover: max |sum dU| = {np.abs(held_dU.sum(axis=0)).max():.3e} N")
    full_dU = np.hstack([(c[:-1] - HOVER_THRUST_PER_ROTOR).T for _, c in full_segs])
    print(f"held-out full envelope: max |sum dU| = {np.abs(full_dU.sum(axis=0)).max():.3e} N, "
          f"rms {np.sqrt((full_dU.sum(axis=0) ** 2).mean()):.3e} N")

    # the gate's own evaluation, for reconciling with the stored result
    gate = DMDcModel().fit(fit_segs)
    print("gate evaluate_one_step on held-out near hover:", evaluate_one_step(gate.step, hover_held)["step_nrmse"])
    import pandas as _pd
    print(_pd.read_csv(HERE.parent / "data" / "processed" / "identification_results.csv").to_string())
    # pydmd mismatch as recorded by the model itself
    model = DMDcModel().fit(fit_segs)
    print(f"\nDMDcModel.fit: pydmd mismatch {model.pydmd_mismatch:.3e} on a rank-{model.pydmd_check_rank} segment; "
          f"same A as above: {np.allclose(model.A, A_hat)}")

    # ------------------------------------------------------------------ C
    banner("C. Residuals and standard errors (Gauss-Markov / Gaussian MLE)")
    R = Zp - G_pinv @ Om
    s2 = (R ** 2).sum(axis=1) / (N - (n + m - 1))      # one dof lost to the unidentified direction
    print("row residual std (state units per step):", np.sqrt(s2))
    r_eff = int(np.sum(s_ > s_[0] * 1e-8))
    cov_diag = ((U_[:, :r_eff] ** 2) / s_[:r_eff] ** 2).sum(axis=1)    # diag of (Omega Omega^T)^+ = U S^-2 U^T
    print("sqrt diag (Omega Omega^T)^+ :", np.sqrt(cov_diag))
    idx_om = {"wx": 3, "wy": 4, "wz": 5}
    for name, row in idx_om.items():
        se = np.sqrt(s2[row] * cov_diag[row])
        print(f"  A[{name},{name}] = {A_hat[row, row]:.6f}  +- {se:.2e}")
    # torque columns of B in the rotor-difference basis
    T = np.array([[1, -1, -1, 1], [-1, 1, -1, 1], [1, 1, -1, -1]], dtype=float) / 2.0   # unit rows
    print("torque-channel gains: row of B projected on the unit torque directions (per N of difference thrust):")
    for name, row, tdir in (("wx", 3, T[0]), ("wy", 4, T[1]), ("wz", 5, T[2])):
        gain = float(B_perp[row] @ tdir)
        se = np.sqrt(s2[row] * float(tdir @ np.diag(cov_diag[n:]) @ tdir))
        print(f"  {name}: {gain:.6f} +- {se:.2e}")

    # ------------------------------------------------------------------ D
    banner("D. Physics prediction of A and B (exact zero-order-hold discretisation)")
    for ax, (I, row) in enumerate(((IXX, 3), (IYY, 4), (IZZ, 5))):
        a = C_ROT / I
        b = (ARM if ax < 2 else KM) / I * 2.0    # per unit of the rotor-difference direction (unit vector has 1/2 entries)
        b_per_N = (ARM if ax < 2 else KM) / I     # per newton on one rotor
        e = np.exp(-a * DT)
        Ad = np.array([[1.0, (1 - e) / a], [0.0, e]])
        Bd = b_per_N * np.array([DT / a - (1 - e) / a ** 2, (1 - e) / a])
        fit = np.array([[A_hat[ax, ax], A_hat[ax, row]], [A_hat[row, ax], A_hat[row, row]]])
        print(f"axis {'xyz'[ax]}: a = C/I = {a:.4f} 1/s ; b = L/I = {b_per_N:.3f} rad/s^2 per N")
        print(f"   theory A_d = {Ad.tolist()}")
        print(f"   fitted A_d = {fit.tolist()}")
        sign = {0: T[0], 1: T[1], 2: T[2]}[ax] * 2.0     # +-1 pattern of the mixer row
        fit_b = float(B_perp[row] @ sign) / 4.0 * 1.0 * 2.0 / 2.0
        gain_unit = float(B_perp[row] @ (sign / 2.0))     # response to unit-norm difference direction
        print(f"   theory B_d(omega row) per N on one rotor = {Bd[1]:.6f}; fitted |B| per rotor = "
              f"{np.abs(B_perp[row]).mean():.6f} (ratio {np.abs(B_perp[row]).mean() / Bd[1]:.4f})")
        print(f"   theory B_d(theta row) per N = {Bd[0]:.3e}; fitted theta-row of B = {B_perp[ax].round(6).tolist()}")

    # ------------------------------------------------------------------ E
    banner("E. Continuous-time model by the matrix logarithm of [[A, B],[0, I]]")
    aug = np.block([[A_t, B_t], [np.zeros((m, n)), np.eye(m)]])
    L = logm(aug).real / DT
    Ac, Bc = L[:n, :n], L[:n, n:]
    print("A_c =\n", Ac)
    print("B_c (identified torque part) =\n", Bc)
    print("Euler estimate (A - I)/dt diag(omega block):", np.diag((A_t - np.eye(n))[3:, 3:]) / DT)
    print("theory -C/I:", -C_ROT / IXX, -C_ROT / IYY, -C_ROT / IZZ)
    print("max |A_c - (A - I)/dt| =", np.abs(Ac - (A_t - np.eye(n)) / DT).max(), " (O(dt) difference)")

    # ------------------------------------------------------------------ F
    banner("F. Spectrum, multiplicities, modes")
    lam, V = np.linalg.eig(A_t)
    order = np.argsort(-lam.real)
    lam, V = lam[order], V[:, order]
    print("eigenvalues of A:", lam)
    print("continuous s = ln(lambda)/dt:", np.log(lam.astype(complex)) / DT)
    print("time constants 1/|s| of the decaying modes:", 1.0 / np.abs(np.log(lam[lam.real < 0.9999].astype(complex)) / DT))
    print("rank(A - I) =", np.linalg.matrix_rank(A_t - np.eye(n), tol=1e-9), "(n - rank = geometric multiplicity of 1)")
    print("condition number of the eigenvector matrix V:", np.linalg.cond(V))
    print("modes (columns), real parts, state order theta_xyz omega_xyz:\n", V.real)
    print("spectral radius:", np.abs(lam).max(), " max |A^k| over k <= 5500:",
          max(np.linalg.norm(np.linalg.matrix_power(A_t, k), 2) for k in (1, 10, 100, 500, 1000, 2500, 5500)))

    # ------------------------------------------------------------------ G
    banner("G. Controllability, reachability, observability")
    Btq = B_t @ T.T                               # 6 x 3, three torque channels (unit directions)
    C_d = np.hstack([np.linalg.matrix_power(A_t, k) @ Btq for k in range(n)])
    sC = np.linalg.svd(C_d, compute_uv=False)
    print("discrete controllability matrix [B AB .. A^5 B] (6 x 18) singular values:", sC)
    print(f"  rank (tol 1e-12) = {int(np.sum(sC > 1e-12))}; with the default tol rank = {np.linalg.matrix_rank(C_d)}; "
          f"cond(restricted to rank) = {sC[0] / sC[5]:.3e}")
    Bct = Bc @ T.T
    C_c = np.hstack([np.linalg.matrix_power(Ac, k) @ Bct for k in range(n)])
    sCc = np.linalg.svd(C_c, compute_uv=False)
    print("continuous controllability matrix singular values:", sCc, f" rank {int(np.sum(sCc > 1e-9 * sCc[0]))}")
    print("PBH test, rank [lambda I - A, B_tau] for each eigenvalue (need 6):",
          [int(np.linalg.matrix_rank(np.hstack([l * np.eye(n) - A_t, Btq]), tol=1e-9)) for l in lam])
    # finite-horizon Gramian and minimum energy to reach a unit state
    for Tf in (0.5, 2.0):
        steps = int(round(Tf / DT))
        W = np.zeros((n, n))
        Ak = np.eye(n)
        for _ in range(steps):
            W += Ak @ Btq @ Btq.T @ Ak.T
            Ak = A_t @ Ak
        ev = np.linalg.eigvalsh(W * DT)           # dt-weighted: matches the continuous Gramian
        print(f"controllability Gramian over {Tf} s: eigenvalues {ev}")
    Tf = 0.5
    steps = int(round(Tf / DT))
    W = np.zeros((n, n)); Ak = np.eye(n)
    for _ in range(steps):
        W += Ak @ Btq @ Btq.T @ Ak.T
        Ak = A_t @ Ak
    # energy sum ||u||^2 (N^2 per step) to reach roll 10 deg at rest in 0.5 s
    target = np.zeros(n); target[0] = np.radians(10.0)
    E_steps = float(target @ np.linalg.solve(W, target))
    print(f"minimum sum of squared difference-thrust commands to reach 10 deg of roll, at rest, in 0.5 s: "
          f"{E_steps:.3f} N^2 summed over {steps} steps (rms {np.sqrt(E_steps / steps):.4f} N per step in the torque channel)")
    print("observability: the full state is the output (C = I), rank", np.linalg.matrix_rank(np.eye(n)))

    # ------------------------------------------------------------------ H
    banner("H. Open-loop rollout on held-out near-hover segments")
    finals, maxes, lens = [], [], []
    for state, control in held_segs:
        zhat = state[0].copy()
        err = np.zeros((len(state) - 1, n))
        for k in range(len(state) - 1):
            zhat = A_t @ zhat + B_t @ (control[k] - HOVER_THRUST_PER_ROTOR)
            err[k] = zhat - state[k + 1]
        finals.append(np.linalg.norm(err[-1])); maxes.append(np.linalg.norm(err, axis=1).max()); lens.append(len(state) * DT)
    print(f"{len(held_segs)} segments, length {min(lens):.2f}..{max(lens):.2f} s")
    print(f"final state error norm: mean {np.mean(finals):.5f}, max {np.max(finals):.5f}")
    print(f"max state error norm over the rollout: mean {np.mean(maxes):.5f}, max {np.max(maxes):.5f}")
    print("rate of change of state norm for scale: rms |z| =",
          np.sqrt(np.mean([np.sum(s ** 2, axis=1).mean() for s, _ in held_segs])))
    ks = np.arange(0, 5501, 500)
    nrm = [np.linalg.norm(np.linalg.matrix_power(A_t, int(k)), 2) for k in ks]
    print("||A^k||_2 at k = 0,500,...,5500:", np.round(nrm, 3))

    # ------------------------------------------------------------------ I
    banner("I. Measurement noise: least-squares bias and the instrumental-variable fix")
    Zc, Zpc, dUc, Zmc = stack_pairs(fit_segs, with_prev=True)
    rng = np.random.default_rng(7)
    sd_theta_ratio = 0.0
    print("noise is ADDED here (the project's data are noise-free); sigma_w in rad/s, sigma_theta = sigma_w * 0.05 s")
    print(f"{'sigma_w':>8} {'lam_wx LS':>12} {'lam_wx IV':>12} {'b_x LS':>10} {'b_x IV':>10}   (true lam_wx = {A_hat[3, 3]:.5f}, b_x = {float(B_perp[3] @ T[0]):.5f})")
    for sw in (0.0, 0.002, 0.005, 0.01, 0.02):
        # noise per SAMPLE, shared by neighbouring snapshots exactly as real sensor noise is
        Z_full = [s + np.hstack([rng.normal(0, sw * 0.05, (len(s), 3)), rng.normal(0, sw, (len(s), 3))]) for s, _ in fit_segs]
        segs_n = [(zn, c) for zn, (_, c) in zip(Z_full, fit_segs)]
        Zn, Zpn, dUn, Zmn = stack_pairs(segs_n, with_prev=True)
        Omn = np.vstack([Zn, dUn])
        Gls = Zpn @ np.linalg.pinv(Omn, rcond=1e-8)
        Psi = np.vstack([Zmn, dUn])                      # instruments: previous state + the same inputs
        Giv = Zpn @ Psi.T @ np.linalg.pinv(Omn @ Psi.T, rcond=1e-8)
        bls = float(Gls[3, n:] @ T[0]); biv = float(Giv[3, n:] @ T[0])
        print(f"{sw:8.3f} {Gls[3, 3]:12.5f} {Giv[3, 3]:12.5f} {bls:10.5f} {biv:10.5f}")

    # ------------------------------------------------------------------ J
    banner("J. Savitzky-Golay as a linear filter (window 31, order 3)")
    h = savgol_coeffs(SG_WINDOW, SG_POLYORDER)
    for f in (0.5, 1, 2, 5, 10, 20):
        H = abs(np.sum(h * np.exp(-2j * np.pi * f * DT * np.arange(-(SG_WINDOW // 2), SG_WINDOW // 2 + 1))))
        print(f"  gain at {f:5.1f} Hz: {H:.6f}")
    print(f"  coefficients sum {h.sum():.6f}, symmetric (zero phase): {np.allclose(h, h[::-1])}")
    print("  ideal frequency bandwidth: the excitation chirps; gain at 1% attenuation ~", end=" ")
    fs = np.linspace(0.1, 80, 800)
    gains = np.array([abs(np.sum(h * np.exp(-2j * np.pi * f * DT * np.arange(-(SG_WINDOW // 2), SG_WINDOW // 2 + 1)))) for f in fs])
    print(f"{fs[np.argmax(gains < 0.99)]:.1f} Hz")

    # ------------------------------------------------------------------ K
    banner("K. What a linear model leaves out (held-out data)")
    def terms(frames):
        th, om, dom, dth = [], [], [], []
        for df in frames.values():
            for _, seg in df.groupby("segment_id", sort=True):
                th.append(seg[STATE_COLS[:3]].to_numpy()); om.append(seg[STATE_COLS[3:]].to_numpy())
                dom.append(seg[DERIV_COLS[3:]].to_numpy()); dth.append(seg[DERIV_COLS[:3]].to_numpy())
        return np.vstack(th), np.vstack(om), np.vstack(dom), np.vstack(dth)
    I3 = np.array([IXX, IYY, IZZ])
    for label, frames in (("near hover", hover_held), ("full envelope", full_held)):
        th, om, dom, dth = terms(frames)
        gyro = -np.cross(om, om * I3) / I3                 # -I^-1 (omega x I omega)
        kin = 0.5 * np.cross(th, om)                       # 1/2 theta x omega
        rms = lambda x: float(np.sqrt((x ** 2).mean()))    # noqa: E731
        print(f"{label:14s}: rms |omega| {rms(np.linalg.norm(om, axis=1)):.3f} rad/s; rms |theta| {np.degrees(rms(np.linalg.norm(th, axis=1))):.2f} deg")
        print(f"   rms of gyroscopic term / rms of omega_dot : {rms(gyro) / rms(dom):.4f}")
        print(f"   rms of (1/2) theta x omega / rms of theta_dot: {rms(kin) / rms(dth):.4f}")
        resid = dth - om
        for sgn in (+1, -1):
            c = float(np.sum(resid * sgn * kin) / np.sum(kin * kin))
            print(f"   regress (theta_dot - omega) on {sgn:+d}*(1/2)theta x omega: coefficient {c:.3f}, "
                  f"R2 {1 - np.sum((resid - c * sgn * kin) ** 2) / np.sum((resid - resid.mean(0)) ** 2):.3f}")

    # ------------------------------------------------------------------ L
    banner("L. Why the fitted thrust gain is a few percent low: the smoothing, not the physics")
    import pandas as pd
    from run_identification import RAW_DIR
    fit_ids = split[(split.regime == "near_hover") & (split.split == "fit")].trial_id
    raw = [pd.read_parquet(RAW_DIR / f"{tid}.parquet") for tid in fit_ids]
    for name, col, tdir, I_, lever in (("roll", "wx", T[0], IXX, ARM), ("pitch", "wy", T[1], IYY, ARM),
                                       ("yaw", "wz", T[2], IZZ, KM)):
        a_, b_ = C_ROT / I_, lever / I_
        gain_theory = 2.0 * b_ * (1 - np.exp(-a_ * DT)) / a_            # gain per unit-norm thrust direction
        rows = []
        for d in raw:
            w = d[col].to_numpy()
            v = (d[["u1", "u2", "u3", "u4"]].to_numpy() - HOVER_THRUST_PER_ROTOR) @ tdir
            rows.append(np.column_stack([w[:-1], v[:-1], w[1:]]))
        M = np.vstack(rows)
        c, *_ = np.linalg.lstsq(M[:, :2], M[:, 2], rcond=None)
        res = M[:, 2] - M[:, :2] @ c
        row = {"roll": 3, "pitch": 4, "yaw": 5}[name]
        sm_gain = float(B_perp[row] @ tdir)
        print(f"{name:5s} RAW omega: lambda {c[0]:.6f} (physics {np.exp(-a_ * DT):.6f}, smoothed fit {A_hat[row, row]:.6f}); "
              f"gain {c[1]:.5f} = {c[1] / gain_theory:.4f} x physics ({gain_theory:.5f}); "
              f"smoothed-data gain {sm_gain:.5f} = {sm_gain / gain_theory:.4f} x physics; "
              f"residual std raw {res.std():.2e} vs smoothed {np.sqrt(s2[row]):.2e}")
    # the chirp sweeps 0.1 -> 15 Hz linearly (excitation.py), so each frequency gets equal time
    sweep = np.linspace(0.1, 15.0, 2000)
    taps = np.arange(-(SG_WINDOW // 2), SG_WINDOW // 2 + 1)
    gain_sweep = np.mean([abs(np.sum(h * np.exp(-2j * np.pi * f * DT * taps))) for f in sweep])
    print(f"mean Savitzky-Golay gain over the 0.1-15 Hz sweep: {gain_sweep:.4f}  (compare the roll/pitch gain ratios above)")
    print("\ndone.")


if __name__ == "__main__":
    main()
