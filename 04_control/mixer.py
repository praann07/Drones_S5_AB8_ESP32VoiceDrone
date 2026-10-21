"""Mixer / inverse-mixer between (thrust, torque) and individual rotor
commands. Matches the rotor geometry in 01_simulation/models/quad.xml
and the mixer derivation in docs/TDD.md section 2 exactly - mixer_matrix()
is checked against that XML's actual rotor site positions and spin signs
in tests/test_mixer.py, not just asserted here.
"""
from __future__ import annotations

import numpy as np

ARM = 0.106066   # l/sqrt(2), m - docs/TDD.md section 2
K_M = 0.02       # rotor reaction-torque coefficient, m
U_MAX = 6.0      # N per rotor
HOVER_THRUST_PER_ROTOR = 1.0 * 9.81 / 4.0


def mixer_matrix() -> np.ndarray:
    """Rows: [T, tau_x, tau_y, tau_z]. Columns: [u1, u2, u3, u4].
    Rotor positions/spin signs per docs/TDD.md section 2 table."""
    return np.array([
        [1.0,    1.0,    1.0,    1.0],
        [ARM,   -ARM,   -ARM,    ARM],
        [-ARM,   ARM,   -ARM,    ARM],
        [K_M,    K_M,   -K_M,   -K_M],
    ])


_M = mixer_matrix()
_M_INV = np.linalg.inv(_M)


def inverse_mix(T: float, tau: np.ndarray, clip: bool = True) -> tuple[np.ndarray, bool]:
    """(thrust, torque) -> 4 rotor commands. Returns (u, saturated) - the
    caller must check `saturated` rather than silently accepting a
    clipped, physically-unachievable command as if it were exact."""
    u = _M_INV @ np.concatenate([[T], tau])
    saturated = bool(np.any(u < 0.0) or np.any(u > U_MAX))
    if clip:
        u = np.clip(u, 0.0, U_MAX)
    return u, saturated


def _feasible(u: np.ndarray) -> bool:
    return bool(np.all(u >= 0.0) and np.all(u <= U_MAX))


def inverse_mix_yaw_priority(T: float, tau: np.ndarray) -> tuple[np.ndarray, bool]:
    """Like inverse_mix, but when the command would clip a rotor, shed
    YAW torque first (then roll/pitch) so the collective thrust is kept.

    Why: yaw is the weakest axis (K_M=0.02 m vs ARM=0.106 m), so each N*m
    of yaw torque costs ~5x more rotor headroom than roll/pitch, yet yaw is
    never commanded to do real work here. With plain clipping, a small
    yaw-gyro error (0.05 rad/s) pinned rotors at 0/6 N on ~70% of steps and
    the lost collective thrust broke altitude hold (see run_robustness.py).
    When nothing would clip this returns exactly what inverse_mix does.

    Returns (u, intervened): intervened=True if any torque had to be shed.
    """
    tau = np.asarray(tau, dtype=float)
    u = _M_INV @ np.concatenate([[T], tau])
    if _feasible(u):
        return u, False

    def mix(sxy: float, sz: float) -> np.ndarray:
        return _M_INV @ np.concatenate([[T], [tau[0] * sxy, tau[1] * sxy, tau[2] * sz]])

    # Feasible set along each scale is an interval containing 0 (convex),
    # so bisection for its upper end is valid. 1) shed yaw only.
    if _feasible(mix(1.0, 0.0)):
        lo, hi = 0.0, 1.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if _feasible(mix(1.0, mid)) else (lo, mid)
        return mix(1.0, lo), True
    # 2) yaw fully shed and still infeasible: shrink roll/pitch too.
    if _feasible(mix(0.0, 0.0)):
        lo, hi = 0.0, 1.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if _feasible(mix(mid, 0.0)) else (lo, mid)
        return mix(lo, 0.0), True
    # 3) thrust alone is unachievable: nothing to prioritise, clip.
    return np.clip(mix(0.0, 0.0), 0.0, U_MAX), True
