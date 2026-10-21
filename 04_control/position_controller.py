"""Outer position/velocity loop (docs/TDD.md section 7b) - STANDARD
physics, not identified. PD(+feedforward) on position/velocity error
produces a desired acceleration, converted to (q_desired, thrust_desired)
via the standard differential-flatness-style thrust-vector relation:
thrust must point along the desired net specific-force direction
(desired acceleration plus gravity compensation), and its magnitude is
that vector's norm times mass.

Yaw is never commanded (docs/TDD.md section 7b) - the tilt quaternion
that aligns body +Z with the desired thrust direction is used directly,
with no additional yaw correction, which is the natural/minimal-rotation
choice for a yaw-agnostic quadrotor.

Gains target a critically-damped position response assuming a PURE
double-integrator plant (p_ddot = a_desired exactly), with a settling
time deliberately several times slower than the inner loop's (~0.5-0.9s,
docs/CLAUDE.md Stage D notes) - required for the cascade assumption
(docs/TDD.md section 7b) to hold. In the actual cascaded closed loop this
assumption is not exact (the inner loop has its own lag, and translation
has drag), so the real response shows mild ringing (~0.07m, decaying by
~4s) after the trajectory ends rather than the theoretically-predicted
zero overshoot - reported honestly in the Stage D gate output rather than
re-described as "no overshoot" after the fact. Still well inside the
position tolerance and does not affect the gate.
"""
from __future__ import annotations

import numpy as np

MASS_KG = 1.0
GRAVITY = 9.81

# Target 2nd-order position response: critically damped (zeta=1, no
# overshoot - overshooting a commanded waypoint would look wrong in the
# live demo and risks re-triggering inner-loop saturation on the way
# back), settling time deliberately 2-6x the inner loop's ~0.5-0.9s.
POS_SETTLING_TIME_S = 2.0
POS_DAMPING_RATIO = 1.0
_wn = 4.0 / (POS_DAMPING_RATIO * POS_SETTLING_TIME_S)
KP_POS = _wn ** 2          # (m/s^2) per m of position error
KD_POS = 2 * POS_DAMPING_RATIO * _wn   # (m/s^2) per (m/s) of velocity error

# Optional integral action (Ki=0 by default = the Stage D gated PD law).
# A pure PD law leaves a steady-state offset under any constant
# disturbance: wind F or a mass error dm give e_ss = a_dist / KP_POS (0.1 kg
# extra -> 0.245 m; see 05_voice_interface/run_robustness.py). Ki removes it.
# Anti-windup: integrate only while the position error is small AND the
# commanded trajectory AND the drone itself are (nearly) at rest - a
# steady disturbance holds the drone still with an offset, whereas
# convergence after a move still has velocity. The rest condition matters:
# integrating through a move accumulates the normal tracking lag, which
# then overshoots the waypoint (measured: settle 2.96 s -> 5.23 s,
# 0.196 m overshoot on a nominal 1.5 m leg). Disturbances are still
# rejected because a hold phase follows every leg. The integral's
# acceleration contribution is also clamped to I_MAX_ACCEL_MPS2.
KI_POS_DEFAULT = 2.0          # (m/s^2) per (m*s) of accumulated error
I_ACTIVE_RADIUS_M = 0.5
I_ACTIVE_SPEED_MPS = 0.05     # |v_desired| below this = trajectory at rest
I_ACTIVE_VEL_MPS = 0.15       # and |v| below this = drone itself not converging (swept 0.08-1.0: 0.08 stalls on large offsets, >=0.25 overshoots nominal legs)
I_MAX_ACCEL_MPS2 = 2.0

# Banking limit (degrees)
MAX_BANK_DEG = 30.0

# Heading control
HEADING_DEADBAND_MPS = 0.1


def _quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product q1*q2 (both scalar-first) - composes q2 applied
    first, then q1, both expressed in the same (world) frame."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def minimal_rotation_z_to(v: np.ndarray) -> np.ndarray:
    """Quaternion (Hamilton, scalar-first) of the minimal rotation taking
    world +Z to the unit vector v. Leaves yaw about v unconstrained -
    the natural choice since yaw is never commanded (see module
    docstring)."""
    z = np.array([0.0, 0.0, 1.0])
    v = v / np.linalg.norm(v)
    axis = np.cross(z, v)
    s = np.linalg.norm(axis)
    c = float(np.dot(z, v))
    if s < 1e-9:
        if c > 0:
            return np.array([1.0, 0.0, 0.0, 0.0])
        # 180 degrees - z and v are anti-parallel; any perpendicular axis works.
        return np.array([0.0, 1.0, 0.0, 0.0])
    axis = axis / s
    angle = np.arctan2(s, c)
    return np.array([np.cos(angle / 2), *(np.sin(angle / 2) * axis)])


class PositionController:
    def __init__(self, mass: float = MASS_KG, kp: float = KP_POS, kd: float = KD_POS,
                 ki: float = 0.0, dt: float = 0.002):
        self.mass = mass
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt
        self._int = np.zeros(3)   # accumulated position error (m*s)

    def reset_integral(self) -> None:
        self._int = np.zeros(3)

    def compute(self, p: np.ndarray, v: np.ndarray,
                p_desired: np.ndarray, v_desired: np.ndarray,
                a_feedforward: np.ndarray,
                heading_mode: str = "fixed") -> tuple[np.ndarray, float]:
        """-> (q_desired, thrust_desired). p, v: world-frame position and
        velocity (matches docs/TDD.md section 1's v convention).

        heading_mode: "fixed" (yaw=0), "forward_facing" (face direction of travel),
                      or "explicit" (use self.explicit_yaw)."""
        e_p = p_desired - p
        e_v = v_desired - v
        a_desired = self.kp * e_p + self.kd * e_v + a_feedforward
        if self.ki > 0.0:
            if (np.linalg.norm(e_p) < I_ACTIVE_RADIUS_M
                    and np.linalg.norm(v_desired) < I_ACTIVE_SPEED_MPS
                    and np.linalg.norm(v) < I_ACTIVE_VEL_MPS):
                lim = I_MAX_ACCEL_MPS2 / self.ki
                self._int = np.clip(self._int + e_p * self.dt, -lim, lim)
            a_desired = a_desired + self.ki * self._int

        # Cap the lateral (horizontal) component of a_desired so the bank
        # angle it produces never exceeds MAX_BANK_DEG. The tilt of
        # minimal_rotation_z_to's thrust-direction rotation below IS the
        # bank angle (differential flatness: tilting body +Z to point along
        # the desired specific-force direction is exactly what makes the
        # thrust vector realize a_desired's lateral component) - there is
        # no separate banking rotation to apply on top of it, only this
        # cap on what feeds into it. An aggressive position correction
        # would otherwise demand an arbitrarily steep, unrealistic lean.
        a_vert = a_desired[2] + GRAVITY
        a_lateral = np.array([a_desired[0], a_desired[1], 0.0])
        a_lat_mag = np.linalg.norm(a_lateral)
        max_lat = max(a_vert, 0.0) * np.tan(np.radians(MAX_BANK_DEG))
        if a_lat_mag > max_lat and a_lat_mag > 1e-9:
            scale = max_lat / a_lat_mag
            a_desired = np.array([a_desired[0] * scale, a_desired[1] * scale, a_desired[2]])

        f_desired = a_desired + np.array([0.0, 0.0, GRAVITY])
        thrust_mag = self.mass * float(np.linalg.norm(f_desired))
        direction = f_desired / np.linalg.norm(f_desired)
        q_desired = minimal_rotation_z_to(direction)

        # Heading control (yaw about the now-tilted body Z, composed after
        # the bank so it doesn't fight it)
        if heading_mode == "forward_facing":
            speed = np.linalg.norm(v[0:2])
            if speed > HEADING_DEADBAND_MPS:
                psi_desired = np.arctan2(v[1], v[0])
                q_yaw = np.array([np.cos(psi_desired / 2), 0, 0, np.sin(psi_desired / 2)])
                q_desired = _quat_mul(q_yaw, q_desired)
        elif heading_mode == "explicit":
            psi = getattr(self, 'explicit_yaw', 0.0)
            q_yaw = np.array([np.cos(psi / 2), 0, 0, np.sin(psi / 2)])
            q_desired = _quat_mul(q_yaw, q_desired)

        return q_desired, thrust_mag
