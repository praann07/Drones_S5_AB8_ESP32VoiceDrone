"""Singularity-free quaternion attitude controller, Fresk-style P^2 law
(docs/TDD.md section 7): proportional feedback on the quaternion error
vector part plus proportional feedback on angular-velocity error.

    tau = -Kq * sign(qe_w) * qe_vec  -  Komega * (omega - omega_desired)

The sign(qe_w) term resolves the quaternion double-cover ambiguity (q and
-q represent the same rotation) so the controller always takes the SHORT
way around - without it, a large-angle error could command a rotation the
long way past 180 degrees. This is the same ambiguity-resolution already
used for the rotation-vector coordinate in 02_identification/
preprocessing.py::quat_log_to_rotvec, so the convention is consistent
across the project.

Gains are DERIVED from the Stage C-selected identified model (SINDy),
not hand-tuned constants - see gains_from_identified_model(). This is
what makes this "a controller designed against the identified model"
rather than a generic PD controller that happens to run on this system.
"""
from __future__ import annotations

import numpy as np

from mixer import HOVER_THRUST_PER_ROTOR, inverse_mix, inverse_mix_yaw_priority, mixer_matrix

# Target closed-loop second-order response, per axis. These are the
# ONLY hand-picked numbers in this file; everything else (Kp, Kd) is
# derived from them plus the identified model's inertia/drag.
SETTLING_TIME_S = 0.5   # 2% settling time; 4x faster than the outer loop's 2.0 s
DAMPING_RATIO = 0.7     # underdamped, small overshoot, no ringing
# 1.5 s was used until 2026-10-20, when the gains still lacked the 1/b
# factor (see gains_from_identified_model). With correct units a 1.5 s
# inner loop is too slow for the 2.0 s position loop above it: a forward
# leg ends 0.32 m short (04_control/run_stability.py history, CLAUDE.md).


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def identified_torque_effectiveness(B_u: np.ndarray) -> np.ndarray:
    """Per-axis b_i = d(omega_dot_i)/d(tau_i) [rad/s^2 per N*m], i.e. the
    identified 1/I_i. B_u is d(omega_dot)/d(u1..u4) from the SINDy model
    (3x4, rotor-thrust columns); mapping it through the inverse mixer
    turns per-rotor effectiveness into per-body-torque effectiveness."""
    B_tau = np.asarray(B_u) @ np.linalg.inv(mixer_matrix())[:, 1:]
    return np.diag(B_tau).copy()


def gains_from_identified_model(A_sindy: np.ndarray,
                                B_u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Per-axis (Kq, Komega) from the SINDy hover-linearization.

    A_sindy is the 6x6 Jacobian and B_u the omega_dot rows (3x4) of the
    control Jacobian from 03_validation.run_validation.
    linearize_sindy_at_hover. Per axis i the identified model is
        omega_dot_i = a_i*omega_i + b_i*tau_i
    with a_i = A_sindy[3+i, 3+i] (identified -c_rot/I_i) and b_i from
    identified_torque_effectiveness (identified 1/I_i). Neither uses the
    simulator's ground-truth I or c_rot.

    Standard 2nd-order design: with tau = -Kp*e - Kd*e_dot the closed loop is
        e_ddot + (b*Kd - a) e_dot + b*Kp e = 0
    so b*Kp = wn^2 and b*Kd - a = 2*zeta*wn. Converting to the quaternion
    error convention (qe_vec ~ e/2 for small angles): Kq = 2*Kp.

    Until 2026-10-20 the 1/b factor was missing, so the "torque" gains
    were really Kp/I and Kd/I - about 290x too large. The loop still
    flew, but its poles sat at -4.8 and -874 1/s instead of the designed
    pair, and it was only stable above ~440 Hz control rate (500 Hz in
    use, unstable with one sample of delay). 04_control/run_stability.py
    measures this and the fix.

    This is a deliberate simplification, stated explicitly rather than
    left implicit: only the diagonal of A_sindy is used, i.e. each axis
    is designed as an INDEPENDENT single-axis 2nd-order system. The
    identified model's off-diagonal terms (the gyroscopic cross-coupling
    between axes, recovered to within 2-6% of ground truth per
    tests/test_identification_physics.py) are real and present in the
    plant, but are not fed into the gain design itself - they show up as
    a disturbance the per-axis loops reject, not something the gains are
    tuned against. This is standard practice for a system with as much
    inertial symmetry as this one (diaginertia, no off-diagonal inertia
    terms - see quad.xml) and is verified adequate empirically in
    docs/CLAUDE.md's Stage D results (settling within tolerance on every
    tested maneuver, including combined-axis and large-angle steps), not
    just assumed.
    """
    wn = 4.0 / (DAMPING_RATIO * SETTLING_TIME_S)   # 2% settling-time rule of thumb
    Kq = np.zeros(3)
    Komega = np.zeros(3)
    b = identified_torque_effectiveness(B_u)
    for i in range(3):
        a = A_sindy[3 + i, 3 + i]   # negative: identified -c_rot/I_i
        Kq[i] = 2 * wn ** 2 / b[i]
        Komega[i] = max((2 * DAMPING_RATIO * wn + a) / b[i], 0.0)  # never negative damping
    return Kq, Komega


class QuaternionController:
    """Inner-loop attitude controller. Outputs 4 rotor thrust commands."""

    def __init__(self, Kq: np.ndarray, Komega: np.ndarray,
                 yaw_priority: bool = False):
        """yaw_priority (default False = the Stage D gated behavior): when
        a command would saturate a rotor, shed yaw torque first rather than
        plain clipping - see mixer.inverse_mix_yaw_priority."""
        self.Kq = Kq
        self.Komega = Komega
        self.yaw_priority = yaw_priority

    def compute(self, q: np.ndarray, omega: np.ndarray,
                q_desired: np.ndarray, omega_desired: np.ndarray,
                thrust_desired: float) -> tuple[np.ndarray, bool]:
        """One control step. q/q_desired: Hamilton scalar-first. omega in
        body frame (matches docs/TDD.md section 1 throughout).
        Returns (rotor_commands, saturated)."""
        qe = quat_multiply(quat_conjugate(q_desired), q)
        if qe[0] < 0:
            qe = -qe  # short-way-around, see module docstring
        qe_vec = qe[1:]
        omega_e = omega - omega_desired

        tau = -self.Kq * qe_vec - self.Komega * omega_e
        if self.yaw_priority:
            return inverse_mix_yaw_priority(thrust_desired, tau)
        return inverse_mix(thrust_desired, tau)

    def attitude_error_deg(self, q: np.ndarray, q_desired: np.ndarray) -> float:
        qe = quat_multiply(quat_conjugate(q_desired), q)
        w = np.clip(abs(qe[0]), -1.0, 1.0)
        return float(np.degrees(2 * np.arccos(w)))
