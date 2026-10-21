"""Point-to-point trajectory planner: straight line, trapezoidal velocity
profile (docs/TDD.md section 7b). Never commands a position step directly
- a raw step would demand instantaneous velocity, which is physically
impossible and would just saturate the outer loop; the trapezoidal
profile instead ramps velocity smoothly so the attitude setpoints it
drives stay within what the inner loop was actually validated against.

Also provides QuinticBezierTrajectory: a smooth, curved trajectory with
full boundary condition control (position, velocity, acceleration at
both endpoints) for natural, physically plausible flight.
"""
from __future__ import annotations

import math

import numpy as np

# Stage D outer-loop parameters, locked here (docs/TDD.md section 9 log).
MAX_VELOCITY_MPS = 1.0
MAX_ACCEL_MPS2 = 1.0
WAYPOINT_OFFSET_M = 1.5   # left/right/forward/back move this far
POSITION_TOLERANCE_M = 0.15

# Quintic Bezier constraint limits
MAX_JERK_MPS3 = 10.0
MAX_THRUST_N = 24.0  # 4 rotors x 6.0 N


def _trapezoidal_1d(distance: float, v_max: float, a_max: float):
    """Returns (t_total, pos_fn, vel_fn, accel_fn) for 1D motion 0->distance."""
    if distance < 1e-9:
        return 0.0, (lambda t: 0.0), (lambda t: 0.0), (lambda t: 0.0)

    t_accel = v_max / a_max
    d_accel = 0.5 * a_max * t_accel ** 2

    if 2 * d_accel >= distance:
        # Triangular profile: never reaches v_max.
        t_accel = np.sqrt(distance / a_max)
        v_peak = a_max * t_accel
        t_total = 2 * t_accel
        t_cruise_end = t_accel
    else:
        d_cruise = distance - 2 * d_accel
        t_cruise = d_cruise / v_max
        v_peak = v_max
        t_total = 2 * t_accel + t_cruise
        t_cruise_end = t_accel + t_cruise

    def pos(t):
        if t <= 0:
            return 0.0
        if t < t_accel:
            return 0.5 * a_max * t ** 2
        if t < t_cruise_end:
            return 0.5 * a_max * t_accel ** 2 + v_peak * (t - t_accel)
        if t < t_total:
            td = t_total - t
            return distance - 0.5 * a_max * td ** 2
        return distance

    def vel(t):
        if t <= 0 or t >= t_total:
            return 0.0
        if t < t_accel:
            return a_max * t
        if t < t_cruise_end:
            return v_peak
        return a_max * (t_total - t)

    def accel(t):
        if t <= 0 or t >= t_total:
            return 0.0
        if t < t_accel:
            return a_max
        if t < t_cruise_end:
            return 0.0
        return -a_max

    return t_total, pos, vel, accel


class PointToPointTrajectory:
    """Straight-line trapezoidal-velocity trajectory from p0 to p1 (world
    frame, per docs/TDD.md section 7b - waypoint directions are WORLD
    frame, not body-relative, so 'forward' means +world-X regardless of
    current yaw - yaw is never commanded by any of the 6 voice commands,
    so this avoids needing to track it for direction semantics."""

    def __init__(self, p0: np.ndarray, p1: np.ndarray,
                 v_max: float = MAX_VELOCITY_MPS, a_max: float = MAX_ACCEL_MPS2):
        self.p0 = p0
        delta = p1 - p0
        self.distance = float(np.linalg.norm(delta))
        self.direction = delta / self.distance if self.distance > 1e-9 else np.zeros(3)
        self.t_total, self._pos, self._vel, self._accel = _trapezoidal_1d(self.distance, v_max, a_max)

    def position(self, t: float) -> np.ndarray:
        return self.p0 + self.direction * self._pos(min(max(t, 0.0), self.t_total))

    def velocity(self, t: float) -> np.ndarray:
        return self.direction * self._vel(min(max(t, 0.0), self.t_total))

    def acceleration(self, t: float) -> np.ndarray:
        return self.direction * self._accel(min(max(t, 0.0), self.t_total))


class CruiseTrajectory:
    """Straight line p0 -> p1 that starts at speed u0 along the line (the
    drone may already be moving), accelerates at a_max to v_max, cruises,
    and brakes at a_max to rest at p1. Used for continuous moves ("go
    straight" until a building or "stop"), where the quintic Bezier is a
    poor fit: its duration grows with distance and it creeps off the start
    (after 4 s of a 79 m leg it has moved ~3 cm).

    If the line is too short to stop from u0 at a_max, it brakes harder,
    at exactly u0^2 / (2 * distance)."""

    def __init__(self, p0: np.ndarray, p1: np.ndarray, u0: float = 0.0,
                 v_max: float = MAX_VELOCITY_MPS, a_max: float = MAX_ACCEL_MPS2):
        self.p0 = np.asarray(p0, dtype=float)
        self.p1 = np.asarray(p1, dtype=float)
        delta = self.p1 - self.p0
        D = float(np.linalg.norm(delta))
        self.distance = D
        self.direction = delta / D if D > 1e-9 else np.zeros(3)
        u0 = float(np.clip(u0, 0.0, v_max))
        if D < 1e-9:
            u0 = 0.0
        a_up, a_dn = a_max, a_max
        if u0 * u0 / (2 * a_max) >= D:             # can't stop in time at a_max
            vp = u0
            a_dn = u0 * u0 / (2 * D) if D > 1e-9 else a_max
        else:
            vp = min(v_max, np.sqrt((2 * a_max * D + u0 * u0) / 2))
        t1 = (vp - u0) / a_up
        d1 = (vp * vp - u0 * u0) / (2 * a_up)
        d3 = vp * vp / (2 * a_dn)
        t3 = vp / a_dn if vp > 0 else 0.0
        t2 = max(D - d1 - d3, 0.0) / vp if vp > 0 else 0.0
        self._seg = (u0, vp, a_up, a_dn, t1, t2, t3, d1)
        self.t_total = t1 + t2 + t3

    def _s(self, t):
        """(distance, speed, accel) along the line at time t."""
        u0, vp, a_up, a_dn, t1, t2, t3, d1 = self._seg
        t = min(max(t, 0.0), self.t_total)
        if t < t1:
            return u0 * t + 0.5 * a_up * t * t, u0 + a_up * t, a_up
        if t < t1 + t2:
            return d1 + vp * (t - t1), vp, 0.0
        if t < self.t_total:
            tt = t - t1 - t2
            return d1 + vp * t2 + vp * tt - 0.5 * a_dn * tt * tt, vp - a_dn * tt, -a_dn
        return self.distance, 0.0, 0.0

    def position(self, t: float) -> np.ndarray:
        return self.p0 + self.direction * self._s(t)[0]

    def velocity(self, t: float) -> np.ndarray:
        return self.direction * self._s(t)[1]

    def acceleration(self, t: float) -> np.ndarray:
        return self.direction * self._s(t)[2]


MAX_LATERAL_ACCEL_MPS2 = 0.8   # slows down for bends: v <= sqrt(a_lat / curvature)


class CruiseTurnTrajectory:
    """CruiseTrajectory that keeps the velocity the drone already has.

    CruiseTrajectory starts the new line at the speed the drone has ALONG it
    and drops the sideways part, so "forward" then "left" is an exact right
    angle with a stop-and-go at the corner. Here the sideways velocity w0 is
    carried and bled off smoothly instead (raised cosine over T seconds):

        p(t) = cruise(t) + w0 * W(t),   v(t) = cruise'(t) + w0 * s(t)
        s(t) = (1 + cos(pi t / T)) / 2,  W(t) = integral of s,  W(inf) = T / 2

    T = pi |w0| / (2 a_lat) keeps the sideways acceleration at or below a_lat,
    so the drone sweeps a rounded corner of radius about v^2 / a_lat and then
    runs on a lane shifted by |w0| T / 2 = pi |w0|^2 / (4 a_lat) in the
    direction it was heading. Position, velocity and acceleration stay
    continuous (up to the existing along-line profile)."""

    MIN_SIDEWAYS_MPS = 0.15      # below this the corner is not worth rounding

    def __init__(self, p0: np.ndarray, p1: np.ndarray, v0: np.ndarray,
                 v_max: float = MAX_VELOCITY_MPS, a_max: float = MAX_ACCEL_MPS2,
                 a_lat: float = MAX_LATERAL_ACCEL_MPS2):
        self.p0 = np.asarray(p0, dtype=float)
        self.p1 = np.asarray(p1, dtype=float)
        delta = self.p1 - self.p0
        D = float(np.linalg.norm(delta))
        d = delta / D if D > 1e-9 else np.zeros(3)
        v0 = np.array(v0, dtype=float)
        v0[2] = 0.0                                      # altitude is handled by the line itself
        u0 = max(float(v0 @ d), 0.0)
        self._w0 = v0 - u0 * d
        w = float(np.linalg.norm(self._w0))
        self.T = np.pi * w / (2.0 * a_lat) if w > 1e-9 else 0.0
        self._cruise = CruiseTrajectory(self.p0, self.p1, u0=u0, v_max=v_max, a_max=a_max)
        self.distance = self._cruise.distance
        self.direction = self._cruise.direction
        self.t_total = self._cruise.t_total
        self.sideways_mps = w

    @property
    def is_turn(self) -> bool:
        return self.sideways_mps >= self.MIN_SIDEWAYS_MPS

    @property
    def settles(self) -> bool:
        """The leg is long enough for the sideways velocity to reach zero before it ends."""
        return self.t_total >= self.T + 0.5

    def _s(self, t: float):
        """(W, s, ds/dt) of the sideways velocity decay at time t."""
        if self.T <= 1e-9:
            return 0.0, 0.0, 0.0
        if t >= self.T:
            return self.T / 2.0, 0.0, 0.0
        x = np.pi * t / self.T
        return (0.5 * (t + self.T / np.pi * np.sin(x)), 0.5 * (1.0 + np.cos(x)),
                -np.pi / (2.0 * self.T) * np.sin(x))

    def position(self, t: float) -> np.ndarray:
        t = min(max(t, 0.0), self.t_total)
        return self._cruise.position(t) + self._w0 * self._s(t)[0]

    def velocity(self, t: float) -> np.ndarray:
        t = min(max(t, 0.0), self.t_total)
        return self._cruise.velocity(t) + self._w0 * self._s(t)[1]

    def acceleration(self, t: float) -> np.ndarray:
        t = min(max(t, 0.0), self.t_total)
        return self._cruise.acceleration(t) + self._w0 * self._s(t)[2]


class PathTrajectory:
    """Smooth flight along a list of waypoints (e.g. a route_planner route).

    A cubic smoothing spline through the waypoints rounds every corner into
    a curve. The speed along it is the fastest profile that keeps
    acceleration <= a_max along the path and <= a_lat sideways in bends
    (forward/backward pass), starting at u0 and ending at rest. Altitude is
    held at the waypoints' z."""

    def __init__(self, waypoints: np.ndarray, u0: float = 0.0,
                 v_max: float = MAX_VELOCITY_MPS, a_max: float = MAX_ACCEL_MPS2,
                 a_lat: float = MAX_LATERAL_ACCEL_MPS2, ds: float = 0.05,
                 smoothing: float = 0.15):
        from scipy.interpolate import splev, splprep
        W = np.asarray(waypoints, dtype=float)
        keep = np.r_[True, np.linalg.norm(np.diff(W[:, :2], axis=0), axis=1) > 1e-6]
        W = W[keep]
        self.p0, self.p1 = W[0].copy(), W[-1].copy()
        if len(W) < 2:
            self._init_static()
            return
        if len(W) == 2:
            W = np.array([W[0], 0.5 * (W[0] + W[1]), W[1]])
        k = min(3, len(W) - 1)
        # smoothing s: allowed squared deviation from the waypoints, summed;
        # endpoints are pinned below so the drone starts and stops exactly
        tck, _u = splprep([W[:, 0], W[:, 1]], k=k, s=smoothing ** 2 * len(W))
        fine = np.linspace(0, 1, max(200, 40 * len(W)))
        xy = np.stack(splev(fine, tck), axis=1)
        xy[0], xy[-1] = W[0, :2], W[-1, :2]
        seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        s = np.r_[0.0, np.cumsum(seg)]
        L = float(s[-1])
        self.distance = L
        n = max(int(L / ds), 2) + 1
        si = np.linspace(0, L, n)
        P = np.stack([np.interp(si, s, xy[:, 0]), np.interp(si, s, xy[:, 1])], axis=1)
        T = np.gradient(P, si, axis=0)
        T /= np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-9)
        dT = np.gradient(T, si, axis=0)
        kappa = np.linalg.norm(dT, axis=1)
        self.kappa = kappa
        v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(kappa, 1e-6)))
        v[0] = min(v[0], max(u0, 0.0))
        v[-1] = 0.0
        h = si[1] - si[0]
        for i in range(1, n):                         # can't speed up faster than a_max
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a_max * h))
        for i in range(n - 2, -1, -1):                # ... or brake harder than a_max
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_max * h))
        vm = 0.5 * (v[1:] + v[:-1])
        t = np.r_[0.0, np.cumsum(h / np.maximum(vm, 1e-3))]
        self.t_total = float(t[-1])
        self._t = t
        z = float(W[-1, 2])
        self._P = np.column_stack([P, np.full(n, z)])
        self._V = np.column_stack([T * v[:, None], np.zeros(n)])
        self._A = np.gradient(self._V, t, axis=0)
        self.speed = v

    def _init_static(self):
        self.distance = 0.0
        self.t_total = 0.0
        self._t = np.array([0.0, 1e-6])
        self._P = np.array([self.p0, self.p0])
        self._V = np.zeros((2, 3))
        self._A = np.zeros((2, 3))

    def _at(self, arr, t):
        t = min(max(t, 0.0), self.t_total)
        return np.array([np.interp(t, self._t, arr[:, k]) for k in range(3)])

    def position(self, t: float) -> np.ndarray:
        return self._at(self._P, t)

    def velocity(self, t: float) -> np.ndarray:
        return self._at(self._V, t) if t < self.t_total else np.zeros(3)

    def acceleration(self, t: float) -> np.ndarray:
        return self._at(self._A, t) if t < self.t_total else np.zeros(3)

    def samples(self) -> np.ndarray:
        """The planned (x, y, z) points, for drawing the route."""
        return self._P.copy()


def _quintic_bezier_eval(tau, P0, P1, P2, P3, P4, P5):
    """Evaluate quintic Bezier curve at parameter tau in [0, 1]."""
    u = 1.0 - tau
    return (u**5 * P0 + 5 * u**4 * tau * P1 + 10 * u**3 * tau**2 * P2
            + 10 * u**2 * tau**3 * P3 + 5 * u * tau**4 * P4 + tau**5 * P5)


def _quintic_bezier_deriv1(tau, P0, P1, P2, P3, P4, P5):
    """First derivative of quintic Bezier curve at parameter tau."""
    u = 1.0 - tau
    return (5 * u**4 * (P1 - P0) + 20 * u**3 * tau * (P2 - P1)
            + 10 * u**2 * tau**2 * (P3 - P2) + 10 * u * tau**3 * (P4 - P3)
            + 5 * tau**4 * (P5 - P4))


def _quintic_bezier_deriv2(tau, P0, P1, P2, P3, P4, P5):
    """Second derivative of quintic Bezier curve at parameter tau."""
    u = 1.0 - tau
    return (20 * u**3 * (P2 - 2*P1 + P0) + 30 * u**2 * tau * (P3 - 2*P2 + P1)
            + 30 * u * tau**2 * (P4 - 2*P3 + P2) + 20 * tau**3 * (P5 - 2*P4 + P3))


def _quintic_bezier_deriv3(tau, P0, P1, P2, P3, P4, P5):
    """Third derivative (jerk) of quintic Bezier curve at parameter tau."""
    u = 1.0 - tau
    return (60 * u**2 * (P3 - 3*P2 + 3*P1 - P0)
            + 120 * u * tau * (P4 - 3*P3 + 3*P2 - P1)
            + 60 * tau**2 * (P5 - 3*P4 + 3*P3 - P2))


class QuinticBezierTrajectory:
    """Quintic Bezier trajectory with full boundary condition control.

    Provides smooth, curved paths with specified position, velocity, and
    acceleration at both endpoints. The path is defined by 6 control points
    computed from the boundary conditions, giving C2-continuous motion
    with bounded jerk.

    Interface is identical to PointToPointTrajectory for backward
    compatibility: position(t), velocity(t), acceleration(t).
    """

    def __init__(self, p0: np.ndarray, p1: np.ndarray,
                 v0: np.ndarray | None = None, a0: np.ndarray | None = None,
                 v1: np.ndarray | None = None, a1: np.ndarray | None = None,
                 v_max: float = MAX_VELOCITY_MPS, a_max: float = MAX_ACCEL_MPS2,
                 j_max: float = MAX_JERK_MPS3, curvature: float = 0.0):
        self.p0 = np.asarray(p0, dtype=float)
        self.p1 = np.asarray(p1, dtype=float)
        self.v0 = np.zeros(3) if v0 is None else np.asarray(v0, dtype=float)
        self.a0 = np.zeros(3) if a0 is None else np.asarray(a0, dtype=float)
        self.v1 = np.zeros(3) if v1 is None else np.asarray(v1, dtype=float)
        self.a1 = np.zeros(3) if a1 is None else np.asarray(a1, dtype=float)

        # Limit initial velocity to prevent overshooting
        v0_norm = np.linalg.norm(self.v0)
        if v0_norm > 0.5 * MAX_VELOCITY_MPS:
            self.v0 = self.v0 * (0.5 * MAX_VELOCITY_MPS / v0_norm)
        self.curvature = curvature
        self.v_max = v_max
        self.a_max = a_max
        self.j_max = j_max

        self.distance = float(np.linalg.norm(self.p1 - self.p0))
        self.t_total = self._compute_duration()
        self._P0, self._P1, self._P2, self._P3, self._P4, self._P5 = self._compute_control_points()

    def _compute_control_points(self):
        """Compute 6 control points from boundary conditions using standard
        quintic Bezier formulas. These correctly enforce all boundary conditions."""
        T = self.t_total
        P0 = self.p0
        P5 = self.p1

        P1 = P0 + self.v0 * T / 5.0
        P2 = P0 + 2.0 * self.v0 * T / 5.0 + self.a0 * T**2 / 20.0
        P4 = P5 - self.v1 * T / 5.0
        P3 = P5 - 2.0 * self.v1 * T / 5.0 + self.a1 * T**2 / 20.0
        return P0, P1, P2, P3, P4, P5

    def _compute_duration(self):
        """Find minimum feasible duration satisfying all constraints."""
        if self.distance < 1e-9:
            return 0.0

        # Lower bound: can't travel faster than v_max
        T_min = self.distance / self.v_max
        # Upper bound: generous limit
        T_max_search = 10.0 * T_min + 1.0

        # Binary search for minimum feasible T
        for _ in range(50):
            T = 0.5 * (T_min + T_max_search)
            if self._is_feasible(T):
                T_max_search = T
            else:
                T_min = T

        return T_max_search

    def _is_feasible(self, T):
        """Check if duration T satisfies all constraints by sampling."""
        if T < 1e-9:
            return False

        P0, P1, P2, P3, P4, P5 = self._compute_control_points_for_T(T)

        # Same 100 samples and same four limit checks as the original
        # per-sample loop, evaluated as one vectorised batch: the duration
        # bisection calls this ~50x per dispatch, and the scalar loop cost
        # ~160 ms per trajectory (vs 0.02 ms for the old trapezoid) - far
        # too slow for the live dispatch path. Output is unchanged.
        tau = np.linspace(0, 1, 100)[:, None]
        v = _quintic_bezier_deriv1(tau, P0, P1, P2, P3, P4, P5) / T
        a = _quintic_bezier_deriv2(tau, P0, P1, P2, P3, P4, P5) / T**2
        j = _quintic_bezier_deriv3(tau, P0, P1, P2, P3, P4, P5) / T**3

        if (np.linalg.norm(v, axis=1) > self.v_max).any():
            return False
        if (np.linalg.norm(a, axis=1) > self.a_max).any():
            return False
        if (np.linalg.norm(j, axis=1) > self.j_max).any():
            return False

        # Thrust constraint: T = m * ||a + g||
        thrust = 1.0 * np.linalg.norm(a + np.array([0, 0, 9.81]), axis=1)
        if (thrust > MAX_THRUST_N).any():
            return False

        return True

    def _compute_control_points_for_T(self, T):
        """Compute control points for a given duration T."""
        P0 = self.p0
        P5 = self.p1
        P1 = P0 + self.v0 * T / 5.0
        P2 = P0 + 2.0 * self.v0 * T / 5.0 + self.a0 * T**2 / 20.0
        P4 = P5 - self.v1 * T / 5.0
        P3 = P5 - 2.0 * self.v1 * T / 5.0 + self.a1 * T**2 / 20.0
        return P0, P1, P2, P3, P4, P5

    def position(self, t: float) -> np.ndarray:
        """Position at time t."""
        if self.t_total < 1e-9:
            return self.p0.copy()
        tau = min(max(t / self.t_total, 0.0), 1.0)
        return _quintic_bezier_eval(tau, self._P0, self._P1, self._P2,
                                    self._P3, self._P4, self._P5)

    def velocity(self, t: float) -> np.ndarray:
        """Velocity at time t."""
        if self.t_total < 1e-9:
            return np.zeros(3)
        tau = min(max(t / self.t_total, 0.0), 1.0)
        return _quintic_bezier_deriv1(tau, self._P0, self._P1, self._P2,
                                       self._P3, self._P4, self._P5) / self.t_total

    def acceleration(self, t: float) -> np.ndarray:
        """Acceleration at time t."""
        if self.t_total < 1e-9:
            return np.zeros(3)
        tau = min(max(t / self.t_total, 0.0), 1.0)
        return _quintic_bezier_deriv2(tau, self._P0, self._P1, self._P2,
                                       self._P3, self._P4, self._P5) / self.t_total**2

    def control_points(self) -> tuple[np.ndarray, ...]:
        """The 6 Bezier control points (P0..P5), for external use such as
        collision checking (see city.py::check_bezier_collision)."""
        return (self._P0, self._P1, self._P2, self._P3, self._P4, self._P5)

    def jerk(self, t: float) -> np.ndarray:
        """Jerk at time t."""
        if self.t_total < 1e-9:
            return np.zeros(3)
        tau = min(max(t / self.t_total, 0.0), 1.0)
        return _quintic_bezier_deriv3(tau, self._P0, self._P1, self._P2,
                                       self._P3, self._P4, self._P5) / self.t_total**3
