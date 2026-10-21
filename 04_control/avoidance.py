"""Autonomous reference generator with moving-obstacle avoidance.

This is the autonomy layer ABOVE the Stage D cascade, not a replacement for
it. It produces a smooth reference (p_d, v_d, a_ff) that the unchanged outer
PD position loop and identified-model inner quaternion loop then track -
exactly the same interface PointToPointTrajectory / QuinticBezierTrajectory
provide, so the "never bypass the cascade" rule holds (see CLAUDE.md).

How the path becomes curved instead of straight
-----------------------------------------------
A "carrot" (the reference point) is steered every control step by a velocity
command built from four terms:

 1. goal attraction      toward the current waypoint (cruise speed, with the
                         component pointing INTO any nearby obstacle removed,
                         so the carrot slides around it instead of pushing in);
 2. repulsion            away from each obstacle, weighted by the larger of
                         a proximity weight and a predictive time-to-collision
                         weight (so a fast crosser triggers an early dodge);
 3. swirl                a tangential component around each obstacle whose
                         direction is COMMITTED per obstacle (no left/right
                         dithering, and it breaks the head-on symmetry where
                         pure repulsion would stall) - this is what bends the
                         path around obstacles;
 4. altitude band        soft floor/ceiling.

The command is turned into an acceleration-limited, jerk-limited carrot
motion so the cascade can actually track it. A hard keep-out projection is
the last resort (counted in ``emergency_count``; a good run has zero).

Honest scope: this is a reactive local planner with perfect knowledge of
obstacle position and velocity (no perception/noise model) and no global
search, so it can in principle get trapped (e.g. pinched between obstacles).
run_autonomous.py measures success over several seeds rather than asserting it.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "01_simulation" / "models"))

DRONE_RADIUS_M = 0.20          # conservative physical radius (arm 0.15 m + prop)


def clearance(p: np.ndarray, obstacles, t: float) -> float:
    """Smallest surface-to-surface distance (m) between the drone body and
    any obstacle; negative = overlap."""
    if not obstacles:
        return float("inf")
    return min(o.query(p, t)[0] for o in obstacles) - DRONE_RADIUS_M


class AvoidancePlanner:
    def __init__(self, waypoints, obstacles, p_start, v_start=None,
                 v_cruise: float = 0.8, v_max: float = 1.0,
                 a_max: float = 1.0, j_max: float = 8.0,
                 keepout_m: float = 0.35, influence_m: float = 1.3,
                 horizon_s: float = 2.5, reach_m: float = 0.45,
                 loop: bool = True):
        self.waypoints = [np.asarray(w, float) for w in waypoints]
        self.obstacles = list(obstacles)
        self.v_cruise, self.v_max = v_cruise, v_max
        self.a_max, self.j_max = a_max, j_max
        self.keepout = keepout_m
        self.influence = influence_m
        self.horizon = horizon_s
        self.reach = reach_m
        self.loop = loop
        self.tau = 0.35                 # velocity-command smoothing time (s)
        self.pr = np.asarray(p_start, float).copy()
        self.vr = np.zeros(3) if v_start is None else np.asarray(v_start, float).copy()
        self.ar = np.zeros(3)
        self.wp_idx = 0
        self.laps = 0
        self.waypoints_reached = 0
        self._side: dict[str, float] = {}      # committed swirl side per obstacle
        self.emergency_count = 0
        self.stall_s = 0.0               # how long the carrot has been ~stationary
        self.escapes = 0                 # times the stall-escape flipped the sides
        self.min_planned_clearance = float("inf")

    # ------------------------------------------------------------------
    @property
    def goal(self) -> np.ndarray:
        return self.waypoints[self.wp_idx]

    def _advance_waypoint(self) -> None:
        if np.linalg.norm(self.goal - self.pr) < self.reach:
            self.waypoints_reached += 1
            if self.wp_idx + 1 < len(self.waypoints):
                self.wp_idx += 1
            elif self.loop:
                self.wp_idx, self.laps = 0, self.laps + 1
            # else: stay on the last waypoint and settle there

    def _freer_side(self, t: float, tang: np.ndarray, gdir: np.ndarray, skip) -> float:
        """+1/-1: which tangent direction has more free space ~1.2 m out and a
        little toward the goal (smallest gap over every OTHER obstacle)."""
        best_side, best_gap = 1.0, -1e9
        for side in (1.0, -1.0):
            q = self.pr + side * tang * 1.2 + gdir * 0.5
            gaps = [ob.query(q, t)[0] for ob in self.obstacles if ob is not skip]
            g = min(gaps) if gaps else 1e3
            if g > best_gap + 1e-9:
                best_side, best_gap = side, g
        return best_side

    def step(self, t: float, p_drone: np.ndarray, v_drone: np.ndarray, dt: float):
        self._advance_waypoint()
        goal = self.goal
        to_goal = goal - self.pr
        dist = float(np.linalg.norm(to_goal))
        gdir = to_goal / dist if dist > 1e-9 else np.zeros(3)
        last_and_final = (not self.loop) and self.wp_idx == len(self.waypoints) - 1
        speed = min(self.v_cruise, np.sqrt(2.0 * 0.6 * dist)) if last_and_final else self.v_cruise
        v_goal = gdir * speed

        # Let the carrot wait for the drone instead of running away from it.
        sep = float(np.linalg.norm(self.pr - p_drone))
        v_goal = v_goal * float(np.clip(1.0 - (sep - 0.35) / 0.45, 0.0, 1.0))

        v_rep = np.zeros(3)
        v_swirl = np.zeros(3)
        r_total = DRONE_RADIUS_M + self.keepout
        for o in self.obstacles:
            gap, n, vo = o.query(self.pr, t)
            gk = gap - r_total                       # gap to the keep-out surface
            if gk > self.influence + 1.0:
                self._side.pop(o.name, None)
                continue
            closing = -float(np.dot(self.vr - vo, n))
            # radial repulsion is SHORT range (only near the keep-out surface);
            # long-range avoidance is done by the swirl below. Pure long-range
            # repulsion cancels the goal attraction head-on and parks the
            # drone in front of the obstacle (a local minimum - observed).
            w_rep = float(np.clip(1.0 - gk / (0.6 * self.influence), 0.0, 1.0)) ** 2
            w_sw = float(np.clip(1.0 - gk / self.influence, 0.0, 1.0))
            w_ttc = 0.0
            if closing > 0.05:
                ttc = max(gk, 0.0) / closing
                w_ttc = float(np.clip(1.0 - ttc / self.horizon, 0.0, 1.0))
            w_rep = max(w_rep, w_ttc)
            w_sw = max(w_sw, w_ttc)
            if max(w_rep, w_sw) <= 1e-3:
                if gk > self.influence * 1.2:
                    self._side.pop(o.name, None)
                continue

            # how much the obstacle blocks the way to the goal (1 = dead ahead)
            block = float(np.clip(-np.dot(gdir, n), 0.0, 1.0))

            # (1) don't push the goal command into the obstacle
            inward = -float(np.dot(v_goal, n))
            if inward > 0.0:
                v_goal = v_goal + n * inward * min(1.0, 3.0 * w_sw)

            # (2) repulsion (+ drift partly with the obstacle to cut closing speed)
            v_rep += n * w_rep * self.v_max + vo * 0.5 * w_rep

            # (3) committed tangential swirl around the obstacle - strongest
            #     when the obstacle is between the drone and its goal
            tang = np.cross(np.array([0.0, 0.0, 1.0]), n)
            tn = float(np.linalg.norm(tang))
            if tn < 0.2:                             # nearly vertical normal
                tang = np.cross(n, gdir if np.linalg.norm(gdir) > 0 else np.array([1.0, 0, 0]))
                tn = float(np.linalg.norm(tang))
            if tn > 1e-6:
                tang = tang / tn
                if o.name not in self._side:
                    along_goal = float(np.dot(tang, gdir))
                    if abs(along_goal) > 0.5:        # clearly one side is toward the goal
                        side = 1.0 if along_goal > 0 else -1.0
                    else:                            # ambiguous / head-on: take the side with more free space
                        side = self._freer_side(t, tang, gdir, o)
                    self._side[o.name] = side
                v_swirl += (self._side[o.name] * tang * (0.3 + 0.7 * block)
                            * w_sw * 1.1 * self.v_cruise)

        v_cmd = v_goal + v_rep + v_swirl
        # soft altitude band
        if self.pr[2] < 0.7:
            v_cmd[2] += (0.7 - self.pr[2]) * 1.5
        elif self.pr[2] > 2.3:
            v_cmd[2] -= (self.pr[2] - 2.3) * 1.5
        v_cmd[2] = float(np.clip(v_cmd[2], -0.5, 0.5))
        nrm = float(np.linalg.norm(v_cmd))
        if nrm > self.v_max:
            v_cmd *= self.v_max / nrm

        # acceleration- and jerk-limited carrot motion
        a_cmd = (v_cmd - self.vr) / self.tau
        an = float(np.linalg.norm(a_cmd))
        if an > self.a_max:
            a_cmd *= self.a_max / an
        da = a_cmd - self.ar
        dn = float(np.linalg.norm(da))
        if dn > self.j_max * dt:
            da *= self.j_max * dt / dn
        self.ar = self.ar + da
        self.vr = self.vr + self.ar * dt
        self.pr = self.pr + self.vr * dt

        # stall escape: a stationary carrot far from its goal means opposing
        # obstacle forces have cancelled (local minimum) - flip the committed
        # swirl sides once so the other way around gets tried.
        if float(np.linalg.norm(self.vr)) < 0.08 and dist > 1.0:
            self.stall_s += dt
            if self.stall_s > 3.0:
                for k in list(self._side):
                    self._side[k] = -self._side[k]
                self.escapes += 1
                self.stall_s = 0.0
        else:
            self.stall_s = 0.0

        # last-resort hard keep-out so the carrot is never inside an obstacle
        hard = DRONE_RADIUS_M + 0.10
        for o in self.obstacles:
            gap, n, vo = o.query(self.pr, t)
            if gap < hard:
                self.pr = self.pr + n * (hard - gap)
                vn = float(np.dot(self.vr - vo, n))
                if vn < 0.0:
                    self.vr = self.vr - vn * n
                self.emergency_count += 1
            self.min_planned_clearance = min(self.min_planned_clearance,
                                             gap - DRONE_RADIUS_M)
        return self.pr.copy(), self.vr.copy(), self.ar.copy()
