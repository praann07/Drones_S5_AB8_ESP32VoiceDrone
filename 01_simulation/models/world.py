"""The obstacle geometry the flight controller plans against, for whichever
scene is loaded. Three questions, answered the same way for every world:

  clamp(p0, p1)          furthest point of the straight flight p0->p1 that
                         stays SAFETY_MARGIN_M clear of every building
  path_collides(traj)    would this curved trajectory clip a building?
  surface_under(x, y)    height and name of what is directly below
                         (what "land" descends onto, what "down" stops above)
  nearest(p)             horizontal distance and name of the closest building
  footprints()           [(name, polygon (N,2), height)] of every solid piece
                         (what the route planner steers around)
  places()               {name: polygon} of the named spots "go to <name>" can target

CityWorld wraps the original box city (city.py, scene.xml) unchanged.
CampusWorld reads the convex building pieces make_campus.py wrote
(campus/campus_world.json, scene_campus.xml).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import city
from city import SAFETY_MARGIN_M, DRONE_RADIUS_M, FOOTPRINT_HALF_M

HERE = Path(__file__).resolve().parent
CAMPUS_JSON = HERE / "campus" / "campus_world.json"
CAMPUS_SCENE = HERE / "scene_campus.xml"


class CityWorld:
    name = "city"

    def clamp(self, p0, p1, sweep: bool = True):
        return city.clamp_target_to_safe_zone(p0, p1)

    def path_collides(self, traj) -> bool:
        return city.check_bezier_collision(*traj.control_points())

    def surface_under(self, x, y):
        return city.surface_under(x, y)

    def nearest(self, p):
        best = (np.inf, "")
        for b in city.buildings():
            dx = max(abs(p[0] - b.x) - b.half_x, 0.0)
            dy = max(abs(p[1] - b.y) - b.half_y, 0.0)
            best = min(best, (float(np.hypot(dx, dy)), b.name))
        return best

    def footprints(self):
        return [(b.name, _rect(b.x, b.y, b.half_x, b.half_y), b.height) for b in city.buildings()]

    def places(self):
        return {o.name: _rect(o.x, o.y, o.half_x, o.half_y)
                for o in city.CITY_OBJECTS if o.kind != "tree"}


def _rect(x, y, hx, hy):
    return np.array([[x - hx, y - hy], [x + hx, y - hy], [x + hx, y + hy], [x - hx, y + hy]])


class _Prism:
    """Convex polygon footprint (CCW) extruded from z=0 to z=height."""

    def __init__(self, pts, height, name):
        self.pts = np.asarray(pts, dtype=float)
        self.height = float(height)
        self.name = name
        edges = np.roll(self.pts, -1, axis=0) - self.pts
        n = np.stack([edges[:, 1], -edges[:, 0]], axis=1)          # outward for CCW
        n /= np.linalg.norm(n, axis=1, keepdims=True)
        self.normals = n
        self.offsets = np.einsum("ij,ij->i", n, self.pts)           # n . x <= offset inside
        self.lo = self.pts.min(axis=0)
        self.hi = self.pts.max(axis=0)

    def signed_distance_2d(self, x, y) -> float:
        """Max over edges of n.p - offset: <= 0 inside. Outside it never exceeds
        the true distance (it under-reads near corners), so checks err on the
        safe side."""
        return float(np.max(self.normals @ np.array([x, y]) - self.offsets))

    def distance_2d(self, x, y) -> float:
        """True horizontal distance to the footprint (0 inside)."""
        if self.signed_distance_2d(x, y) <= 0.0:
            return 0.0
        p = np.array([x, y])
        a = self.pts
        ab = np.roll(a, -1, axis=0) - a
        t = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.einsum("ij,ij->i", ab, ab), 0.0, 1.0)
        return float(np.min(np.linalg.norm(a + t[:, None] * ab - p, axis=1)))

    def segment_entry(self, p0, d):
        """Cyrus-Beck: parameter t in [0,1] where p0 + t d first enters the
        prism, or None if it never does."""
        t_in, t_out = 0.0, 1.0
        planes = [(np.array([n[0], n[1], 0.0]), c) for n, c in zip(self.normals, self.offsets)]
        planes.append((np.array([0.0, 0.0, 1.0]), self.height))     # roof
        for n, c in planes:
            num = c - n @ p0
            den = n @ d
            if abs(den) < 1e-12:
                if num < 0:
                    return None
                continue
            t = num / den
            if den < 0:
                t_in = max(t_in, t)
            else:
                t_out = min(t_out, t)
            if t_in > t_out:
                return None
        return t_in


class CampusWorld:
    name = "campus"

    def __init__(self, path: Path = CAMPUS_JSON):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        self.prisms = [_Prism(part, b["height"], b["name"])
                       for b in data["buildings"] for part in b["parts"]]
        self.scale = data["scale"]
        # every edge of every piece, stacked, so nearest() is one numpy pass
        # (looping 209 pieces in Python cost ~4 ms per call, called 20x/s)
        self._ea = np.concatenate([pr.pts for pr in self.prisms])
        self._eb = np.concatenate([np.roll(pr.pts, -1, axis=0) for pr in self.prisms])
        self._eowner = np.concatenate([np.full(len(pr.pts), i) for i, pr in enumerate(self.prisms)])
        self._lo = np.array([pr.lo for pr in self.prisms])
        self._hi = np.array([pr.hi for pr in self.prisms])
        self._places = {b["name"]: np.asarray(b["footprint"], dtype=float)
                        for b in data["buildings"] if not b["name"].startswith("building_")}

    def clamp(self, p0, p1, margin: float = SAFETY_MARGIN_M, sweep: bool = True):
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        d = p1 - p0
        seg = float(np.linalg.norm(d))
        if seg < 1e-9:
            return p1.copy()
        lo, hi = np.minimum(p0, p1)[:2], np.maximum(p0, p1)[:2]
        best_t = 1.0
        for pr in self.prisms:
            if np.any(pr.hi < lo - 1e-9) or np.any(pr.lo > hi + 1e-9):
                continue
            t = pr.segment_entry(p0, d)
            if t is not None:
                best_t = min(best_t, max(0.0, t - margin / seg))
        if sweep:
            best_t = min(best_t, self._graze_stop(p0, d, seg, margin))
        return p1.copy() if best_t >= 1.0 - 1e-9 else p0 + best_t * d

    def _graze_stop(self, p0, d, seg: float, margin: float, step_m: float = 0.05) -> float:
        """Parameter t in [0,1] of the first point of the flight p0 + t d that
        has come within `margin` of a wall it is getting closer to. The entry
        test above only stops a flight that would ENTER a building, so a drone
        sliding along an angled wall passed a few centimetres from it - and
        the 0.34 m wide hull touched, tipped over and pinned itself. A flight
        that starts inside the margin may still move away or along a wall at
        constant distance, only getting closer stops it."""
        ts = np.linspace(0.0, 1.0, max(2, int(seg / step_m) + 1))
        pts = p0 + ts[:, None] * d
        lo = pts.min(axis=0)[:2] - margin
        hi = pts.max(axis=0)[:2] + margin
        stop = 1.0
        for pr in self.prisms:
            if np.any(pr.hi < lo) or np.any(pr.lo > hi):
                continue
            dist = (pr.normals @ pts[:, :2].T - pr.offsets[:, None]).max(axis=0)
            below_roof = pts[:, 2] <= pr.height + DRONE_RADIUS_M
            bad = below_roof & (dist < margin) & (dist < dist[0] - 1e-3)
            if bad.any():
                stop = min(stop, float(ts[int(np.argmax(bad))]))
        return stop

    def path_collides(self, traj, margin: float = SAFETY_MARGIN_M,
                      radius: float = DRONE_RADIUS_M, samples: int = 60) -> bool:
        ts = np.linspace(0.0, traj.t_total, samples)
        pts = np.array([traj.position(t) for t in ts])
        lo, hi = pts.min(axis=0)[:2] - margin - radius, pts.max(axis=0)[:2] + margin + radius
        start = pts[0]
        for pr in self.prisms:
            if np.any(pr.hi < lo) or np.any(pr.lo > hi):
                continue
            # a path that starts inside the margin (e.g. already parked next
            # to a wall) only counts if it gets closer
            d0 = pr.signed_distance_2d(start[0], start[1])
            for p in pts:
                if p[2] > pr.height + radius:
                    continue
                dist = pr.signed_distance_2d(p[0], p[1])
                if dist < margin + radius and dist < d0 - 1e-6:
                    return True
        return False

    def surface_under(self, x, y, pad: float = FOOTPRINT_HALF_M):
        best = (0.0, "ground")
        for pr in self.prisms:
            if pr.height > best[0] and pr.signed_distance_2d(x, y) <= pad:
                best = (pr.height, pr.name)
        return best

    def nearest(self, p):
        """(horizontal distance, name) of the closest building piece; 0 inside one."""
        q = np.asarray(p[:2], dtype=float)
        in_box = np.nonzero(np.all((self._lo <= q) & (q <= self._hi), axis=1))[0]
        for i in in_box:
            if self.prisms[i].signed_distance_2d(q[0], q[1]) <= 0.0:
                return 0.0, self.prisms[i].name
        ab = self._eb - self._ea
        t = np.clip(np.einsum("ij,ij->i", q - self._ea, ab) / np.einsum("ij,ij->i", ab, ab), 0.0, 1.0)
        d = np.linalg.norm(self._ea + t[:, None] * ab - q, axis=1)
        k = int(np.argmin(d))
        return float(d[k]), self.prisms[self._eowner[k]].name

    def footprints(self):
        return [(pr.name, pr.pts, pr.height) for pr in self.prisms]

    def places(self):
        return dict(self._places)


def world_for(name: str):
    if name == "campus":
        return CampusWorld()
    return CityWorld()
