"""Obstacle-avoiding B-spline planner (drop-in replacement for path_planner.py).

Pipeline: inflate every building box -> A* on a 1 m grid (finds a collision-free
route around ALL buildings, so it can snake) -> prune to key corners ->
cubic B-spline -> verify the spline itself stays outside the inflated boxes.
Same public API as before: SplinePathPlanner().generate_bspline_path(start, goal, ...)
Coordinates: local metres, x = East, y = North, z = altitude.
"""
import heapq
import json

import numpy as np
from scipy.interpolate import splev, splprep


class SplinePathPlanner:

  def __init__(self, obstacle_file="obstacles.json"):
    self.obstacles = self._load(obstacle_file)

  def _load(self, path):
    try:
      with open(path) as f:
        data = json.load(f)
    except Exception as e:
      print(f"   [Error loading {path}: {e}]")
      return []
    raw = data.get("buildings", []) if isinstance(data, dict) else data
    out = []
    for b in raw:
      if all(k in b for k in ("x_min", "x_max", "y_min", "y_max")):
        out.append(dict(b))
      elif "x" in b and "y" in b:  # circle -> bounding box
        r = b.get("radius", 5.0)
        out.append({"name": b.get("name", "obs"), "x_min": b["x"] - r,
                    "x_max": b["x"] + r, "y_min": b["y"] - r,
                    "y_max": b["y"] + r})
    return out

  # ---- geometry helpers ----
  def _boxes(self, margin):
    return [(o["x_min"] - margin, o["x_max"] + margin,
             o["y_min"] - margin, o["y_max"] + margin) for o in self.obstacles]

  @staticmethod
  def _hit(p, boxes):
    return any(x0 <= p[0] <= x1 and y0 <= p[1] <= y1 for x0, x1, y0, y1 in boxes)

  def _segment_free(self, a, b, boxes, step=0.5):
    n = max(2, int(np.linalg.norm(b - a) / step))
    for t in np.linspace(0, 1, n):
      if self._hit(a + t * (b - a), boxes):
        return False
    return True

  def _astar(self, start, goal, boxes, res=1.0, pad=40.0):
    xs = [start[0], goal[0]] + [v for b in boxes for v in b[:2]]
    ys = [start[1], goal[1]] + [v for b in boxes for v in b[2:]]
    x0, y0 = min(xs) - pad, min(ys) - pad
    nx = int((max(xs) + pad - x0) / res) + 1
    ny = int((max(ys) + pad - y0) / res) + 1
    blocked = np.zeros((nx, ny), bool)
    for bx0, bx1, by0, by1 in boxes:
      blocked[max(0, int((bx0 - x0) / res)):int((bx1 - x0) / res) + 1,
              max(0, int((by0 - y0) / res)):int((by1 - y0) / res) + 1] = True
    cell = lambda p: (int(round((p[0] - x0) / res)), int(round((p[1] - y0) / res)))
    s, g = cell(start), cell(goal)
    blocked[s] = blocked[g] = False  # never trap start/goal
    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    h = lambda c: np.hypot(c[0] - g[0], c[1] - g[1])
    pq, best, prev = [(h(s), 0, s)], {s: 0}, {}
    while pq:
      _, d, c = heapq.heappop(pq)
      if c == g:
        break
      if d > best.get(c, 1e18):
        continue
      for dx, dy, w in moves:
        n = (c[0] + dx, c[1] + dy)
        if not (0 <= n[0] < nx and 0 <= n[1] < ny) or blocked[n]:
          continue
        if dx and dy and (blocked[c[0] + dx, c[1]] or blocked[c[0], c[1] + dy]):
          continue  # no corner cutting
        nd = d + w
        if nd < best.get(n, 1e18):
          best[n], prev[n] = nd, c
          heapq.heappush(pq, (nd + h(n), nd, n))
    if g not in prev and s != g:
      return None
    path, c = [g], g
    while c != s:
      c = prev[c]
      path.append(c)
    return [np.array([x0 + i * res, y0 + j * res]) for i, j in path[::-1]]

  def _prune(self, pts, boxes):
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
      j = len(pts) - 1
      while j > i + 1 and not self._segment_free(pts[i], pts[j], boxes):
        j -= 1
      out.append(pts[j])
      i = j
    return out

  # ---- public API ----
  def generate_bspline_path(self, start, goal, num_points=35,
                            safety_margin=5.0, **_):
    start, goal = np.array(start, float), np.array(goal, float)
    margin = safety_margin
    for _try in range(6):  # if the spline clips a building, widen margin
      boxes = self._boxes(margin)
      route = self._astar(start[:2], goal[:2], boxes)
      if route is None:
        print("   [Planner: no collision-free route found]")
        return np.array([start, goal])
      key = self._prune(route, boxes)
      # densify so the spline hugs the corners instead of cutting them
      dense = [key[0]]
      for a, b in zip(key[:-1], key[1:]):
        k = max(1, int(np.linalg.norm(b - a) / 8.0))
        dense += [a + (b - a) * t for t in np.linspace(0, 1, k + 1)[1:]]
      dense = np.array(dense)
      seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
      z = np.interp(seg, [0, seg[-1]], [start[2], goal[2]])
      pts = np.c_[dense, z]
      if len(pts) < 2:
        return np.array([start, goal])
      k = min(3, len(pts) - 1)
      tck, _ = splprep(pts.T, s=0, k=k)
      path = np.array(splev(np.linspace(0, 1, num_points), tck)).T
      check = self._boxes(safety_margin * 0.5)
      fine = np.array(splev(np.linspace(0, 1, 400), tck)).T
      if not any(self._hit(p, check) for p in fine):
        return path
      margin += 2.0
    return path
