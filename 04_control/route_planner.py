"""Route planning for "go to <place>": a path around buildings, not through them.

Standard (not identified) outer-loop planning, like trajectory.py:

1. Occupancy grid (CELL_M) of the map at the flight altitude. A building
   counts only if it reaches within OVERFLY_GAP_M of that altitude; lower
   ones are flown over.
2. Distance transform: every cell's distance to the nearest building.
3. A* from the drone to a ring of cells just outside the target building.
   A step costs its length, plus a penalty that grows the closer it passes
   to a wall (inside PREFERRED_M). The route therefore keeps to the middle
   of open spaces and bends around corners, instead of hugging walls in
   straight lines. Cells closer than CLEARANCE_M to a wall are forbidden.
4. The cell path is thinned to a waypoint every WAYPOINT_SPACING_M; the
   smooth curve through them is trajectory.PathTrajectory.
"""
from __future__ import annotations

import heapq
import math

import numpy as np
from matplotlib.path import Path as MplPath
from scipy.ndimage import distance_transform_edt

CELL_M = 0.25
CLEARANCE_M = 0.45          # hard: route never passes closer than this to a wall
PREFERRED_M = 1.2           # soft: inside this the penalty starts
WALL_PENALTY = 4.0          # extra cost factor at CLEARANCE_M, falling to 0 at PREFERRED_M
OVERFLY_GAP_M = 0.35        # a roof this far below the flight altitude is flown over
GOAL_STANDOFF_M = 0.6       # stop this far from the target building's wall
WAYPOINT_SPACING_M = 1.0


class RoutePlanner:
    def __init__(self, world, bounds):
        """world: world.CityWorld / CampusWorld; bounds: (lo, hi) xy corners."""
        self.world = world
        lo, hi = (np.asarray(b, dtype=float) for b in bounds)
        self.lo = lo
        self.nx = int(math.ceil((hi[0] - lo[0]) / CELL_M))
        self.ny = int(math.ceil((hi[1] - lo[1]) / CELL_M))
        xs = lo[0] + (np.arange(self.nx) + 0.5) * CELL_M
        ys = lo[1] + (np.arange(self.ny) + 0.5) * CELL_M
        self._X, self._Y = np.meshgrid(xs, ys)        # [row=iy, col=ix]
        self._cache = {}

    # ---- grid helpers -------------------------------------------------
    def cell(self, x, y):
        ix = int(np.clip((x - self.lo[0]) / CELL_M, 0, self.nx - 1))
        iy = int(np.clip((y - self.lo[1]) / CELL_M, 0, self.ny - 1))
        return iy, ix

    def centre(self, iy, ix):
        return (self.lo[0] + (ix + 0.5) * CELL_M, self.lo[1] + (iy + 0.5) * CELL_M)

    def _mask(self, polygon):
        """Boolean grid of the cells inside a polygon (only its bounding box is tested)."""
        pts = np.asarray(polygon, dtype=float)
        m = np.zeros((self.ny, self.nx), dtype=bool)
        r0, c0 = self.cell(*(pts.min(axis=0) - CELL_M))
        r1, c1 = self.cell(*(pts.max(axis=0) + CELL_M))
        sub = np.stack([self._X[r0:r1 + 1, c0:c1 + 1].ravel(),
                        self._Y[r0:r1 + 1, c0:c1 + 1].ravel()], axis=1)
        inside = MplPath(pts).contains_points(sub, radius=1e-9)
        m[r0:r1 + 1, c0:c1 + 1] = inside.reshape(r1 - r0 + 1, c1 - c0 + 1)
        if not m.any():                               # sliver smaller than a cell
            m[self.cell(*pts.mean(axis=0))] = True
        return m

    def clearance(self, z):
        """Distance (m) from each cell to the nearest building that matters at altitude z."""
        key = round(float(z), 1)
        if key not in self._cache:
            occ = np.zeros((self.ny, self.nx), dtype=bool)
            for _name, pts, height in self.world.footprints():
                if height + OVERFLY_GAP_M > z:
                    occ |= self._mask(pts)
            self._cache[key] = distance_transform_edt(~occ) * CELL_M
        return self._cache[key]

    # ---- planning -------------------------------------------------------
    def plan_to_polygon(self, start_xy, polygon, z):
        """Waypoints (K,2) from start_xy to a spot GOAL_STANDOFF_M outside
        `polygon`, or None if no route exists. The last point is the goal."""
        clear = self.clearance(z)
        to_target = distance_transform_edt(~self._mask(polygon)) * CELL_M
        lo_r = max(GOAL_STANDOFF_M, CLEARANCE_M) - 0.5 * CELL_M
        goal = (to_target >= lo_r) & (to_target <= lo_r + 1.5 * CELL_M) & (clear >= CLEARANCE_M)
        if not goal.any():
            cand = (to_target > 0) & (clear >= CLEARANCE_M)   # crowded: nearest free cells
            if cand.any():
                goal = cand & (to_target <= to_target[cand].min() + CELL_M)
        if not goal.any():
            return None
        return self._astar(start_xy, goal, clear)

    def plan_to_point(self, start_xy, goal_xy, z):
        clear = self.clearance(z)
        goal = np.zeros_like(clear, dtype=bool)
        goal[self.cell(*goal_xy)] = True
        if clear[self.cell(*goal_xy)] < CLEARANCE_M:
            return None
        return self._astar(start_xy, goal, clear)

    def _astar(self, start_xy, goal, clear):
        free = clear >= CLEARANCE_M
        h = distance_transform_edt(~goal) * CELL_M          # admissible: straight-line to nearest goal
        penalty = WALL_PENALTY * np.clip((PREFERRED_M - clear) / (PREFERRED_M - CLEARANCE_M), 0, 1)
        start = self.cell(*start_xy)
        lead_in = []
        if not free[start]:
            # parked inside the clearance band (e.g. stopped 0.3 m from a
            # wall): first step out to the nearest free cell
            fy, fx = np.nonzero(free)
            if fy.size == 0:
                return None
            k = int(np.argmin((fy - start[0]) ** 2 + (fx - start[1]) ** 2))
            lead_in = [start]
            start = (int(fy[k]), int(fx[k]))
        g = {start: 0.0}
        parent = {start: None}
        heap = [(h[start], 0.0, start)]
        steps = [(dy, dx, CELL_M * math.hypot(dy, dx))
                 for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
        ny, nx = self.ny, self.nx
        end = None
        while heap:
            _f, gc, cur = heapq.heappop(heap)
            if gc > g[cur] + 1e-12:
                continue
            if goal[cur]:
                end = cur
                break
            cy, cx = cur
            for dy, dx, length in steps:
                y, x = cy + dy, cx + dx
                if not (0 <= y < ny and 0 <= x < nx) or not free[y, x]:
                    continue
                if dy and dx and not (free[cy, x] and free[y, cx]):
                    continue                                  # no corner cutting
                ng = gc + length * (1.0 + 0.5 * (penalty[cy, cx] + penalty[y, x]))
                nxt = (y, x)
                if ng < g.get(nxt, math.inf):
                    g[nxt] = ng
                    parent[nxt] = cur
                    heapq.heappush(heap, (ng + h[y, x], ng, nxt))
        if end is None:
            return None
        cells = []
        while end is not None:
            cells.append(end)
            end = parent[end]
        cells = lead_in + cells[::-1]
        pts = np.array([self.centre(*c) for c in cells])
        pts[0] = start_xy
        return thin(pts, WAYPOINT_SPACING_M)


def thin(pts, spacing):
    """Keep a point every `spacing` metres of path, always the first and last."""
    keep = [pts[0]]
    run = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        run += float(np.linalg.norm(b - a))
        if run >= spacing:
            keep.append(b)
            run = 0.0
    if np.linalg.norm(keep[-1] - pts[-1]) > 1e-9:
        if len(keep) > 1 and np.linalg.norm(keep[-1] - pts[-1]) < 0.4 * spacing:
            keep[-1] = pts[-1]
        else:
            keep.append(pts[-1])
    return np.array(keep)
