"""Live top-down map window for demo.py: where the drone is on the whole map.

Left panel: the full printed floor map (campus_map.png or map_texture.png,
the same image MuJoCo puts on the floor) with the drone, its heading, its
flown trail and the current target. Right panel: the same map zoomed to a
few metres around the drone, so nearby buildings and their names are
readable.

Display only: it reads fc.position()/attitude()/traj and never writes to
the controller. Map extents come from the same files the scene is built
from, so a dot on this map is exactly where the drone is in the sim.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MODELS = ROOT / "01_simulation" / "models"
CITY_MAP = MODELS / "map_texture.png"
CITY_HALF_M = 10.0                       # make_city.HALF_WORLD, scene.xml floor size
CAMPUS_MAP = MODELS / "campus" / "campus_map.png"
CAMPUS_JSON = MODELS / "campus" / "campus_world.json"

ZOOM_HALF_M = {"city": 4.0, "campus": 8.0}   # right panel: +- this many m around the drone
TRAIL_STEP_M = 0.05         # add a trail point every 5 cm flown
REDRAW_S = 0.15
FIGSIZE = (12.5, 6.6)
DPI = 100


def map_for(world_name: str):
    """(image path, extent [xmin, xmax, ymin, ymax] in world metres, title)."""
    if world_name == "campus":
        meta = json.loads(CAMPUS_JSON.read_text(encoding="utf-8"))
        cx, cy = meta["floor_centre"]
        h = meta["floor_half"]
        return (CAMPUS_MAP, (cx - h, cx + h, cy - h, cy + h),
                f"Amrita Coimbatore campus (1:{meta['scale']:g}, OpenStreetMap)")
    return CITY_MAP, (-CITY_HALF_M, CITY_HALF_M, -CITY_HALF_M, CITY_HALF_M), "Test city"


def yaw_deg(q) -> float:
    """Heading of body +x in the world frame (Hamilton [w, x, y, z])."""
    w, x, y, z = q
    return float(np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def target_of(fc):
    traj = getattr(fc, "traj", None)
    if traj is None:
        return None
    return np.asarray(traj.position(traj.t_total), dtype=float)


class MiniMap:
    def __init__(self, world_name: str = "city", show: bool = True):
        import matplotlib.pyplot as plt
        from PIL import Image

        path, extent, title = map_for(world_name)
        img = Image.open(path).convert("RGB")
        overview = np.asarray(img.resize((1024, 1024)))
        detail = np.asarray(img)
        self.extent = extent
        self.zoom = ZOOM_HALF_M.get(world_name, 4.0)
        self.trail: list[tuple[float, float]] = []

        self.fig, (self.ax_all, self.ax_zoom) = plt.subplots(
            1, 2, figsize=FIGSIZE, dpi=DPI, facecolor="#0b0e14",
            gridspec_kw={"width_ratios": [1, 1], "wspace": 0.08})
        self.fig.subplots_adjust(left=0.04, right=0.98, top=0.90, bottom=0.06)
        # the full-resolution image is only ever drawn as a small crop around
        # the drone (drawing all 4096 px every frame took ~4 s per redraw)
        self.detail = detail
        self.px_per_m = detail.shape[1] / (extent[1] - extent[0])
        self.img_all = self.ax_all.imshow(overview, extent=extent, origin="upper",
                                          interpolation="bilinear")
        self.img_zoom = self.ax_zoom.imshow(detail[:2, :2], extent=extent, origin="upper",
                                            interpolation="bilinear")
        for ax in (self.ax_all, self.ax_zoom):
            ax.set_aspect("equal")
            ax.tick_params(colors="#899bb0", labelsize=7)
            for s in ax.spines.values():
                s.set_color("#2a364f")
        self.ax_all.set_xlim(extent[0], extent[1])
        self.ax_all.set_ylim(extent[2], extent[3])
        self.ax_all.set_title("WHOLE MAP", color="#cdd6f4", fontsize=10, fontweight="bold")
        self.ax_zoom.set_title(f"CLOSE-UP (±{self.zoom:g} m)", color="#cdd6f4",
                               fontsize=10, fontweight="bold")
        self.fig.suptitle(title + "   -   N is up, +x east (forward), +y north (left)",
                          color="#00e5ff", fontsize=10)

        # per panel: trail, target line, target marker, drone marker, zoom box
        self.art = []
        self.routes = []      # planned "go to" route, dotted white
        for ax, size in ((self.ax_all, 12), (self.ax_zoom, 20)):
            route, = ax.plot([], [], color="#ffffff", lw=2.2, ls=":", alpha=0.9)
            trail, = ax.plot([], [], color="#ff2bd6", lw=1.6, alpha=0.85)
            tline, = ax.plot([], [], color="#ffb000", lw=1.2, ls="--")
            tmark, = ax.plot([], [], marker="X", ms=size * 0.7, color="#ffb000",
                             mec="black", ls="none")
            drone, = ax.plot([], [], marker=(3, 0, 0), ms=size, color="#ff1f1f",
                             mec="white", mew=1.5, ls="none", zorder=10)
            self.art.append((trail, tline, tmark, drone))
            self.routes.append(route)
        from matplotlib.patches import Rectangle
        self.zoom_box = Rectangle((0, 0), 2 * self.zoom, 2 * self.zoom, fill=False,
                                  ec="#00e5ff", lw=1.2)
        self.ax_all.add_patch(self.zoom_box)
        self.info = self.fig.text(0.04, 0.015, "", family="monospace", fontsize=9,
                                  color="#00e5ff")
        # Blitting: the whole-map image, titles and axes are drawn once and
        # cached; each update repaints only the moving parts (drone, trail,
        # target, zoom crop, readout). A full redraw costs ~200 ms, which
        # would stall the real-time sim loop.
        self.ax_zoom.tick_params(bottom=False, left=False, labelbottom=False, labelleft=False)
        # The close-up is drawn in drone-centred coordinates with FIXED axis
        # limits: moving the limits every frame marks the figure stale and
        # Tk then redraws the whole window (~150 ms) on top of the blit.
        self.ax_zoom.set_xlim(-self.zoom, self.zoom)
        self.ax_zoom.set_ylim(-self.zoom, self.zoom)
        self.ax_zoom.set_autoscale_on(False)
        self.ax_all.set_autoscale_on(False)
        self.animated = [self.img_zoom, self.zoom_box, self.info]
        self.animated += [a for group in self.art for a in group] + self.routes
        for a in self.animated:
            a.set_animated(True)
        self._bg = None
        self.fig.canvas.mpl_connect("draw_event", self._on_draw)
        if self.fig.canvas.manager is not None:
            try:
                self.fig.canvas.manager.set_window_title("Drone Map - where am I?")
            except Exception:
                pass
        import matplotlib
        if show and matplotlib.get_backend().lower() != "agg":
            self.fig.show()

    def update(self, p, q, target=None, status: str = "", nearest=None, route=None):
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        offsets = ((0.0, 0.0), (x, y))           # whole map: world coords; close-up: drone-centred
        for r, (ox, oy) in zip(self.routes, offsets):
            if route is None:
                r.set_data([], [])
            else:
                r.set_data(route[:, 0] - ox, route[:, 1] - oy)
        if not self.trail or np.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > TRAIL_STEP_M:
            self.trail.append((x, y))
        tx, ty = np.array(self.trail).T
        # (3, 0, a): triangle pointing up (+y) at a=0, so rotate by yaw-90
        marker = (3, 0, yaw_deg(q) - 90.0)
        for (trail, tline, tmark, drone), (ox, oy) in zip(self.art, offsets):
            trail.set_data(tx - ox, ty - oy)
            drone.set_data([x - ox], [y - oy])
            drone.set_marker(marker)
            if target is not None and np.hypot(target[0] - x, target[1] - y) > 0.03:
                if route is None:                 # a route draws its own line
                    tline.set_data([x - ox, target[0] - ox], [y - oy, target[1] - oy])
                else:
                    tline.set_data([], [])
                tmark.set_data([target[0] - ox], [target[1] - oy])
            else:
                tline.set_data([], [])
                tmark.set_data([], [])
        self._crop_zoom(x, y)
        self.zoom_box.set_xy((x - self.zoom, y - self.zoom))
        near = f"   nearest: {nearest[1]} {nearest[0]:.2f} m" if nearest else ""
        self.info.set_text(f"DRONE x={x:+6.2f}  y={y:+6.2f}  alt={z:5.2f} m   "
                           f"heading={yaw_deg(q):+5.0f}°   {status}{near}")
        canvas = self.fig.canvas
        if self._bg is None:
            canvas.draw()                      # fires _on_draw: caches bg, paints movers
        else:
            canvas.restore_region(self._bg)
            self._draw_animated()
            canvas.blit(self.fig.bbox)
        # the blit already shows the change; without this, the dashboard's
        # plt.pause() sees a "stale" figure and does a full ~200 ms redraw
        self.fig.stale = False
        canvas.flush_events()

    def _on_draw(self, _event):
        """Any full redraw (first show, window resize): re-cache the static
        background, then paint the moving parts on top."""
        self._bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self._draw_animated()

    def _draw_animated(self):
        for a in self.animated:
            self.fig.draw_artist(a)

    def _crop_zoom(self, x, y):
        """Show only the pixels of the full map within the zoom window."""
        x0, x1, y0, y1 = self.extent
        n_rows, n_cols = self.detail.shape[:2]
        h = self.zoom * 1.05
        c0 = int(np.clip((x - h - x0) * self.px_per_m, 0, n_cols - 1))
        c1 = int(np.clip((x + h - x0) * self.px_per_m, c0 + 1, n_cols))
        r0 = int(np.clip((y1 - (y + h)) * self.px_per_m, 0, n_rows - 1))   # row 0 = top (y1)
        r1 = int(np.clip((y1 - (y - h)) * self.px_per_m, r0 + 1, n_rows))
        self.img_zoom.set_data(self.detail[r0:r1, c0:c1])
        self.img_zoom.set_extent((x0 + c0 / self.px_per_m - x, x0 + c1 / self.px_per_m - x,
                                  y1 - r1 / self.px_per_m - y, y1 - r0 / self.px_per_m - y))

    def update_from(self, fc):
        traj = getattr(fc, "traj", None)
        route = traj.samples() if hasattr(traj, "samples") else None
        self.update(fc.position(), fc.attitude(), target_of(fc), fc.status(),
                    fc.world.nearest(fc.position()), route)

    def save(self, path):
        for a in self.animated:                # savefig skips animated artists
            a.set_animated(False)
        self.fig.savefig(path, dpi=90, facecolor=self.fig.get_facecolor())
        for a in self.animated:
            a.set_animated(True)
        self._bg = None
