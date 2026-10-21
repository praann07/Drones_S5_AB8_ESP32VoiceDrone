"""Live flight-status graphs for demo.py: altitude, speed, distance to the
nearest building, x/y position and a command timeline, over the last minute.

Why a separate window: the Flight Telemetry HUD charts controller quality
(attitude error, position error, control effort over 8 s). These charts show
what the drone is DOING (where it is, how fast, how close to a wall, what it
was told), over a longer history. Same idea as minimap.py: display only, it
reads fc and never writes to the controller.

The window is blitted (same technique as minimap.py): axes, grid, labels and
reference lines are drawn once and cached, and each update repaints only the
moving lines. A full redraw of this figure costs about 0.6 s on the dev PC,
which would stall the real-time sim loop. So the x axis is "seconds ago"
(fixed, -60..0) and the y limits only ever grow (growing forces one redraw).

StatusHistory has no matplotlib in it, so the recording logic is testable
without a display. Samples are taken every SAMPLE_DT_S of SIMULATION time (not
wall time), so a faster-than-real-time run records the same flight.
"""
from __future__ import annotations

import csv
from collections import deque
from pathlib import Path

import numpy as np

SAMPLE_DT_S = 0.1            # at most 10 samples per simulated second
HISTORY_S = 60.0             # the charts show the last minute
REDRAW_S = 0.5               # window repaint cadence (2 Hz): a 60 s history does not need more
CRUISE_MPS = 1.0             # speed reference lines
GOTO_CRUISE_MPS = 2.0
SAFETY_MARGIN_M = 0.3        # city.SAFETY_MARGIN_M
PLANNER_CLEARANCE_M = 0.45   # route_planner.CLEARANCE_M
MIN_CLEARANCE_M = 0.3        # flight.MIN_CLEARANCE_M ("down" stops this far above a surface)
WALL_CHART_MAX_M = 8.0       # the wall-distance chart never grows past this

COLUMNS = ("t", "x", "y", "z", "surface_z", "hspeed", "vz", "nearest_m", "phase", "command")
EVENT_COLUMNS = ("t", "kind", "text")
BAD_KINDS = ("heard_miss", "no_route")          # a phrase not understood, a go-to with no route

# palette shared with the HUD in demo.py
BG, PANEL, GRID, SPINE = "#0b0e14", "#121722", "#232d3f", "#2a364f"
TEXT, TICK = "#cdd6f4", "#899bb0"
CYAN, CORAL, GREEN, AMBER, VIOLET = "#00e5ff", "#ff5370", "#22da6e", "#ffb86c", "#bd93f9"

STATES = ("landed", "landing", "hover", "moving", "go to")


def state_label(phase: str, command: str) -> str:
    """One word for the timeline strip."""
    if phase == "landed":
        return "landed"
    if phase == "landing":
        return "landing"
    if command in ("hover", "stop"):
        return "hover"
    return "go to" if command == "goto" else "moving"


class StatusHistory:
    def __init__(self, maxlen: int = int(HISTORY_S / SAMPLE_DT_S)):
        self.window = deque(maxlen=maxlen)   # what the charts draw
        self.rows: list[tuple] = []          # the whole session, for the CSV
        self.events: list[tuple] = []        # (t, kind, text)
        self._next_t: float | None = None

    def record(self, fc, t: float) -> bool:
        """Store one sample if SAMPLE_DT_S of simulation time has passed."""
        if self._next_t is not None and t + 1e-9 < self._next_t:
            return False
        self._next_t = t + SAMPLE_DT_S if self._next_t is None else max(self._next_t + SAMPLE_DT_S, t)
        p, v = fc.position(), fc.velocity()
        row = (float(t), float(p[0]), float(p[1]), float(p[2]),
               float(fc.world.surface_under(p[0], p[1])[0]),
               float(np.hypot(v[0], v[1])), float(v[2]),
               float(fc.world.nearest(p)[0]), fc.phase, fc.command)
        self.window.append(row)
        self.rows.append(row)
        return True

    def event(self, t: float, kind: str, text: str) -> None:
        self.events.append((float(t), kind, text))

    def column(self, name: str) -> np.ndarray:
        i = COLUMNS.index(name)
        return np.array([r[i] for r in self.window])

    def save_csv(self, path) -> Path:
        """Samples to `path`, events next to it as `<name>_events.csv`."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(COLUMNS)
            w.writerows(self.rows)
        with open(path.with_name(path.stem + "_events.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(EVENT_COLUMNS)
            w.writerows(self.events)
        return path


def grow_ylim(ax, lo: float, hi: float) -> bool:
    """Widen the y limits (with headroom) if the data left them. Never shrinks:
    a changed limit forces a full redraw, so it must be rare. True if changed."""
    cur_lo, cur_hi = ax.get_ylim()
    if lo >= cur_lo and hi <= cur_hi:
        return False
    pad = 0.25 * (hi - lo + 1.0)
    ax.set_ylim(cur_lo if lo >= cur_lo else lo - pad, cur_hi if hi <= cur_hi else hi + pad)
    return True


class StatusGraphs:
    def __init__(self, show: bool = True):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button

        self.hist = StatusHistory()
        self.paused = False
        self._bg = None
        self._labels: list = []              # one Text per command, written on the timeline

        self.fig = plt.figure(figsize=(7.4, 9.0), facecolor=BG)
        gs = self.fig.add_gridspec(5, 1, hspace=0.34, left=0.11, right=0.96, top=0.93,
                                   bottom=0.10, height_ratios=[1, 1, 1, 1, 0.9])
        axes = []
        for i in range(5):
            ax = self.fig.add_subplot(gs[i, 0], facecolor=PANEL, sharex=axes[0] if axes else None)
            ax.tick_params(colors=TICK, labelsize=8, labelbottom=(i == 4))
            ax.grid(True, color=GRID, linestyle="--", linewidth=0.7, alpha=0.8)
            for s in ax.spines.values():
                s.set_color(SPINE)
            ax.set_autoscale_on(False)
            axes.append(ax)
        self.axes = axes
        self.ax_alt, self.ax_spd, self.ax_near, self.ax_pos, self.ax_tl = axes
        self.ax_tl.set_xlim(-HISTORY_S, 0.0)
        self.ax_tl.set_xticks(np.arange(-HISTORY_S, 1, 10))
        self.ax_alt.set_ylim(-0.1, 2.0)
        self.ax_spd.set_ylim(-0.8, 2.4)
        self.ax_near.set_ylim(0.0, 2.5)
        self.ax_pos.set_ylim(-2.0, 2.0)
        self.ax_tl.set_ylim(-0.6, len(STATES) - 0.4)
        self.fig.suptitle("FLIGHT STATUS   (last 60 s)", color=CYAN, fontsize=10, fontweight="bold")

        def label(ax, text):
            ax.set_ylabel(text, color=TEXT, fontsize=8.5, fontweight="bold")

        def line(ax, **kw):
            ln, = ax.plot([], [], **kw)
            return ln

        def ref(ax, y, color, text, va="bottom"):
            ax.axhline(y, color=color, ls=":", lw=1.0, alpha=0.9)
            ax.text(0.995, y, text, transform=ax.get_yaxis_transform(), color=color,
                    fontsize=7, ha="right", va=va)

        label(self.ax_alt, "Altitude (m)")
        self.l_alt = line(self.ax_alt, color=CYAN, lw=1.8, label="drone")
        self.l_surf = line(self.ax_alt, color=VIOLET, lw=1.2, label="surface below")
        self.l_floor = line(self.ax_alt, color=AMBER, lw=1.0, ls=":",
                            label=f"{MIN_CLEARANCE_M:g} m above surface")

        label(self.ax_spd, "Speed (m/s)")
        self.l_hs = line(self.ax_spd, color=GREEN, lw=1.8, label="horizontal")
        self.l_vz = line(self.ax_spd, color=CORAL, lw=1.2, label="vertical")
        ref(self.ax_spd, CRUISE_MPS, AMBER, "cruise")
        ref(self.ax_spd, GOTO_CRUISE_MPS, AMBER, "go to")

        label(self.ax_near, "Wall distance (m)")
        self.l_near = line(self.ax_near, color=CYAN, lw=1.8)
        ref(self.ax_near, SAFETY_MARGIN_M, CORAL, "safety margin", va="top")
        ref(self.ax_near, PLANNER_CLEARANCE_M, AMBER, "planner clearance")

        label(self.ax_pos, "Position (m)")
        self.l_x = line(self.ax_pos, color=CYAN, lw=1.6, label="x (east)")
        self.l_y = line(self.ax_pos, color=GREEN, lw=1.6, label="y (north)")

        label(self.ax_tl, "State / commands")
        self.l_state, = self.ax_tl.step([], [], where="post", color=VIOLET, lw=1.6)
        self.ax_tl.set_yticks(range(len(STATES)))
        self.ax_tl.set_yticklabels(STATES, fontsize=7.5)
        self.ax_tl.set_xlabel("seconds ago", color=TEXT, fontsize=8.5, fontweight="bold")
        self.l_miss = line(self.ax_tl, color=CORAL, marker="x", ms=8, mew=2, ls="none")

        # one collection per chart for ALL command lines (dashed amber) and one
        # for all misses (solid red), so the cost does not grow with the events
        def vcoll(ax, color, style):
            return ax.vlines([], 0, 1, colors=color, linestyles=style, linewidth=0.9, alpha=0.55,
                             transform=ax.get_xaxis_transform())

        self.cmd_lines = [vcoll(ax, AMBER, "--") for ax in axes]
        self.bad_lines = [vcoll(ax, CORAL, "-") for ax in axes]

        for ax in (self.ax_alt, self.ax_spd, self.ax_pos):
            for t in ax.legend(loc="upper left", fontsize=7, frameon=False, ncol=3).get_texts():
                t.set_color(TEXT)

        self.info = self.fig.text(0.11, 0.945, "", family="monospace", fontsize=8, color=CYAN)

        ax_btn = self.fig.add_axes((0.36, 0.02, 0.28, 0.04))
        self.btn = Button(ax_btn, "PAUSE", color="#1b2333", hovercolor="#2a4a5f")
        self.btn.label.set_color(CYAN)
        self.btn.label.set_fontsize(8)
        self.btn.label.set_fontweight("bold")
        self.btn.on_clicked(self._toggle_pause)

        self._moving = [self.l_alt, self.l_surf, self.l_floor, self.l_hs, self.l_vz, self.l_near,
                        self.l_x, self.l_y, self.l_state, self.l_miss, self.info,
                        *self.cmd_lines, *self.bad_lines]
        for a in self._moving:
            a.set_animated(True)
        self.fig.canvas.mpl_connect("draw_event", self._on_draw)

        if self.fig.canvas.manager is not None:
            try:
                self.fig.canvas.manager.set_window_title("Flight Status - altitude, speed, distance, position")
            except Exception:
                pass
        import matplotlib
        if show and matplotlib.get_backend().lower() != "agg":
            self.fig.show()

    # -- recording (cheap, called every frame) ------------------------------
    def record(self, fc, t: float) -> bool:
        return self.hist.record(fc, t)

    def event(self, t: float, kind: str, text: str) -> None:
        self.hist.event(t, kind, text)

    # -- drawing (called at REDRAW_S) ---------------------------------------
    def _toggle_pause(self, _event=None):
        self.paused = not self.paused
        self.btn.label.set_text("RESUME" if self.paused else "PAUSE")
        self._bg = None                      # the button text is part of the cached background
        self.fig.canvas.draw_idle()

    def update(self) -> None:
        h = self.hist
        if self.paused or not h.window:
            self.fig.canvas.flush_events()
            return
        t_now = h.window[-1][0]
        t = h.column("t") - t_now                 # seconds ago, -60 .. 0
        z, sz = h.column("z"), h.column("surface_z")
        hs, vz, near = h.column("hspeed"), h.column("vz"), h.column("nearest_m")
        x, y = h.column("x"), h.column("y")
        self.l_alt.set_data(t, z)
        self.l_surf.set_data(t, sz)
        self.l_floor.set_data(t, sz + MIN_CLEARANCE_M)
        self.l_hs.set_data(t, hs)
        self.l_vz.set_data(t, vz)
        self.l_near.set_data(t, near)
        self.l_x.set_data(t, x)
        self.l_y.set_data(t, y)
        self.l_state.set_data(t, [STATES.index(state_label(r[8], r[9])) for r in h.window])

        stale = self._place_events(t_now)
        for ax, lo, hi in ((self.ax_alt, z.min(), z.max()),
                           (self.ax_spd, min(hs.min(), vz.min()), max(hs.max(), vz.max())),
                           (self.ax_near, 0.0, min(near.max(), WALL_CHART_MAX_M)),
                           (self.ax_pos, min(x.min(), y.min()), max(x.max(), y.max()))):
            stale |= grow_ylim(ax, float(lo), float(hi))

        p = h.window[-1]
        self.info.set_text(f"t={p[0]:6.1f}s  x={p[1]:+6.2f} y={p[2]:+6.2f} alt={p[3]:5.2f} m  "
                           f"v={p[5]:4.2f} m/s  wall={p[7]:5.2f} m  [{state_label(p[8], p[9])}]")
        canvas = self.fig.canvas
        if self._bg is None or stale:
            canvas.draw()                    # fires _on_draw: caches the background, paints movers
        else:
            canvas.restore_region(self._bg)
            self._draw_moving()
            canvas.blit(self.fig.bbox)
        self.fig.stale = False               # the blit already shows the change
        canvas.flush_events()

    def _place_events(self, t_now: float) -> bool:
        """Command lines on every chart, labels on the timeline, crosses for
        phrases that were not understood or go-tos with no route. True if a
        new label artist was added (the cached background must be redone)."""
        ev = [e for e in self.hist.events if e[0] - t_now >= -HISTORY_S - 1.0]
        cmds = [e for e in ev if e[1] not in BAD_KINDS]
        bad = [e[0] - t_now for e in ev if e[1] in BAD_KINDS]
        for coll in self.cmd_lines:
            coll.set_segments([[(e[0] - t_now, 0.0), (e[0] - t_now, 1.0)] for e in cmds])
        for coll in self.bad_lines:
            coll.set_segments([[(x, 0.0), (x, 1.0)] for x in bad])
        self.l_miss.set_data(bad, [len(STATES) - 1] * len(bad))
        added = False
        while len(self._labels) < len(cmds):
            self._labels.append(self.ax_tl.text(0, 0.0, "", color=AMBER, fontsize=6.5, rotation=90,
                                                va="bottom", ha="left", clip_on=True, animated=True))
            added = True
        for tx, (t_ev, _kind, text) in zip(self._labels, cmds):
            tx.set_position((t_ev - t_now, 0.0))
            tx.set_text(" " + text[:14])
        for tx in self._labels[len(cmds):]:
            tx.set_text("")
        return added

    def _on_draw(self, _event):
        """Any full redraw (first show, resize, new limits): re-cache the static
        background, then paint the moving parts on top."""
        self._bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self._draw_moving()

    def _draw_moving(self):
        for a in self._moving + self._labels:
            self.fig.draw_artist(a)

    def save(self, path) -> None:
        artists = self._moving + self._labels
        for a in artists:                    # savefig skips animated artists
            a.set_animated(False)
        self.fig.savefig(path, dpi=90, facecolor=self.fig.get_facecolor())
        for a in artists:
            a.set_animated(True)

    def save_csv(self, path) -> Path:
        return self.hist.save_csv(path)
