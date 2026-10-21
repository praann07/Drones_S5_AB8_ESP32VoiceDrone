"""Autonomous patrol among MOVING obstacles, flown through the Stage D cascade.

The drone patrols a 4-waypoint loop around the plaza with no operator input.
Four obstacles move on closed-form paths (two pillars, two spheres) and the
city's static buildings/tower are avoided too; the east leg passes straight
through the obstacle tower. The avoidance planner (04_control/avoidance.py)
generates a smooth, curved reference that the SAME outer PD loop + identified-
model inner loop used everywhere else then track.

Every mission is scored against the TRUE obstacle positions (not the planner's
belief) and compared with a straight-line baseline: the same cascade and
patrol, but a planner that ignores obstacles.

    .venv\\Scripts\\python.exe 05_voice_interface\\run_autonomous.py              # stats + plot
    .venv\\Scripts\\python.exe 05_voice_interface\\run_autonomous.py --render     # + 3D GIF
    .venv\\Scripts\\python.exe 05_voice_interface\\run_autonomous.py --live       # live 3D viewer

Outputs: data/processed/autonomy/{autonomy_results.csv, autonomy_paths.png,
autonomous_patrol.gif}
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
for _p in ("05_voice_interface", "01_simulation", "01_simulation/models",
           "04_control", "02_identification", "03_validation"):
    sys.path.insert(0, str(ROOT / _p))

import mujoco  # noqa: E402
from flight import DT, FlightController, IDENTITY_Q  # noqa: E402
from avoidance import AvoidancePlanner, DRONE_RADIUS_M, clearance  # noqa: E402
from obstacles import (SCENE_AUTO_PATH, default_moving_obstacles,  # noqa: E402
                       default_waypoints, city_static_obstacles, write_scene_auto)

OUT = ROOT / "data" / "processed" / "autonomy"
START = np.array([-2.8, -2.8, 1.0])
ARENA_CAM_Z = 10.5                     # matches the 'arena' camera in scene_auto.xml
DURATION_S = 75.0
SAMPLE_EVERY = 10                      # 50 Hz trace
MIN_WAYPOINTS_FOR_SUCCESS = 4          # one full lap
DIVERGE_TILT_DEG = 60.0
STALL_SPEED_MPS = 0.1                  # slower than this = not making progress
MAX_STALL_S = 8.0                      # a mission that parks longer than this fails


class Mission:
    """One autonomous flight. Steps the world (moves the mocap obstacles),
    the planner and the cascade together; used headless, for rendering and
    for the live viewer so all three fly the identical simulation."""

    def __init__(self, seed: int = 0, avoid: bool = True):
        write_scene_auto(default_moving_obstacles(seed))
        self.seed, self.avoid = seed, avoid
        self.moving = default_moving_obstacles(seed)
        self.static = city_static_obstacles()
        self.fc = FlightController(scene_path=SCENE_AUTO_PATH)
        self.fc.reset(p0=START)
        self.mocap_ids = []
        for o in self.moving:
            bid = mujoco.mj_name2id(self.fc.model, mujoco.mjtObj.mjOBJ_BODY, o.name)
            self.mocap_ids.append(int(self.fc.model.body_mocapid[bid]))
        # The planner sees every obstacle; the baseline sees none.
        self.planner = AvoidancePlanner(
            default_waypoints(), (self.moving + self.static) if avoid else [],
            p_start=START)
        self.fc.reference = self.planner
        self._place_obstacles(0.0)
        self.step_count = 0
        self.trace: list[dict] = []
        self.leg_paths: list[list[np.ndarray]] = [[START.copy()]]
        self.leg_chords: list[float] = [float(np.linalg.norm(default_waypoints()[0] - START))]
        self._last_reached = 0
        self.min_clear = float("inf")
        self.collision_steps = 0
        self.max_tilt = 0.0
        self.max_speed = 0.0
        self.diverged = False

    def _place_obstacles(self, t: float) -> None:
        for o, mid in zip(self.moving, self.mocap_ids):
            self.fc.data.mocap_pos[mid] = o.position(t)

    def step(self) -> None:
        fc = self.fc
        self._place_obstacles(fc.data.time)
        fc.step()
        self.step_count += 1
        t = fc.data.time
        p = fc.position()
        if self.step_count % SAMPLE_EVERY:
            return
        # score against TRUE obstacle positions at the current time
        c = clearance(p, self.moving + self.static, t)
        self.min_clear = min(self.min_clear, c)
        if c < 0.0:
            self.collision_steps += SAMPLE_EVERY
        v = fc.velocity()
        tilt = fc.attitude_error_deg(IDENTITY_Q)
        self.max_tilt = max(self.max_tilt, tilt)
        self.max_speed = max(self.max_speed, float(np.linalg.norm(v)))
        if tilt > DIVERGE_TILT_DEG or not np.isfinite(p).all():
            self.diverged = True
        # per-leg path length for the straightness metric
        if self.planner.waypoints_reached != self._last_reached:
            self._last_reached = self.planner.waypoints_reached
            wps = default_waypoints()
            self.leg_paths.append([p.copy()])
            nxt = wps[self.planner.wp_idx]
            self.leg_chords.append(float(np.linalg.norm(nxt - p)))
        self.leg_paths[-1].append(p.copy())
        row = {"t": t, "px": p[0], "py": p[1], "pz": p[2],
               "speed": float(np.linalg.norm(v)), "tilt_deg": tilt, "clearance_m": c,
               "ref_x": self.planner.pr[0], "ref_y": self.planner.pr[1],
               "ref_z": self.planner.pr[2], "wp_idx": self.planner.wp_idx}
        for o in self.moving:
            q = o.position(t)
            row[f"{o.name}_x"], row[f"{o.name}_y"], row[f"{o.name}_z"] = q
        self.trace.append(row)

    def run(self, duration_s: float = DURATION_S) -> "Mission":
        for _ in range(int(round(duration_s / DT))):
            self.step()
            if self.diverged:
                break
        return self

    # --------------------------------------------------------------
    def leg_straightness(self) -> list[float]:
        """path length / straight-line chord for every COMPLETED leg."""
        ratios = []
        for path in self.leg_paths[:-1]:
            pts = np.array(path)
            # chord = straight line between the leg's own endpoints (legs end
            # reach_m short of the waypoint), so a straight flight is 1.00
            chord = float(np.linalg.norm(pts[-1] - pts[0])) if len(pts) > 1 else 0.0
            if chord > 0.5:
                ratios.append(float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)) / chord))
        return ratios

    def stall_stats(self) -> tuple[float, float]:
        """(total seconds, longest contiguous seconds) below STALL_SPEED_MPS,
        ignoring the first second (the drone starts at rest)."""
        sp = np.array([r["speed"] for r in self.trace if r["t"] > 1.0])
        slow = sp < STALL_SPEED_MPS
        best = cur = 0
        for s_ in slow:
            cur = cur + 1 if s_ else 0
            best = max(best, cur)
        dt = SAMPLE_EVERY * DT
        return float(slow.sum() * dt), float(best * dt)

    def summary(self) -> dict:
        st = self.leg_straightness()
        reached = self.planner.waypoints_reached
        stall_total, stall_longest = self.stall_stats()
        success = (not self.diverged and self.collision_steps == 0
                   and reached >= MIN_WAYPOINTS_FOR_SUCCESS
                   and stall_longest <= MAX_STALL_S)
        return {
            "seed": self.seed, "mode": "avoid" if self.avoid else "straight-line",
            "success": success, "waypoints_reached": reached, "laps": self.planner.laps,
            "min_clearance_m": round(self.min_clear, 3),
            "collision_time_s": round(self.collision_steps * DT, 2),
            "mean_leg_path_ratio": round(float(np.mean(st)), 3) if st else float("nan"),
            "max_leg_path_ratio": round(float(np.max(st)), 3) if st else float("nan"),
            "stall_time_s": round(stall_total, 1), "longest_stall_s": round(stall_longest, 1),
            "stall_escapes": self.planner.escapes,
            "max_speed_mps": round(self.max_speed, 2), "max_tilt_deg": round(self.max_tilt, 1),
            "planner_emergencies": self.planner.emergency_count, "diverged": self.diverged,
        }


# ----------------------------------------------------------------------------
def plot_paths(avoid: Mission, base: Mission, path: Path) -> None:
    da, db = pd.DataFrame(avoid.trace), pd.DataFrame(base.trace)
    fig = plt.figure(figsize=(17, 6.2))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.25, 1.0, 1.0])
    axes = {"A": fig.add_subplot(gs[:, 0]), "B": fig.add_subplot(gs[0, 1]),
            "C": fig.add_subplot(gs[1, 1]), "D": fig.add_subplot(gs[0, 2]),
            "E": fig.add_subplot(gs[1, 2])}

    ax = axes["A"]
    for b in avoid.static:
        ax.add_patch(plt.Rectangle((b.lo[0], b.lo[1]), b.hi[0] - b.lo[0], b.hi[1] - b.lo[1],
                                   color="#8d99ae", alpha=0.55, lw=0))
    colors = ["#e63946", "#f4a261", "#d62f8a", "#7b2cbf"]
    for o, col in zip(avoid.moving, colors):
        ax.plot(da[f"{o.name}_x"], da[f"{o.name}_y"], color=col, alpha=0.25, lw=5,
                solid_capstyle="round", label=f"{o.name} (moving)")
    ax.plot(db.px, db.py, color="#999999", lw=1.6, ls="--", label="straight-line baseline")
    ax.plot(da.px, da.py, color="#118ab2", lw=2.2, label="autonomous (avoiding)")
    wps = np.array(default_waypoints())
    ax.scatter(wps[:, 0], wps[:, 1], marker="*", s=220, color="gold", edgecolor="k", zorder=5,
               label="waypoints")
    ax.scatter([START[0]], [START[1]], marker="s", color="green", zorder=5, label="start")
    ax.set_aspect("equal"); ax.set_xlim(-4.4, 5.8); ax.set_ylim(-4.4, 4.4)
    ax.set_title("Top-down: obstacle tower + 4 moving obstacles (faint = their paths)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.grid(alpha=0.25)
    ax.legend(loc="upper left", fontsize=7)

    for key, title, col, ylabel in (("B", "Clearance to nearest obstacle", "clearance_m", "m"),
                                    ("D", "Speed", "speed", "m/s"),
                                    ("C", "Tilt", "tilt_deg", "deg"),
                                    ("E", "Altitude", "pz", "m")):
        ax = axes[key]
        ax.plot(db.t, db[col], color="#999999", lw=1, label="straight-line")
        ax.plot(da.t, da[col], color="#118ab2", lw=1.4, label="autonomous")
        if col == "clearance_m":
            ax.axhline(0, color="r", lw=1.2, label="collision (0 m)")
            ax.set_ylim(-1.0, 3.0)
        ax.set_title(title, fontsize=10); ax.set_ylabel(ylabel); ax.set_xlabel("t (s)")
        ax.grid(alpha=0.3)
    axes["B"].legend(fontsize=7)
    sa, sb = avoid.summary(), base.summary()
    fig.suptitle(f"Autonomous patrol (seed {avoid.seed}):  avoiding -> min clearance "
                 f"{sa['min_clearance_m']} m, {sa['waypoints_reached']} waypoints, "
                 f"collision {sa['collision_time_s']} s   |   straight-line -> min clearance "
                 f"{sb['min_clearance_m']} m, collision {sb['collision_time_s']} s")
    fig.tight_layout()
    fig.savefig(path, dpi=105)
    plt.close(fig)


# ----------------------------------------------------------------------------
def render_gif(seed: int, path: Path, seconds: float = 45.0, fps: int = 10,
               size=(480, 360)) -> None:
    from PIL import Image, ImageDraw
    m = Mission(seed, avoid=True)
    renderer = mujoco.Renderer(m.fc.model, width=size[0], height=size[1])
    every = int(round(1.0 / (fps * DT)))
    frames, trail = [], []
    w0, h0 = size
    # arena camera is a fixed pinhole at (0, 0, ARENA_CAM_Z) looking straight down
    # (fovy 62 deg, +x right, +y up): project world points to its pixels so the
    # drone's recent PATH can be drawn on the top-down view.
    f_px = (h0 / 2) / np.tan(np.radians(31.0))
    def to_px(pt):
        zc = ARENA_CAM_Z - pt[2]
        return (w0 / 2 + f_px * pt[0] / zc, h0 / 2 - f_px * pt[1] / zc)
    for k in range(int(round(seconds / DT))):
        m.step()
        if (k + 1) % every:
            continue
        trail.append(m.fc.position().copy())
        imgs = []
        for cam in ("arena", "chase"):
            renderer.update_scene(m.fc.data, camera=cam)
            imgs.append(Image.fromarray(renderer.render()).convert("RGB"))
        w, h = size
        frame = Image.new("RGB", (w * 2, h))
        frame.paste(imgs[0], (0, 0)); frame.paste(imgs[1], (w, 0))
        d = ImageDraw.Draw(frame)
        pts = [to_px(q) for q in trail[-150:]]          # last ~15 s of path
        if len(pts) > 1:
            d.line(pts, fill=(0, 200, 255), width=2)
        cx, cy = to_px(trail[-1])
        d.ellipse((cx - 9, cy - 9, cx + 9, cy + 9), outline=(255, 255, 0), width=2)
        c = m.trace[-1]["clearance_m"] if m.trace else float("nan")
        d.rectangle((0, 0, 250, 36), fill=(0, 0, 0))
        d.text((6, 4), f"AUTONOMOUS  t={m.fc.data.time:5.1f}s  wp {m.planner.wp_idx}  "
                       f"reached {m.planner.waypoints_reached}", fill=(0, 255, 0))
        d.text((6, 20), f"speed {m.trace[-1]['speed']:.2f} m/s   clearance {c:+.2f} m",
               fill=(0, 255, 0) if c > 0 else (255, 80, 80))
        d.rectangle((w, 0, w + 110, 16), fill=(0, 0, 0))
        d.text((w + 6, 3), "chase view", fill=(0, 255, 0))
        d.rectangle((0, h - 16, 200, h), fill=(0, 0, 0))
        d.text((6, h - 13), "top-down: cyan = path, red ring = drone", fill=(255, 255, 255))
        frames.append(frame.quantize(colors=96, method=Image.MEDIANCUT))
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True)
    print(f"wrote {len(frames)} frames -> {path} ({path.stat().st_size / 1e6:.1f} MB)")


def live(seed: int) -> None:  # pragma: no cover - needs a display
    import mujoco.viewer as mjviewer
    m = Mission(seed, avoid=True)
    cid = mujoco.mj_name2id(m.fc.model, mujoco.mjtObj.mjOBJ_CAMERA, "arena_side")
    with mjviewer.launch_passive(m.fc.model, m.fc.data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = cid
        t0 = time.perf_counter()
        print("Autonomous patrol running - close the viewer window to stop.", flush=True)
        while viewer.is_running():
            # keep sim time ~ wall time: run whatever steps are owed, then draw
            owed = int((time.perf_counter() - t0) / DT) - m.step_count
            for _ in range(max(1, min(owed, 40))):
                m.step()
            viewer.sync()
            time.sleep(0.004)
    print(m.summary())


# ----------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5, help="number of obstacle-phase seeds")
    ap.add_argument("--duration", type=float, default=DURATION_S)
    ap.add_argument("--render", action="store_true", help="also write the 3D GIF")
    ap.add_argument("--live", action="store_true", help="live 3D viewer (needs a display)")
    args = ap.parse_args()
    if args.live:
        live(0)
        return

    OUT.mkdir(parents=True, exist_ok=True)
    rows, first = [], {}
    for seed in range(args.seeds):
        for avoid in (True, False):
            m = Mission(seed, avoid).run(args.duration)
            s = m.summary()
            rows.append(s)
            if seed == 0:
                first[avoid] = m
            print(f"seed {seed} {s['mode']:<13s} success={s['success']!s:<5} "
                  f"wps={s['waypoints_reached']:>2} min_clear={s['min_clearance_m']:+.3f} m "
                  f"collision={s['collision_time_s']:.2f}s stall_max={s['longest_stall_s']}s "
                  f"leg_ratio={s['mean_leg_path_ratio']} "
                  f"vmax={s['max_speed_mps']} tilt<={s['max_tilt_deg']} "
                  f"emerg={s['planner_emergencies']}", flush=True)
    df = pd.DataFrame(rows)
    df.to_csv(OUT / "autonomy_results.csv", index=False)
    plot_paths(first[True], first[False], OUT / "autonomy_paths.png")

    a, b = df[df["mode"] == "avoid"], df[df["mode"] == "straight-line"]
    print("\n" + "=" * 70 + "\nAUTONOMOUS PATROL AMONG MOVING OBSTACLES\n" + "=" * 70)
    print(f"avoiding     : {int(a.success.sum())}/{len(a)} missions collision-free, "
          f"min clearance {a.min_clearance_m.min():+.3f} m, mean leg path ratio "
          f"{a.mean_leg_path_ratio.mean():.2f} (1.00 = straight line), longest stall "
          f"{a.longest_stall_s.max():.1f} s")
    print(f"straight-line: {int(b.success.sum())}/{len(b)} missions collision-free, "
          f"min clearance {b.min_clearance_m.min():+.3f} m, total time inside obstacles "
          f"{b.collision_time_s.sum():.1f} s")
    print(f"wrote {OUT / 'autonomy_results.csv'} and autonomy_paths.png")
    if args.render:
        render_gif(0, OUT / "autonomous_patrol.gif")


if __name__ == "__main__":
    main()
