"""Record the campus demo as a video: take-off, cruise, STOP (the drone brakes
to rest), a turn, a routed flight to a named building and a landing.

Each frame shows the chase camera and the bird's-eye camera side by side,
the command that was "spoken", and a live speed trace underneath, so the
braking is visible as speed falling to zero when "stop" is given. The
braking numbers (distance and time to rest) are measured from the same run
and written next to the video.

Needs a window-less render only (no display, no microphone). The commands
are scripted text commands, the same ones the voice pipeline produces.

    .venv\\Scripts\\python.exe 05_voice_interface\\record_demo_video.py

Writes docs/recordings/campus_demo.mp4, campus_demo.gif, stop_metrics.json and
docs/images/stop_braking.png. With --plot-only it skips the rendering and only
redraws the braking plot and metrics (seconds instead of minutes).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
for _p in ("05_voice_interface", "04_control", "01_simulation", "01_simulation/models"):
    sys.path.insert(0, str(ROOT / _p))
import mujoco  # noqa: E402
from flight import DT, FlightController  # noqa: E402
from world import CAMPUS_SCENE, CampusWorld  # noqa: E402

OUT = ROOT / "docs" / "recordings"
IMAGES = ROOT / "docs" / "images"
FPS = 25
STEPS_PER_FRAME = int(round(1.0 / (FPS * DT)))
VIEW_W, VIEW_H = 640, 400
STRIP_H = 150
TRACE_WINDOW_S = 16.0
REST_SPEED = 0.05          # m/s below which the drone counts as stopped
GIF_WIDTH, GIF_EVERY = 640, 3

BG = (11, 14, 20)
GREEN, RED, CYAN, AMBER, GREY = (34, 218, 110), (255, 83, 112), (0, 229, 255), (255, 184, 108), (137, 155, 176)


def _font(size: int):
    for name in ("segoeui.ttf", "arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


FONT_S, FONT_M, FONT_L = _font(14), _font(18), _font(26)


class Recorder:
    def __init__(self, render: bool = True):
        self.render = render
        self.fc = FlightController(scene_path=CAMPUS_SCENE, world=CampusWorld(), continuous=True)
        self.renderer = mujoco.Renderer(self.fc.model, width=VIEW_W, height=VIEW_H)
        body = mujoco.mj_name2id(self.fc.model, mujoco.mjtObj.mjOBJ_BODY, "quad")
        self.cam_chase = self._tracking_camera(body, azimuth=215, elevation=-18, distance=1.6)
        self.cam_bird = self._tracking_camera(body, azimuth=90, elevation=-89, distance=5.5)
        self.frames: list[Image.Image] = []
        self.steps = 0
        self.t_s: list[float] = []
        self.speed: list[float] = []
        self.events: list[tuple[float, str, tuple]] = []      # (time, label, colour)
        self.caption = ("", GREEN)
        self.metrics: list[dict] = []

    @staticmethod
    def _tracking_camera(body: int, azimuth: float, elevation: float, distance: float):
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = body
        cam.azimuth, cam.elevation, cam.distance = azimuth, elevation, distance
        return cam

    @property
    def t(self) -> float:
        return self.steps * DT

    def say(self, command: str, colour=GREEN, label: str | None = None) -> None:
        self.caption = (f'"{command}"', colour)
        self.events.append((self.t, label or command, colour))

    def advance(self, seconds: float, until=None) -> None:
        for _ in range(int(round(seconds / DT))):
            self.fc.step()
            self.steps += 1
            if self.steps % STEPS_PER_FRAME == 0:
                v = self.fc.velocity()
                self.t_s.append(self.t)
                self.speed.append(float(np.linalg.norm(v[:2])))
                if self.render:
                    self.frames.append(self._compose())
            if until is not None and until():
                break

    # --- frame drawing ------------------------------------------------------
    def _compose(self) -> Image.Image:
        self.renderer.update_scene(self.fc.data, camera=self.cam_chase)
        chase = Image.fromarray(self.renderer.render()).convert("RGB")
        self.renderer.update_scene(self.fc.data, camera=self.cam_bird)
        bird = Image.fromarray(self.renderer.render()).convert("RGB")
        img = Image.new("RGB", (VIEW_W * 2, VIEW_H + STRIP_H), BG)
        img.paste(chase, (0, 0))
        img.paste(bird, (VIEW_W, 0))
        d = ImageDraw.Draw(img)
        d.text((10, 8), "CHASE VIEW", fill=GREY, font=FONT_S)
        d.text((VIEW_W + 10, 8), "BIRD'S-EYE VIEW", fill=GREY, font=FONT_S)

        text, colour = self.caption
        d.rounded_rectangle((VIEW_W - 170, 10, VIEW_W + 170, 56), radius=10, fill=(18, 23, 34), outline=colour, width=2)
        d.text((VIEW_W, 33), f"SAID  {text}", fill=colour, font=FONT_L, anchor="mm")

        speed = self.speed[-1] if self.speed else 0.0
        p = self.fc.position()
        state = self.fc.status()
        moving = speed > REST_SPEED
        braking = self.caption[0] == '"stop"' and moving
        tag, tcol = (("BRAKING", RED) if braking else ("MOVING", CYAN) if moving else ("AT REST", GREEN))
        d.rounded_rectangle((8, VIEW_H - 74, 470, VIEW_H - 8), radius=8, fill=(14, 18, 27), outline=(42, 54, 79))
        d.text((18, VIEW_H - 70), f"{tag}", fill=tcol, font=FONT_L)
        d.text((18, VIEW_H - 34),
               f"speed {speed:4.2f} m/s   x {p[0]:+6.2f}  y {p[1]:+6.2f}  z {p[2]:4.2f}   {state}",
               fill=(205, 214, 244), font=FONT_S)
        self._strip(d, 0, VIEW_H, VIEW_W * 2, STRIP_H)
        return img

    def _strip(self, d: ImageDraw.ImageDraw, x0: int, y0: int, w: int, h: int) -> None:
        pad_l, pad_r, pad_t, pad_b = 40, 16, 22, 24
        gx0, gx1, gy0, gy1 = x0 + pad_l, x0 + w - pad_r, y0 + pad_t, y0 + h - pad_b
        d.rectangle((gx0, gy0, gx1, gy1), fill=(18, 23, 34), outline=(42, 54, 79))
        t_end = self.t_s[-1] if self.t_s else 0.0
        t_start = max(0.0, t_end - TRACE_WINDOW_S)
        t_lo = t_end - TRACE_WINDOW_S if t_end > TRACE_WINDOW_S else 0.0
        vmax = 1.3

        def X(t):
            return gx0 + (t - t_lo) / TRACE_WINDOW_S * (gx1 - gx0)

        def Y(v):
            return gy1 - min(v, vmax) / vmax * (gy1 - gy0)

        for v in (0.0, 0.5, 1.0):
            d.line((gx0, Y(v), gx1, Y(v)), fill=(35, 45, 63))
            d.text((gx0 - 8, Y(v)), f"{v:.1f}", fill=GREY, font=FONT_S, anchor="rm")
        d.text((gx0, y0 + 3), "speed (m/s)", fill=GREY, font=FONT_S)
        d.text((gx1, gy1 + 5), "time (s)", fill=GREY, font=FONT_S, anchor="ra")
        for tick in range(int(np.ceil(t_lo / 5.0)) * 5, int(t_lo + TRACE_WINDOW_S) + 1, 5):
            d.line((X(tick), gy1, X(tick), gy1 + 4), fill=GREY)
            d.text((X(tick), gy1 + 5), f"{tick}", fill=GREY, font=FONT_S, anchor="ma")
        for t, label, col in self.events:
            if t_lo <= t <= t_end + 0.01:
                d.line((X(t), gy0, X(t), gy1), fill=col, width=1)
                d.text((X(t) + 4, gy0 + 1), label, fill=col, font=FONT_S)
        pts = [(X(t), Y(s)) for t, s in zip(self.t_s, self.speed) if t >= t_start]
        if len(pts) > 1:
            d.line(pts, fill=CYAN, width=2)

    # --- stop measurement ------------------------------------------------------
    def stop_and_measure(self, label: str, settle_s: float = 3.0) -> dict:
        """Say 'stop', then record how far and how long the drone takes to rest."""
        v0 = self.fc.velocity().copy()
        v0[2] = 0.0
        p0 = self.fc.position().copy()
        t0 = self.t
        self.fc.dispatch("stop")
        self.say("stop", RED, "stop")
        t_rest = {"t": None}

        def rested():
            if t_rest["t"] is None and np.linalg.norm(self.fc.velocity()[:2]) < REST_SPEED:
                t_rest["t"] = self.t
            return False

        self.advance(settle_s, until=rested)
        p1 = self.fc.position()
        m = {"move": label,
             "speed_at_stop_command_m_s": round(float(np.linalg.norm(v0)), 3),
             "time_to_rest_s": round((t_rest["t"] or self.t) - t0, 2),
             "braking_distance_m": round(float(np.linalg.norm((p1 - p0)[:2])), 3),
             "final_speed_m_s": round(float(np.linalg.norm(self.fc.velocity()[:2])), 4),
             "altitude_change_m": round(float(p1[2] - p0[2]), 3)}
        self.metrics.append(m)
        return m


def nearest_place(fc: FlightController, min_dist: float = 4.0) -> str:
    p = fc.position()[:2]
    best, best_d = None, 1e9
    for name, poly in fc.world.places().items():
        c = np.mean(np.asarray(poly), axis=0)
        dist = float(np.linalg.norm(c - p))
        if min_dist <= dist < best_d:
            best, best_d = name, dist
    return best


def script(rec: Recorder) -> None:
    fc = rec.fc
    rec.advance(1.0)
    rec.say("up", GREEN)
    fc.dispatch("up")
    rec.advance(3.5)
    rec.say("forward", GREEN)
    fc.dispatch("forward")
    rec.advance(4.0)
    rec.stop_and_measure("forward")
    rec.advance(1.0)
    rec.say("left", GREEN)
    fc.dispatch("left")
    rec.advance(3.5)
    rec.stop_and_measure("left")
    rec.advance(1.0)
    place = nearest_place(fc)
    rec.say(f"go to {place}", AMBER, f"go to {place}")
    info = fc.go_to(place)
    print(f"[record] go to {place}: {info['length_m']:.1f} m, eta {info['eta_s']:.1f} s")
    rec.advance(min(info["eta_s"] + 2.0, 24.0))
    rec.say("land", AMBER)
    fc.dispatch("land")
    rec.advance(14.0, until=lambda: fc.phase == "landed")
    rec.advance(2.0)


def write_metrics(rec: Recorder) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "stop_metrics.json").write_text(json.dumps(
        {"note": "scripted run, campus world 1:10 scale, speeds in simulation metres per second",
         "rest_threshold_m_s": REST_SPEED, "stops": rec.metrics}, indent=1), encoding="utf-8")


def write_braking_plot(rec: Recorder) -> Path:
    """Speed against time for the whole scripted flight, with every spoken
    command marked, so the braking after each "stop" is visible on paper."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    IMAGES.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(10, 3.6), dpi=130)
    ax.plot(rec.t_s, rec.speed, color="#0b7285", lw=2)
    ax.axhline(REST_SPEED, color="#868e96", lw=1, ls=":")
    ax.text(rec.t_s[-1], REST_SPEED + 0.02, f"rest threshold {REST_SPEED} m/s", ha="right", va="bottom",
            fontsize=8, color="#495057")
    for t, label, col in rec.events:
        c = "#e03131" if label == "stop" else "#495057"
        ax.axvline(t, color=c, lw=1.1, ls="--" if label == "stop" else "-", alpha=0.8)
        ax.text(t + 0.15, 1.57, label, rotation=90, va="top", fontsize=8, color=c)
    for m, (t, _, _) in zip(rec.metrics, [e for e in rec.events if e[1] == "stop"]):
        ax.annotate(f"{m['braking_distance_m']:.2f} m, {m['time_to_rest_s']:.2f} s", (t + m["time_to_rest_s"], 0.04),
                    (t + 1.1, 0.55), fontsize=8, color="#c92a2a", arrowprops=dict(arrowstyle="->", color="#c92a2a"))
    ax.set_xlabel("time (s)")
    ax.set_ylabel("speed (m/s)")
    ax.set_ylim(0, 1.6)
    ax.set_title("Speed during the scripted campus flight: every 'stop' brings the drone to rest", fontsize=10)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    path = IMAGES / "stop_braking.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def write_outputs(rec: Recorder) -> None:
    import imageio.v2 as imageio

    OUT.mkdir(parents=True, exist_ok=True)
    mp4 = OUT / "campus_demo.mp4"
    with imageio.get_writer(mp4, fps=FPS, codec="libx264", quality=6,
                            macro_block_size=2, pixelformat="yuv420p") as w:
        for fr in rec.frames:
            w.append_data(np.asarray(fr))
    small = []
    for fr in rec.frames[::GIF_EVERY]:
        h = round(fr.height * GIF_WIDTH / fr.width)
        small.append(fr.resize((GIF_WIDTH, h), Image.LANCZOS).quantize(colors=64, method=Image.Quantize.MEDIANCUT))
    gif = OUT / "campus_demo.gif"
    small[0].save(gif, save_all=True, append_images=small[1:], duration=int(1000 / FPS * GIF_EVERY),
                  loop=0, optimize=True, disposal=1)
    write_metrics(rec)
    write_braking_plot(rec)
    print(f"wrote {mp4} ({mp4.stat().st_size / 1e6:.1f} MB), {gif} ({gif.stat().st_size / 1e6:.1f} MB), "
          f"{len(rec.frames)} frames = {len(rec.frames) / FPS:.0f} s")
    for m in rec.metrics:
        print("  ", m)


if __name__ == "__main__":
    plot_only = "--plot-only" in sys.argv
    recorder = Recorder(render=not plot_only)
    script(recorder)
    if plot_only:
        write_metrics(recorder)
        print("wrote", write_braking_plot(recorder))
        for m in recorder.metrics:
            print("  ", m)
    else:
        write_outputs(recorder)
