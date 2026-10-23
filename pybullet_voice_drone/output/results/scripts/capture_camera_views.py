"""Real-renderer frames from the chase and top camera framings (scratch tool, not part of the project)."""
import math, os, sys, time
CLONE = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))     # the project folder (three levels up from this script)
sys.path.insert(0, CLONE)
import pybullet as p
import drone_core as dc

OUT = os.path.join(os.environ["TEMP"], "city_shots")
os.makedirs(OUT, exist_ok=True)
assert dc.connect_gui() >= 0
c = dc.DroneController(dc.DEFAULT_OBSTACLES, log=lambda m: None)
world = dc.World(dc.DEFAULT_OBSTACLES, c.home, "city")
model = dc.DroneModel(c)
overlays = dc.Overlays(dc.DEFAULT_OBSTACLES, trail=True)
DT = 1 / 60


def advance(seconds):
    for _ in range(int(seconds / DT)):
        c.step(DT); model.update(c, DT); world.update(c, model.clock); overlays.update(c, model.clock)


def shot(name, target, yaw, pitch, dist, fov=60):
    p.resetDebugVisualizerCamera(dist, yaw, pitch, target)
    time.sleep(0.6)
    view = p.computeViewMatrixFromYawPitchRoll(target, dist, yaw, pitch, 0, 2)
    proj = p.computeProjectionMatrixFOV(fov, 1280 / 800, 0.1, 500)
    w, h, rgba, *_ = p.getCameraImage(1280, 800, view, proj, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    data = bytes(bytearray(rgba)) if not hasattr(rgba, "tobytes") else rgba.tobytes()
    rows = [b"".join(data[(y * w + x) * 4:(y * w + x) * 4 + 3] for x in range(w)) for y in range(h)]
    dc._write_png(os.path.join(OUT, name + ".png"), w, h, rows)
    print("saved", name)


c.apply_intent(dc.Intent.TAKEOFF)
advance(4.0)
c.apply_intent(dc.Intent.LEFT, 4.0)
advance(5.0)
c.apply_intent(dc.Intent.ROTATE_RIGHT, 35.0)
advance(3.0)
c.apply_intent(dc.Intent.FORWARD)
advance(1.5)
# chase framing, exactly as CameraRig computes it
dist, pitch = dc.CameraRig.START["chase"]
shot("6_chase", [c.pos[0], c.pos[1], c.pos[2] + 0.0], math.degrees(c.yaw) - 90.0, pitch, dist)
# top framing
dist, pitch = dc.CameraRig.START["top"]
shot("7_top", list(c.pos), 90.0, pitch, dist)
p.disconnect()
