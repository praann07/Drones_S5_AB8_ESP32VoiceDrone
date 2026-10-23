"""Capture frames of the city scene from the REAL OpenGL window (scratch tool, not part of the project)."""
import math, os, sys, time
CLONE = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))     # the project folder (three levels up from this script)
sys.path.insert(0, CLONE)
import pybullet as p
import drone_core as dc

OUT = os.path.join(os.environ["TEMP"], "city_shots")
os.makedirs(OUT, exist_ok=True)
SCENE = sys.argv[1] if len(sys.argv) > 1 else "city"

assert dc.connect_gui() >= 0
c = dc.DroneController(dc.DEFAULT_OBSTACLES, log=lambda m: None)
t0 = time.time()
world = dc.World(dc.DEFAULT_OBSTACLES, c.home, SCENE)
print("World(%s) built in %.2fs; bodies=%d" % (SCENE, time.time() - t0, p.getNumBodies()))
model = dc.DroneModel(c)
overlays = dc.Overlays(dc.DEFAULT_OBSTACLES, trail=True, zones=False)
DT = 1 / 60
clock = [0.0]


def advance(seconds):
    for i in range(int(seconds / DT)):
        c.step(DT)
        model.update(c, DT)
        world.update(c, model.clock)
        overlays.update(c, model.clock)
        p.stepSimulation()


def shot(name, target, yaw, pitch, dist, fov=60):
    p.resetDebugVisualizerCamera(dist, yaw, pitch, target)
    time.sleep(0.6)
    view = p.computeViewMatrixFromYawPitchRoll(target, dist, yaw, pitch, 0, 2)
    proj = p.computeProjectionMatrixFOV(fov, 1280 / 800, 0.1, 500)
    w, h, rgba, *_ = p.getCameraImage(1280, 800, view, proj, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    data = bytes(bytearray(rgba)) if not hasattr(rgba, "tobytes") else rgba.tobytes()
    rows = [b"".join(data[(y * w + x) * 4:(y * w + x) * 4 + 3] for x in range(w)) for y in range(h)]
    dc._write_png(os.path.join(OUT, "%s_%s.png" % (SCENE, name)), w, h, rows)
    print("saved", name)


# per-frame python cost with everything running
t = time.time()
advance(1.0)
print("python cost per frame: %.2f ms" % ((time.time() - t) / (1.0 / DT) * 1000))

shot("1_overview", [0, 0, 2], 35, -32, 34)
shot("2_helipad_parked", [0, 0, 0.8], 40, -24, 8)
c.apply_intent(dc.Intent.TAKEOFF)
advance(4.0)
shot("3_low_toward_city", [0, 0, 3], 215, -6, 12, 70)
c.apply_intent(dc.Intent.FORWARD)
advance(6.0)
print("label:", c.label, "pos:", [round(v, 2) for v in c.pos])
shot("4_held_at_kiosk", [c.pos[0] + 2.0, 0, 1.8], 30, -18, 11)
advance(3.0)
shot("5_traffic_road", [0, -23, 0.5], 100, -22, 14, 70)
p.disconnect()
