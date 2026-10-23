"""Time one frame of the REAL main-loop work (city scene, HUD, camera, overlays) in the GUI."""
import math, os, sys, time
CLONE = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))     # the project folder (three levels up from this script)
sys.path.insert(0, CLONE)
import pybullet as p
import drone_core as dc

assert dc.connect_gui() >= 0
c = dc.DroneController(dc.DEFAULT_OBSTACLES, log=lambda m: None)
world = dc.World(dc.DEFAULT_OBSTACLES, c.home, "city")
model = dc.DroneModel(c)
rig = dc.CameraRig("chase")
hud = dc.Hud(model.body, dc.DEFAULT_OBSTACLES)
overlays = dc.Overlays(dc.DEFAULT_OBSTACLES, trail=True)
c.apply_intent(dc.Intent.TAKEOFF)

FRAME = [0]


def frame(dt, now):
    t = {}
    t0 = time.perf_counter(); c.step(dt); t["controller"] = time.perf_counter() - t0
    t0 = time.perf_counter(); model.update(c, dt); t["drone"] = time.perf_counter() - t0
    t0 = time.perf_counter(); rig.update(c.pos, dt, c.yaw); t["camera"] = time.perf_counter() - t0
    t0 = time.perf_counter(); world.update(c, model.clock); t["world+traffic"] = time.perf_counter() - t0
    t0 = time.perf_counter(); overlays.update(c, now); t["overlays"] = time.perf_counter() - t0
    t0 = time.perf_counter(); hud.update(c, "take off", now); t["hud"] = time.perf_counter() - t0
    t0 = time.perf_counter()
    if FRAME[0] % 3 == 0:
        p.stepSimulation()
    FRAME[0] += 1
    t["step"] = time.perf_counter() - t0
    return t

DT = 1 / 120
totals, frames, start = {}, 0, time.perf_counter()
c.apply_intent(dc.Intent.TAKEOFF)
while time.perf_counter() - start < 6.0:
    now = time.perf_counter()
    t = frame(DT, now)
    for k, v in t.items():
        totals[k] = totals.get(k, 0.0) + v
    frames += 1
    if frames == 400:
        c.apply_intent(dc.Intent.FORWARD)
    time.sleep(max(0.0, DT - (time.perf_counter() - now)))
print("frames run: %d in 6 s (target rate %d Hz -> %.0f frames)" % (frames, 1 / DT, 6 / DT))
for k, v in sorted(totals.items(), key=lambda kv: -kv[1]):
    print("  %-14s %.2f ms/frame average" % (k, v / frames * 1000))
print("  TOTAL work     %.2f ms/frame (budget %.1f ms)" % (sum(totals.values()) / frames * 1000, DT * 1000))
print("hud items:", len(hud.items), " label:", c.label)
p.disconnect()
