"""Cost of Dashboard.record() per call, with the real obstacle-gap function (headless)."""
import os, sys, time, statistics
CLONE = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))     # the project folder (three levels up from this script)
sys.path.insert(0, CLONE)
import drone_core as dc
from dashboard import Dashboard

c = dc.DroneController(obstacles=dc.DEFAULT_OBSTACLES, log=lambda m: None)
c.apply_intent(dc.Intent.TAKEOFF)
for _ in range(600):
    c.step(dc.DT)
zones = dc.DEFAULT_OBSTACLES
d = Dashboard({}, lambda k: min(dc.zone_gap(k.pos, ob) for ob in zones), port=0)
t0 = d.start_time
stored, skipped = [], []
for i in range(20000):                      # 20000 frames at ~110 Hz
    now = t0 + i / 110.0
    a = time.perf_counter()
    ok = d.record(c, now=now)
    b = time.perf_counter()
    (stored if ok else skipped).append((b - a) * 1000)
print("frames that store a sample: n=%d mean %.4f ms  max %.4f ms" % (len(stored), statistics.mean(stored), max(stored)))
print("frames that skip:           n=%d mean %.5f ms  max %.4f ms" % (len(skipped), statistics.mean(skipped), max(skipped)))
per_second = (sum(stored) + sum(skipped)) / (20000 / 110.0)
print("total per second of flight: %.3f ms (of 1000 ms)" % per_second)
