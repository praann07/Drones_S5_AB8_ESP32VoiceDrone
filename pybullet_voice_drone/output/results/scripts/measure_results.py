"""Measure how the flight model behaves and write the results table.

Runs the real controller headless (no window) at 60 steps per second, with fixed random seeds, so the
numbers are repeatable. Writes next to the `results` folder:

    results_table.md    the tables, ready to read or paste
    results_table.csv   the same numbers, one measurement per row
    results.json        everything, for further processing

Run it with the project's Python (it needs pybullet installed):

    python measure_results.py

PROJECT must point at the folder that contains drone_core.py.
"""

import csv
import json
import math
import os
import platform
import random
import re
import sys
import time
from datetime import datetime

PROJECT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))     # the project folder (three levels up from this script)
RESULTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))     # .../output/results
sys.path.insert(0, PROJECT)

import pybullet                                   # noqa: E402  (only used to report its version)
import drone_core as dc                           # noqa: E402
from drone_core import DroneController, FlightMode, Intent   # noqa: E402

DT = 1 / 60
OBSTACLES = dc.DEFAULT_OBSTACLES
quiet = lambda message: None


def run_until(c, done, limit=120.0):
    t = 0.0
    while not done(c) and t < limit:
        c.step(DT)
        t += DT
    return t


def airborne(obstacles=(), strategy="stop", start=(0.0, 0.0)):
    c = DroneController(obstacles, start=start, log=quiet, on_obstacle=strategy)
    c.apply_intent(Intent.TAKEOFF)
    run_until(c, lambda c: c.mode is FlightMode.FLYING)
    return c


def closest_zone_gap(c):
    return min(dc.zone_gap(c.pos, ob) for ob in OBSTACLES)


rows = []          # (behaviour, command, metric, value, unit)


def add(behaviour, command, metric, value, unit):
    rows.append((behaviour, command, metric, round(value, 3), unit))


# 1. take-off -----------------------------------------------------------------------------------
c = DroneController((), log=quiet)
c.apply_intent(Intent.TAKEOFF)
t = run_until(c, lambda c: c.mode is FlightMode.FLYING)
add("Take-off", "take off", "time to reach 2 m", t, "s")
add("Take-off", "take off", "final altitude error", abs(c.pos[2] - dc.TAKEOFF_HEIGHT), "m")

# 2. measured move ------------------------------------------------------------------------------
c = airborne()
c.apply_intent(Intent.FORWARD, 3.0)
t = run_until(c, lambda c: c.maneuver is None)
for _ in range(120):
    c.step(DT)                                                  # let it settle for two seconds
add("Measured move", "forward 3 meters", "time to complete", t, "s")
add("Measured move", "forward 3 meters", "along-track error", abs(c.pos[0] - 3.0), "m")
add("Measured move", "forward 3 meters", "sideways error", abs(c.pos[1]), "m")

# 3. measured turn ------------------------------------------------------------------------------
c = airborne()
c.apply_intent(Intent.ROTATE_LEFT, 90.0)
t = run_until(c, lambda c: c.maneuver is None)
for _ in range(120):
    c.step(DT)
add("Measured turn", "rotate left 90 degrees", "time to complete", t, "s")
add("Measured turn", "rotate left 90 degrees", "heading error", abs(math.degrees(c.yaw) - 90.0), "deg")

# 4. stop before an obstacle (the default behaviour) -------------------------------------------------
c = airborne(OBSTACLES, "stop")
c.apply_intent(Intent.FORWARD)
previous, peak_deceleration, closest, t = 0.0, 0.0, 1e9, 0.0
while t < 15.0:
    c.step(DT)
    t += DT
    peak_deceleration = max(peak_deceleration, (previous - c.vel[0]) / DT)
    previous = c.vel[0]
    closest = min(closest, closest_zone_gap(c))
add("Stop before an obstacle", "forward (towards the first tower)", "stopping position x", c.pos[0], "m")
add("Stop before an obstacle", "forward (towards the first tower)", "distance held from the safety zone",
    closest_zone_gap(c), "m")
add("Stop before an obstacle", "forward (towards the first tower)", "closest approach to the zone", closest, "m")
add("Stop before an obstacle", "forward (towards the first tower)", "peak deceleration", peak_deceleration, "m/s^2")
add("Stop before an obstacle", "forward (towards the first tower)", "hard safety-net interventions",
    c.collisions_prevented, "count")

# 5. steering around obstacles (optional behaviour) ---------------------------------------------------
c = airborne(OBSTACLES, "steer")
c.apply_intent(Intent.FORWARD)
t, closest = 0.0, 1e9
while t < 40.0 and c.pos[0] < dc.WORLD_LIMIT - 1e-6:
    c.step(DT)
    t += DT
    closest = min(closest, closest_zone_gap(c))
add("Steer around obstacles", "forward through all three towers (--on-obstacle steer)", "time to the far wall", t, "s")
add("Steer around obstacles", "forward through all three towers (--on-obstacle steer)",
    "closest approach to any zone", closest, "m")
add("Steer around obstacles", "forward through all three towers (--on-obstacle steer)",
    "hard safety-net interventions", c.collisions_prevented, "count")

# 6. return home, open space ---------------------------------------------------------------------
c = airborne(())
c.pos = [8.0, -5.0, 2.0]
c.apply_intent(Intent.HOME)
t = run_until(c, lambda c: c.mode is FlightMode.LANDED)
add("Return home (open space)", "go home", "starting distance", math.hypot(8.0, 5.0), "m")
add("Return home (open space)", "go home", "time to land", t, "s")
add("Return home (open space)", "go home", "landing error from the pad centre", math.hypot(*c.pos[:2]), "m")

# 7. return home through the towers --------------------------------------------------------------
c = airborne(OBSTACLES, "steer")
c.apply_intent(Intent.FORWARD)
run_until(c, lambda c: c.pos[0] >= dc.WORLD_LIMIT - 1e-6, 60)
start = math.hypot(*c.pos[:2])
c.apply_intent(Intent.HOME)
t = run_until(c, lambda c: c.mode is FlightMode.LANDED, 90)
add("Return home (through the towers)", "go home (--on-obstacle steer)", "starting distance", start, "m")
add("Return home (through the towers)", "go home (--on-obstacle steer)", "time to land", t, "s")
add("Return home (through the towers)", "go home (--on-obstacle steer)", "landing error from the pad centre",
    math.hypot(*c.pos[:2]), "m")

# 8. randomised safety check (default stopping behaviour) -----------------------------------------------
random.seed(11)
inside_zone = lambda c: any(dc.inside_box(c.pos, *dc.inflated_box(o, dc.SAFETY_DISTANCE)) for o in OBSTACLES)
runs = skipped = entered = hard = held = 0
for _ in range(220):
    start = (random.uniform(-10, 10), random.uniform(-10, 10))
    c = DroneController(OBSTACLES, start=start, log=quiet)
    c.yaw = random.uniform(-math.pi, math.pi)
    c.apply_intent(Intent.TAKEOFF)
    run_until(c, lambda c: c.mode is FlightMode.FLYING)
    if inside_zone(c):
        skipped += 1                                            # started inside a zone: not a fair test
        continue
    c.apply_intent(random.choice([Intent.FORWARD, Intent.BACKWARD, Intent.LEFT, Intent.RIGHT]))
    runs += 1
    for _ in range(int(20 / DT)):
        c.step(DT)
        if inside_zone(c):
            entered += 1
            break
    hard += c.collisions_prevented > 0
    held += c.label == "OBSTACLE STOP"
add("Randomised safety check", "random start, heading and direction, 20 s each", "runs", runs, "count")
add("Randomised safety check", "random start, heading and direction, 20 s each",
    "starts discarded (inside a zone)", skipped, "count")
add("Randomised safety check", "random start, heading and direction, 20 s each",
    "runs that entered a safety zone", entered, "count")
add("Randomised safety check", "random start, heading and direction, 20 s each",
    "runs that needed the hard safety net", hard, "count")
add("Randomised safety check", "random start, heading and direction, 20 s each",
    "runs that ended held at an obstacle", held, "count")


# loop performance and GUI call costs come from the live-window scripts, if they have been run
def read(name):
    path = os.path.join(RESULTS, name)
    return open(path, encoding="utf-8").read() if os.path.exists(path) else ""


performance = read("loop_performance.txt")
call_costs = read("gui_call_costs.txt")
frames = re.search(r"frames run: (\d+) in 6 s", performance)
work = re.search(r"TOTAL work\s+([\d.]+) ms/frame \(budget ([\d.]+) ms\)", performance)

# ------------------------------------------------------------------------------------------ outputs
stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
def pybullet_version():
    from importlib import metadata
    for name in ("pybullet", "pybullet_arm64"):           # the ARM64 build installs under its own name
        try:
            return metadata.version(name)
        except metadata.PackageNotFoundError:
            continue
    return "unknown version"


environment = "Python %s, pybullet %s, %s" % (platform.python_version(), pybullet_version(), platform.platform())

with open(os.path.join(RESULTS, "results_table.csv"), "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(["Behaviour", "Command", "Metric", "Value", "Unit"])
    writer.writerows(rows)

with open(os.path.join(RESULTS, "results.json"), "w", encoding="utf-8") as f:
    json.dump({"generated": stamp, "environment": environment, "time_step_s": DT,
               "measurements": [dict(zip(("behaviour", "command", "metric", "value", "unit"), r)) for r in rows],
               "loop_performance": ({"frames_in_6_s": int(frames.group(1)),
                                     "work_ms_per_frame": float(work.group(1)),
                                     "budget_ms": float(work.group(2))} if frames and work else None)},
              f, indent=2)


def section(title):
    lines = ["| Command | Metric | Value |", "|---|---|---:|"]
    for behaviour, command, metric, value, unit in rows:
        if behaviour == title:
            lines.append("| %s | %s | %s %s |" % (command, metric, ("%g" % value), unit))
    return "### %s\n\n%s\n" % (title, "\n".join(lines))


titles = []
for behaviour, *_ in rows:
    if behaviour not in titles:
        titles.append(behaviour)

markdown = [
    "# Measured results\n",
    "Generated %s by `scripts/measure_results.py` (%s).  \n"
    "The controller was run headless at 60 steps per second with fixed random seeds, so the numbers repeat.\n" % (
        stamp, environment),
    "## Flight-model behaviour\n",
] + [section(t) for t in titles if t != "Randomised safety check"] + [
    "## Randomised safety check\n",
    "Random start position in the plaza, random heading, and a random one of forward / backward / left / right, each "
    "flown for 20 s with the default stopping behaviour. Starts that began inside a safety zone were discarded.\n",
    section("Randomised safety check").split("\n", 2)[2],
]
if frames and work:
    markdown += [
        "## Main-loop performance (live window)\n",
        "Measured by `scripts/measure_loop_performance.py` with the full city, traffic, labels, chase camera and "
        "overlays running (raw output in `loop_performance.txt`).\n",
        "| Frames in 6 s | Loop rate | Work per frame |\n|---:|---:|---:|\n| %s | about %.0f Hz | %s ms of a %s ms budget |\n"
        % (frames.group(1), int(frames.group(1)) / 6.0, work.group(1), work.group(2)),
        "This measures the Python loop, not the graphics card's own frame rate.\n",
    ]
if call_costs:
    markdown += ["## Cost of each PyBullet call (live window)\n",
                 "Raw output of `scripts/benchmark_gui_calls.py`. The window-touching calls (text, lines, colours) "
                 "cost far more than moving a body, and how much more varies from run to run with how busy the "
                 "window is; moving bodies stays under 0.1 ms in every run.\n",
                 "```text\n%s\n```\n" % call_costs.strip().replace("\r", "")]

with open(os.path.join(RESULTS, "results_table.md"), "w", encoding="utf-8") as f:
    f.write("\n".join(markdown))

for row in rows:
    print(" | ".join(str(v) for v in row))
print("\nwrote results_table.md, results_table.csv, results.json to", RESULTS)
