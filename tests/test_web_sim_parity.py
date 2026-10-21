"""The browser simulator (web/sim_core.js) must fly like the real cascade
(05_voice_interface/flight.py on MuJoCo). Runs the JS core under Node and
compares positions on the same command sequence. Skipped if Node is not
installed or web/params.js has not been exported."""
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
for _p in ("04_control", "05_voice_interface", "01_simulation", "01_simulation/models",
           "02_identification", "03_validation"):
    sys.path.insert(0, str(ROOT / _p))

from commands import classify  # noqa: E402
from flight import DT, FlightController  # noqa: E402

NODE = shutil.which("node")
pytestmark = pytest.mark.skipif(
    NODE is None or not (ROOT / "web" / "params.js").exists(),
    reason="needs Node.js and web/params.js (run web/export_web_sim.py)")

EVERY = 50  # compare every 0.1 s


def run_js(spec: dict) -> dict:
    out = subprocess.run([NODE, str(ROOT / "web" / "parity_cli.mjs"), json.dumps(spec)],
                         capture_output=True, text=True, check=True, timeout=300)
    return json.loads(out.stdout)


def run_py(legs) -> np.ndarray:
    fc = FlightController()
    ps, k = [], 0
    for cmd, secs in legs:
        fc.dispatch(cmd, measure_traj_latency=False)
        for _ in range(int(round(secs / DT))):
            fc.step()
            k += 1
            if k % EVERY == 0:
                ps.append(fc.position())
    return np.array(ps)


@pytest.mark.parametrize("legs", [
    [["forward", 6.0]],
    [["forward", 2.0], ["left", 2.5], ["up", 3.0], ["back", 4.0]],   # chained, mid-flight
])
def test_js_cascade_tracks_mujoco_cascade(legs):
    js = np.array(run_js({"legs": legs, "sample_every": EVERY})["p"])
    py = run_py(legs)
    assert js.shape == py.shape
    # measured 0.02-0.05 mm on 2026-10-20; 1 mm leaves room for float/OS noise
    assert np.max(np.linalg.norm(js - py, axis=1)) < 0.001


def test_js_classifier_matches_python():
    phrases = ["go straight", "five meters to left", "foward", "lef", "turn right please",
               "go up", "descend", "hover", "stay there", "banana", "laptop", "their",
               "finally does to left", "back", "pause"]
    js = run_js({"phrases": phrases})["classify"]
    assert js == [classify(p) for p in phrases]
