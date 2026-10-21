"""The 1:10 Amrita Coimbatore campus scene (make_campus.py, world.py):
it loads, adds no degrees of freedom or mass to the drone, its convex
building pieces exactly tile each real footprint, and the voice commands
stop at its walls and land on its roofs."""
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
for _p in ("05_voice_interface", "04_control", "01_simulation", "01_simulation/models"):
    sys.path.insert(0, str(ROOT / _p))

from flight import DT, FlightController  # noqa: E402
from world import CAMPUS_JSON, CAMPUS_SCENE, CampusWorld  # noqa: E402

MODELS = ROOT / "01_simulation" / "models"


def _area(poly):
    p = np.asarray(poly)
    x, y = p[:, 0], p[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))


def test_campus_scene_adds_no_dofs_or_mass():
    q = mujoco.MjModel.from_xml_path(str(MODELS / "quad.xml"))
    c = mujoco.MjModel.from_xml_path(str(CAMPUS_SCENE))
    assert (c.nq, c.nv, c.nu, c.nbody) == (q.nq, q.nv, q.nu, q.nbody)
    assert np.allclose(c.body_mass, q.body_mass) and np.allclose(c.body_inertia, q.body_inertia)


def test_convex_pieces_tile_every_footprint():
    data = json.loads(CAMPUS_JSON.read_text(encoding="utf-8"))
    assert len(data["buildings"]) >= 80
    for b in data["buildings"]:
        total = sum(_area(p) for p in b["parts"])
        assert abs(total - _area(b["footprint"])) <= 0.01 * _area(b["footprint"]) + 1e-4, b["name"]
        for part in b["parts"]:                       # every piece is convex (CCW)
            p = np.asarray(part)
            e1 = np.roll(p, -1, axis=0) - p
            e2 = np.roll(e1, -1, axis=0)
            assert np.all(e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0] >= -1e-6), b["name"]


def _fly(fc, command, seconds):
    fc.dispatch(command)
    for _ in range(int(round(seconds / DT))):
        fc.step()


def test_campus_wall_stop_and_roof_landing_at_the_library():
    world = CampusWorld()
    fc = FlightController(scene_path=CAMPUS_SCENE, world=world)
    for _ in range(3):
        _fly(fc, "left", 4.0)                         # the library is to the left of the spawn
    d, name = world.nearest(fc.position())
    assert name == "Central Library" and 0.15 < d < 0.4
    _fly(fc, "up", 4.0)
    _fly(fc, "left", 5.0)
    _fly(fc, "land", 7.0)
    assert fc.phase == "landed" and fc.surface[1] == "Central Library"
    assert abs(fc.position()[2] - (fc.surface[0] + fc.foot_z)) < 0.01
