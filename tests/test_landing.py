"""Landing on real surfaces: the floor and city geoms are solid (they
collide with the drone's four feet and hull), "down" stops above whatever
is below, and "land" sets the drone down on the ground, the home pad or a
roof and switches the motors off."""
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
for _p in ("05_voice_interface", "04_control", "01_simulation", "01_simulation/models"):
    sys.path.insert(0, str(ROOT / _p))

from city import OBSTACLE_FACE_X, obstacle, surface_under  # noqa: E402
from commands import classify  # noqa: E402
from flight import DT, MIN_CLEARANCE_M, FlightController  # noqa: E402


def _fly(fc, command, seconds):
    fc.dispatch(command)
    z_low = np.inf
    for _ in range(int(round(seconds / DT))):
        fc.step()
        z_low = min(z_low, fc.position()[2])
    return z_low


def test_drone_rests_on_its_feet_on_the_floor_and_on_a_roof():
    m = mujoco.MjModel.from_xml_path(str(ROOT / "01_simulation" / "models" / "scene.xml"))
    d = mujoco.MjData(m)
    tower = obstacle()
    for start, surface in (([1.5, 1.5, 1.0], 0.0), ([tower.x, tower.y, 4.2], tower.height)):
        mujoco.mj_resetData(m, d)
        d.qpos[:3] = start
        for _ in range(2500):          # motors off: free fall, then rest
            mujoco.mj_step(m, d)
        assert abs(d.qpos[2] - (surface + 0.07)) < 2e-3
        assert np.linalg.norm(d.qvel) < 1e-6


def test_quad_alone_has_nothing_to_collide_with():
    # identification data comes from quad.xml: the new feet must never touch anything there
    m = mujoco.MjModel.from_xml_path(str(ROOT / "01_simulation" / "models" / "quad.xml"))
    d = mujoco.MjData(m)
    for _ in range(1500):
        mujoco.mj_step(m, d)
        assert d.ncon == 0


def test_repeated_down_stops_above_the_ground():
    fc = FlightController()
    floor = surface_under(0.0, 0.0)[0] + fc.foot_z + MIN_CLEARANCE_M
    for _ in range(5):
        z_low = _fly(fc, "down", 4.0)
        assert z_low > floor - 0.03
    assert fc.phase == "flying"


def test_land_on_the_pad_switches_motors_off_and_stays():
    fc = FlightController()
    _fly(fc, "land", 8.0)
    h, name = surface_under(0.0, 0.0)
    assert fc.phase == "landed" and name == "home_pad"
    assert abs(fc.position()[2] - (h + fc.foot_z)) < 0.01
    assert np.all(fc.data.ctrl == 0.0)
    for cmd in ("land", "down", "stop", "hover"):   # all mean "stay landed"
        _fly(fc, cmd, 1.0)
        assert fc.phase == "landed"
    assert abs(fc.position()[2] - (h + fc.foot_z)) < 0.01


def test_take_off_after_landing():
    fc = FlightController()
    _fly(fc, "land", 8.0)
    _fly(fc, classify("take off"), 5.0)
    assert fc.phase == "flying" and fc.position()[2] > 0.5


def test_obstacle_stops_the_drone_then_land_on_its_roof():
    fc = FlightController()
    for _ in range(3):
        _fly(fc, "forward", 5.0)
    assert fc.position()[0] < OBSTACLE_FACE_X           # stopped short of the wall
    tower = obstacle()
    for _ in range(6):
        _fly(fc, "up", 5.0)
    assert fc.position()[2] > tower.height
    _fly(fc, "forward", 5.0)
    _fly(fc, "land", 8.0)
    assert fc.phase == "landed" and fc.surface[1] == tower.name
    assert abs(fc.position()[2] - (tower.height + fc.foot_z)) < 0.01


def test_land_is_never_triggered_by_a_similar_word():
    assert classify("and") is None
    assert classify("a little deployed and are they look their relatives") is None
    assert classify("please land") == "land"
    assert classify("touch down") == "land"
