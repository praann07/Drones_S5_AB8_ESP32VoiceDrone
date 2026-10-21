"""'go to <place>': name matching, route planning around buildings, the
curved path trajectory, and continuous (fly-until-stop) moves."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
for p in ("05_voice_interface", "04_control", "01_simulation", "01_simulation/models"):
    sys.path.insert(0, str(ROOT / p))

from world import CampusWorld, CityWorld, CAMPUS_SCENE  # noqa: E402
from places import PlaceBook  # noqa: E402
from route_planner import RoutePlanner, CLEARANCE_M  # noqa: E402
from trajectory import PathTrajectory, CruiseTrajectory, MAX_ACCEL_MPS2  # noqa: E402
from flight import FlightController, DT  # noqa: E402

AB1 = "Academic Block 1 - School of Engineering"


@pytest.fixture(scope="module")
def campus():
    return CampusWorld()


@pytest.mark.parametrize("said,place", [
    ("go to AB1", AB1), ("go to a b one", AB1), ("go to ab three",
                                                  "Academic Block 3 - International school of Law"),
    ("fly to the central kitchen", "Central Kitchen"), ("take me to central kitchens", "Central Kitchen"),
    ("go to the library", "Central Library"), ("library", "Central Library"),
    ("go to m b a canteen", "MBA Canteen"), ("go to the canteen", "Main Canteen"),
])
def test_place_names_are_recognised(campus, said, place):
    assert PlaceBook(campus).match(said) == place


@pytest.mark.parametrize("said", ["go left", "go straight", "stop", "land", "and", "go to the left",
                                  "take off", "up"])
def test_plain_commands_are_not_places(campus, said):
    assert PlaceBook(campus).match(said) is None


def test_city_places():
    pb = PlaceBook(CityWorld())
    assert pb.match("go to the tower") == "obstacle_tower"
    assert pb.match("go home") == "home_pad"


def test_route_goes_around_buildings_and_ends_beside_target(campus):
    fc = FlightController(scene_path=CAMPUS_SCENE, world=campus)
    rp = RoutePlanner(campus, fc.fence)
    poly = campus.places()["Central Kitchen"]
    wp = rp.plan_to_polygon(np.zeros(2), poly, 1.0)
    assert wp is not None and np.allclose(wp[0], 0.0)
    clear = rp.clearance(1.0)
    traj = PathTrajectory(np.column_stack([wp, np.ones(len(wp))]))
    for p in traj.samples():
        assert clear[rp.cell(*p[:2])] >= 0.8 * CLEARANCE_M
    # ends next to the kitchen: inside 1.5 m of its wall, not inside it
    from world import _Prism
    d = _Prism(poly, 1.0, "k").distance_2d(*traj.p1[:2])
    assert 0.3 < d < 1.5
    # longer than the straight line: it bends around what is in the way
    assert traj.distance > np.linalg.norm(wp[-1]) * 1.02


def test_path_trajectory_respects_limits():
    W = np.array([[0, 0, 1], [3, 0, 1], [3, 3, 1], [6, 3, 1]], dtype=float)
    tr = PathTrajectory(W, v_max=2.0)
    ts = np.linspace(0, tr.t_total, 600)
    V = np.array([tr.velocity(t) for t in ts])
    assert np.linalg.norm(V, axis=1).max() <= 2.0 + 1e-6
    assert np.allclose(tr.position(0), W[0]) and np.allclose(tr.position(tr.t_total), W[-1])
    assert np.allclose(tr.velocity(tr.t_total), 0)
    assert tr.speed.min() < 1.5          # slows down in the bends


def test_cruise_trajectory_brakes_at_a_max():
    tr = CruiseTrajectory(np.zeros(3), np.array([0.5, 0, 0]), u0=1.0)
    assert tr.t_total == pytest.approx(1.0 / MAX_ACCEL_MPS2)       # straight braking, no reversal
    xs = [tr.position(t)[0] for t in np.linspace(0, tr.t_total, 50)]
    assert np.all(np.diff(xs) >= -1e-12)


def test_continuous_forward_flies_until_the_tower_and_stop_brakes_ahead():
    fc = FlightController(continuous=True)
    fc.dispatch("forward")
    for _ in range(int(8 / DT)):
        fc.step()
    d, name = fc.world.nearest(fc.position())
    assert name == "obstacle_tower" and 0.2 < d < 0.4
    fc = FlightController(continuous=True)
    fc.dispatch("left")
    for _ in range(int(3 / DT)):
        fc.step()
    fc.dispatch("stop")
    ys = []
    for _ in range(int(4 / DT)):
        fc.step()
        ys.append(fc.position()[1])
    assert max(ys) - ys[-1] < 0.02       # comes to rest ahead, never turns back


def test_default_controller_keeps_fixed_steps():
    fc = FlightController()
    r = fc.dispatch("left")
    assert np.allclose(r["target"][:2], [0.0, 1.5], atol=1e-6)
