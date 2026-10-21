"""A drone must not slide along a wall close enough to touch it.

The campus clamp used to stop only a flight that would ENTER a building. On an
angled wall the straight path grazed it at a few centimetres, the 0.34 m wide
hull touched, tipped onto its side and pinned itself against the building."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
for _p in ("05_voice_interface", "04_control", "01_simulation", "01_simulation/models"):
    sys.path.insert(0, str(ROOT / _p))

from flight import DT, FlightController  # noqa: E402
from world import CAMPUS_SCENE, CampusWorld  # noqa: E402

# the spot in the campus alley where the demo drone got wedged: it sits at the
# margin of one wall and a "back" flight would run along an angled second wall
ALLEY = np.array([21.54, -15.12, 0.5])
IDENTITY_Q = np.array([1.0, 0.0, 0.0, 0.0])


@pytest.fixture(scope="module")
def campus():
    return CampusWorld()


def test_clamp_stops_a_flight_that_grazes_a_wall(campus):
    target = ALLEY + np.array([-80.0, 0.0, 0.0])
    swept = campus.clamp(ALLEY, target)
    assert np.linalg.norm(swept - ALLEY) < 0.1                       # may not slide along the wall
    unswept = campus.clamp(ALLEY, target, sweep=False)               # the old entry-only check
    assert np.linalg.norm(unswept - ALLEY) > 0.5


def test_clamp_still_lets_the_drone_move_away_from_a_wall(campus):
    target = ALLEY + np.array([0.0, -1.0, 0.0])                      # away from the wall
    assert np.allclose(campus.clamp(ALLEY, target), target)


def test_clamp_ignores_walls_the_drone_flies_above(campus):
    high = np.array([ALLEY[0], ALLEY[1], 6.0])
    target = high + np.array([-20.0, 0.0, 0.0])
    assert campus.clamp(high, target) == pytest.approx(target)


def test_flying_back_along_the_wall_never_touches_it(campus):
    fc = FlightController(scene_path=CAMPUS_SCENE, world=campus, continuous=True)
    fc.reset(ALLEY)
    fc.step()
    fc.dispatch("back")
    closest, worst_tilt = np.inf, 0.0
    for _ in range(int(6.0 / DT)):
        fc.step()
        closest = min(closest, campus.nearest(fc.position())[0])
        worst_tilt = max(worst_tilt, fc.attitude_error_deg(IDENTITY_Q))
    assert closest > 0.22                                            # hull half-width is 0.17 m
    assert worst_tilt < 30.0                                         # never knocked over
    assert fc.data.ncon == 0                                         # no contact with any building
