"""Changing direction in continuous flight must sweep a rounded corner, not an
exact right angle with a stop at the corner."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
for _p in ("05_voice_interface", "04_control", "01_simulation", "01_simulation/models"):
    sys.path.insert(0, str(ROOT / _p))

from flight import DT, FlightController  # noqa: E402
from trajectory import (CruiseTrajectory, CruiseTurnTrajectory,  # noqa: E402
                        MAX_LATERAL_ACCEL_MPS2)
from world import CAMPUS_SCENE, CampusWorld  # noqa: E402

P0 = np.array([0.0, 0.0, 1.0])
LEFT_END = np.array([0.0, 40.0, 1.0])
HEADING_X = np.array([1.0, 0.0, 0.0])


def test_turn_starts_exactly_where_the_drone_is_and_moves():
    t = CruiseTurnTrajectory(P0, LEFT_END, v0=HEADING_X)
    assert t.is_turn
    assert t.position(0.0) == pytest.approx(P0)
    assert t.velocity(0.0) == pytest.approx(HEADING_X)            # no jump in velocity


def test_turn_never_stops_at_the_corner_and_never_jumps():
    t = CruiseTurnTrajectory(P0, LEFT_END, v0=HEADING_X)
    ts = np.arange(0.0, t.T + 2.0, DT)
    v = np.array([t.velocity(x) for x in ts])
    speed = np.linalg.norm(v, axis=1)
    assert speed.min() > 0.85                                     # CruiseTrajectory drops to its along-line speed (0)
    assert np.abs(np.diff(v, axis=0)).max() < 0.01                # smooth: < 0.01 m/s change per 2 ms step
    a = np.array([t.acceleration(x) for x in ts])
    assert np.abs(a[:, 1]).max() <= 1.0 + 1e-9                    # along-line limit kept
    assert np.abs(a[:, 0]).max() <= MAX_LATERAL_ACCEL_MPS2 + 1e-9 # sideways acceleration limit kept


def test_turn_ends_on_the_new_line_with_the_old_heading_bled_off():
    t = CruiseTurnTrajectory(P0, LEFT_END, v0=HEADING_X)
    late = t.T + 1.0
    assert t.velocity(late)[0] == pytest.approx(0.0, abs=1e-9)    # nothing left of the old heading
    shift = np.pi * 1.0 ** 2 / (4 * MAX_LATERAL_ACCEL_MPS2)       # lane shift = pi w^2 / (4 a_lat)
    assert t.position(late)[0] == pytest.approx(shift)
    assert t.position(late)[1] > 1.0                              # and it is already well on its way along +y


def test_no_turn_when_already_moving_along_the_new_line():
    straight = CruiseTurnTrajectory(P0, LEFT_END, v0=np.array([0.0, 0.8, 0.0]))
    assert not straight.is_turn
    ref = CruiseTrajectory(P0, LEFT_END, u0=0.8)
    for x in (0.0, 1.0, 5.0, 30.0):
        assert straight.position(x) == pytest.approx(ref.position(x))


def test_short_leg_does_not_claim_to_settle():
    short = CruiseTurnTrajectory(P0, np.array([0.0, 0.5, 1.0]), v0=HEADING_X)
    assert not short.settles                                      # flight falls back to the straight line


def test_flown_corner_is_rounded_and_the_drone_keeps_moving():
    world = CampusWorld()
    fc = FlightController(scene_path=CAMPUS_SCENE, world=world, continuous=True)
    fc.reset(np.array([0.0, 0.0, 1.0]))
    fc.dispatch("forward")
    for _ in range(int(5.0 / DT)):
        fc.step()
    x_turn = float(fc.position()[0])
    fc.dispatch("left")
    assert isinstance(fc.traj, CruiseTurnTrajectory)              # the open campus lane allows the arc
    speeds, xs = [], []
    for _ in range(int(5.0 / DT)):
        fc.step()
        speeds.append(float(np.linalg.norm(fc.velocity()[:2])))
        xs.append(fc.position()[0])
    assert min(speeds) > 0.8                                      # was 0.32 with the sharp corner
    assert max(xs) - x_turn > 0.5                                 # carried on past the corner while turning (about 1 m)
