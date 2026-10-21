"""The live flight-status graphs must report what the drone really does."""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
for _p in ("05_voice_interface", "04_control", "01_simulation", "01_simulation/models"):
    sys.path.insert(0, str(ROOT / _p))

from flight import DT, FlightController  # noqa: E402
from status_graphs import (COLUMNS, SAMPLE_DT_S, StatusGraphs, StatusHistory,  # noqa: E402
                           state_label)
from world import CAMPUS_SCENE, CampusWorld  # noqa: E402


@pytest.fixture(scope="module")
def campus():
    return CampusWorld()


def _fly(fc, hist, command, seconds, t0):
    """Step the real controller and record exactly as demo.py does."""
    fc.dispatch(command)
    n = int(round(seconds / DT))
    for i in range(n):
        fc.step()
        hist.record(fc, t0 + (i + 1) * DT)
    return t0 + n * DT


def test_samples_at_most_ten_per_simulated_second_and_window_is_bounded(campus):
    fc = FlightController(scene_path=CAMPUS_SCENE, world=campus)
    hist = StatusHistory(maxlen=20)
    _fly(fc, hist, "hover", 1.0, 0.0)
    assert len(hist.rows) == pytest.approx(1.0 / SAMPLE_DT_S, abs=1)
    t = [r[0] for r in hist.rows]
    assert min(np.diff(t)) >= SAMPLE_DT_S - 1e-6          # never faster than 10 Hz
    _fly(fc, hist, "hover", 3.0, 1.0)
    assert len(hist.window) == 20                          # the chart window stays bounded
    assert len(hist.rows) > 20                             # the CSV keeps the whole session


def test_recorded_values_equal_the_controller_state(campus):
    fc = FlightController(scene_path=CAMPUS_SCENE, world=campus)
    hist = StatusHistory()
    t = _fly(fc, hist, "forward", 2.0, 0.0)
    assert hist.record(fc, t + 1.0)                        # force one more sample, then compare
    row = dict(zip(COLUMNS, hist.rows[-1]))
    p, v = fc.position(), fc.velocity()
    assert (row["x"], row["y"], row["z"]) == pytest.approx(tuple(p))
    assert row["hspeed"] == pytest.approx(np.hypot(v[0], v[1]))
    assert row["vz"] == pytest.approx(v[2])
    assert row["nearest_m"] == pytest.approx(fc.world.nearest(p)[0])
    assert row["surface_z"] == pytest.approx(fc.world.surface_under(p[0], p[1])[0])
    assert (row["phase"], row["command"]) == (fc.phase, fc.command)
    assert row["hspeed"] > 0.2                             # it really was flying forward


def test_events_keep_kind_and_text():
    hist = StatusHistory()
    hist.event(1.5, "command", "forward")
    hist.event(2.0, "heard_miss", "banana")
    hist.event(3.0, "no_route", "central library")
    assert hist.events == [(1.5, "command", "forward"), (2.0, "heard_miss", "banana"),
                           (3.0, "no_route", "central library")]


def test_state_label():
    assert state_label("landed", "hover") == "landed"
    assert state_label("landing", "land") == "landing"
    assert state_label("flying", "stop") == "hover"
    assert state_label("flying", "forward") == "moving"
    assert state_label("flying", "goto") == "go to"


def test_window_builds_and_draws_headless_and_lines_match_history(campus, tmp_path):
    fc = FlightController(scene_path=CAMPUS_SCENE, world=campus)
    g = StatusGraphs(show=False)
    t = 0.0
    for cmd in ("forward", "stop"):
        g.event(t, "command", cmd)
        fc.dispatch(cmd)
        for _ in range(int(round(1.5 / DT))):
            fc.step()
            t += DT
            g.record(fc, t)
    g.event(t, "heard_miss", "banana")
    g.update()
    xs, ys = g.l_alt.get_data()
    assert len(xs) == len(g.hist.window)
    assert np.allclose(ys, g.hist.column("z"))
    assert np.allclose(g.l_hs.get_data()[1], g.hist.column("hspeed"))
    out = tmp_path / "status.png"
    g.save(out)
    assert out.stat().st_size > 10_000                     # something real was drawn


def test_pause_freezes_the_drawn_lines_but_not_the_recording(campus):
    fc = FlightController(scene_path=CAMPUS_SCENE, world=campus)
    g = StatusGraphs(show=False)
    t = _fly(fc, g.hist, "hover", 1.0, 0.0)
    g.update()
    n_drawn = len(g.l_alt.get_data()[0])
    g._toggle_pause()
    assert g.paused
    _fly(fc, g.hist, "hover", 1.0, t)
    g.update()
    assert len(g.l_alt.get_data()[0]) == n_drawn           # frozen on screen
    assert len(g.hist.rows) > n_drawn                      # still recorded
    g._toggle_pause()
    g.update()
    assert len(g.l_alt.get_data()[0]) > n_drawn


def test_csv_has_the_columns_and_every_row(campus, tmp_path):
    fc = FlightController(scene_path=CAMPUS_SCENE, world=campus)
    hist = StatusHistory()
    hist.event(0.0, "command", "hover")
    _fly(fc, hist, "hover", 1.0, 0.0)
    path = hist.save_csv(tmp_path / "run_status.csv")
    rows = list(csv.reader(open(path, newline="", encoding="utf-8")))
    assert tuple(rows[0]) == COLUMNS
    assert len(rows) - 1 == len(hist.rows)
    ev = list(csv.reader(open(tmp_path / "run_status_events.csv", newline="", encoding="utf-8")))
    assert ev[0] == ["t", "kind", "text"] and ev[1][1:] == ["command", "hover"]
