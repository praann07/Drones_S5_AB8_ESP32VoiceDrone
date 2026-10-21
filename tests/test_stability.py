"""Closed-loop stability of the inner attitude loop, from the identified
model (04_control/run_stability.py). Pins the 2026-10-20 gain-units fix:
before it the gains lacked the 1/b factor, poles sat at -4.8/-874 1/s,
the loop needed >= ~440 Hz control and went unstable with one sample of
computation delay at the 500 Hz in use."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
for _p in ("04_control", "01_simulation"):
    sys.path.insert(0, str(ROOT / _p))

import run_stability as rs  # noqa: E402
from controller import (DAMPING_RATIO, SETTLING_TIME_S,  # noqa: E402
                        gains_from_identified_model, identified_torque_effectiveness)


@pytest.fixture(scope="module")
def plant():
    p = rs.identified_plant()
    Kq, Ko = gains_from_identified_model(p["A_sindy"], p["B_u"])
    return p, Kq, Ko


def test_identified_effectiveness_matches_true_inverse_inertia(plant):
    """b is identified, not looked up - but it must be the physical 1/I."""
    p, _, _ = plant
    b = identified_torque_effectiveness(p["B_u"])
    np.testing.assert_allclose(b, 1.0 / rs.TRUE_INERTIA, rtol=0.05)


@pytest.mark.parametrize("axis", [0, 1, 2])
def test_continuous_poles_match_design_intent(plant, axis):
    p, Kq, Ko = plant
    A, B = rs.axis_ol(p["a"][axis], p["b"][axis])
    poles = np.sort_complex(np.linalg.eigvals(A - B @ rs.axis_gain(Kq[axis], Ko[axis])))
    wn = 4.0 / (DAMPING_RATIO * SETTLING_TIME_S)
    want = np.sort_complex(np.roots([1.0, 2 * DAMPING_RATIO * wn, wn ** 2]))
    np.testing.assert_allclose(poles, want, rtol=1e-6)


@pytest.mark.parametrize("axis", [0, 1, 2])
def test_robust_to_sampling_and_one_sample_delay(plant, axis):
    """At 500 Hz: stable with one sample of delay, and a gain margin far
    above the 1.14x the unfixed gains had."""
    p, Kq, Ko = plant
    A, B = rs.axis_ol(p["a"][axis], p["b"][axis])
    K = rs.axis_gain(Kq[axis], Ko[axis])
    T = 1.0 / rs.CONTROL_RATE_HZ
    assert rs.sampled_rho(A, B, K, T, delay=True) < 1.0
    assert rs.gain_margin(A, B, K, T, delay=True) > 10.0
    assert rs.min_stable_rate(A, B, K, delay=True) < 100.0


def test_cascade_stable_with_delay(plant):
    p, Kq, Ko = plant
    A, B = rs.cascade_ol(p["a"][1], p["b"][1])
    K = rs.cascade_gain(Kq[1], Ko[1])
    assert np.all(np.linalg.eigvals(A - B @ K).real < 0)
    assert rs.sampled_rho(A, B, K, 1.0 / rs.CONTROL_RATE_HZ, delay=True) < 1.0


def test_sampled_analysis_predicts_mujoco(plant):
    """The analysis is only worth trusting if MuJoCo agrees: the MuJoCo
    roll recovery must follow the identified linear closed loop."""
    p, Kq, Ko = plant
    t, e, _ = rs.mujoco_roll_response(Kq, Ko, hold_steps=1, physics_dt=0.002)
    pred = rs.linear_prediction(p["a"][0], p["b"][0], Kq[0], Ko[0], t)
    assert np.max(np.abs(e - pred)) < 0.05 * 0.05   # < 5% of the 0.05 rad start


def test_legacy_gains_reproduce_the_finding(plant):
    """The pre-fix gains: barely stable at 500 Hz, unstable with one
    sample of delay. Pins the reason the fix exists."""
    p, _, _ = plant
    Kq, Ko = rs.legacy_gains(p["A_sindy"])
    A, B = rs.axis_ol(p["a"][0], p["b"][0])
    K = rs.axis_gain(Kq[0], Ko[0])
    assert 400 < rs.min_stable_rate(A, B, K, delay=False) < 500
    assert rs.sampled_rho(A, B, K, 1.0 / rs.CONTROL_RATE_HZ, delay=True) > 1.0
