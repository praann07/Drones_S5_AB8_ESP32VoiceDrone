"""Closed-loop robustness additions: yaw-priority rotor desaturation,
outer-loop integral action, and the opt-in disturbance hooks on
FlightController. Each test pins a failure that run_robustness.py
measured on the original gated cascade."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
for _p in ("04_control", "05_voice_interface", "01_simulation", "01_simulation/models",
           "02_identification", "03_validation"):
    sys.path.insert(0, str(ROOT / _p))

from mixer import U_MAX, inverse_mix, inverse_mix_yaw_priority, mixer_matrix  # noqa: E402
from flight import DT, FlightController  # noqa: E402
from trajectory import POSITION_TOLERANCE_M  # noqa: E402


def _fly_forward(seconds=8.0, **kw):
    fc = FlightController(**kw)
    target = fc.dispatch("forward")["target"]
    for _ in range(int(round(seconds / DT))):
        fc.step()
    return float(np.linalg.norm(fc.position() - target))


@pytest.mark.parametrize("T,tau", [
    (9.81, np.array([0.0, 0.0, 0.0])),
    (9.81, np.array([0.05, -0.03, 0.01])),
])
def test_yaw_priority_identical_when_not_saturating(T, tau):
    u_plain, sat = inverse_mix(T, tau)
    u_prio, intervened = inverse_mix_yaw_priority(T, tau)
    assert not sat and not intervened
    assert np.allclose(u_plain, u_prio, atol=1e-12)


def test_yaw_priority_preserves_thrust_and_rollpitch_under_yaw_overload():
    T, tau = 9.81, np.array([0.02, -0.02, 0.5])    # yaw demand far past headroom
    u_clip, _ = inverse_mix(T, tau)
    u_prio, intervened = inverse_mix_yaw_priority(T, tau)
    assert intervened
    assert np.all(u_prio >= -1e-9) and np.all(u_prio <= U_MAX + 1e-9)
    T_back, tx, ty, tz = mixer_matrix() @ u_prio
    assert T_back == pytest.approx(T, rel=1e-6)          # collective thrust kept
    assert (tx, ty) == pytest.approx((0.02, -0.02), abs=1e-6)  # roll/pitch kept
    assert abs(tz) < abs(tau[2])                          # yaw was the sacrifice
    assert abs(u_clip.sum() - T) > 1e-3                   # plain clipping lost thrust


def test_noise_and_wind_hooks_are_off_by_default():
    fc = FlightController()
    p, v, q, w = fc.measured_state()
    assert np.array_equal(p, fc.position()) and np.array_equal(w, fc.angular_velocity())
    assert not fc._noise and not fc.wind_force.any() and fc.wind_fn is None


def test_robust_false_reproduces_original_gated_cascade():
    fc = FlightController(robust=False)
    assert fc.outer.ki == 0.0 and fc.inner.yaw_priority is False


def test_gyro_noise_is_tolerated_with_corrected_gains():
    # 0.05 rad/s once broke the original cascade (0.36 m miss); the cause was the
    # missing 1/b factor in the inner-loop gains. With it fixed, both cascades
    # tolerate 10x that noise.
    noise = {"gyro_std": 0.5}
    assert _fly_forward(sensor_noise=noise, seed=0, robust=False) < POSITION_TOLERANCE_M
    assert _fly_forward(sensor_noise=noise, seed=0, robust=True) < POSITION_TOLERANCE_M


def test_integral_action_removes_payload_steady_state_offset():
    def err(robust):
        fc = FlightController(robust=robust)
        fc.set_payload_scale(1.2)
        target = fc.dispatch("forward")["target"]
        for _ in range(int(round(8.0 / DT))):
            fc.step()
        return float(np.linalg.norm(fc.position() - target))

    assert err(False) > 0.3                      # PD: ~0.2*g/Kp = 0.49 m sag
    assert err(True) < POSITION_TOLERANCE_M


def test_integral_does_not_wind_up_far_from_target():
    from position_controller import PositionController
    pc = PositionController(ki=2.0)
    far = np.array([5.0, 0.0, 1.0])
    for _ in range(2000):
        pc.compute(np.array([0.0, 0.0, 1.0]), np.zeros(3), far, np.zeros(3), np.zeros(3))
    assert np.allclose(pc._int, 0.0)             # error > active radius -> frozen
