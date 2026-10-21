"""Tests for natural motion features: velocity blending, acceleration
continuity, banking, and heading control."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
for _p in ("04_control", "05_voice_interface", "01_simulation", "01_simulation/models"):
    sys.path.insert(0, str(ROOT / _p))

from trajectory import QuinticBezierTrajectory, PointToPointTrajectory  # noqa: E402
from position_controller import PositionController, MAX_BANK_DEG, GRAVITY  # noqa: E402


def test_quintic_bezier_zero_boundary_conditions():
    """Zero boundary conditions produce a smooth trajectory from p0 to p1."""
    p0 = np.array([0.0, 0.0, 1.0])
    p1 = np.array([1.5, 0.0, 1.0])
    traj = QuinticBezierTrajectory(p0, p1)

    assert traj.t_total > 0
    assert np.allclose(traj.position(0), p0, atol=1e-6)
    assert np.allclose(traj.position(traj.t_total), p1, atol=1e-6)
    assert np.allclose(traj.velocity(0), np.zeros(3), atol=1e-6)
    assert np.allclose(traj.velocity(traj.t_total), np.zeros(3), atol=1e-6)


def test_quintic_bezier_nonzero_initial_velocity():
    """Non-zero initial velocity is preserved at trajectory start."""
    p0 = np.array([0.0, 0.0, 1.0])
    p1 = np.array([1.5, 0.0, 1.0])
    v0 = np.array([0.5, 0.0, 0.0])
    traj = QuinticBezierTrajectory(p0, p1, v0=v0)

    v_start = traj.velocity(0)
    assert np.linalg.norm(v_start - v0) < 0.1


def test_quintic_bezier_nonzero_initial_acceleration():
    """Non-zero initial acceleration is preserved at trajectory start."""
    p0 = np.array([0.0, 0.0, 1.0])
    p1 = np.array([1.5, 0.0, 1.0])
    a0 = np.array([0.5, 0.0, 0.0])
    traj = QuinticBezierTrajectory(p0, p1, a0=a0)

    a_start = traj.acceleration(0)
    assert np.linalg.norm(a_start - a0) < 0.5


def test_quintic_bezier_velocity_within_limits():
    """Trajectory velocity stays within v_max limit."""
    p0 = np.array([0.0, 0.0, 1.0])
    p1 = np.array([1.5, 0.0, 1.0])
    traj = QuinticBezierTrajectory(p0, p1, v_max=1.0)

    for t in np.linspace(0, traj.t_total, 50):
        v = np.linalg.norm(traj.velocity(t))
        assert v <= 1.0 + 0.1


def test_quintic_bezier_acceleration_within_limits():
    """Trajectory acceleration stays within a_max limit."""
    p0 = np.array([0.0, 0.0, 1.0])
    p1 = np.array([1.5, 0.0, 1.0])
    traj = QuinticBezierTrajectory(p0, p1, a_max=1.0)

    for t in np.linspace(0, traj.t_total, 50):
        a = np.linalg.norm(traj.acceleration(t))
        assert a <= 1.0 + 0.2


def test_quintic_bezier_jerk_bounded():
    """Trajectory jerk is bounded."""
    p0 = np.array([0.0, 0.0, 1.0])
    p1 = np.array([1.5, 0.0, 1.0])
    traj = QuinticBezierTrajectory(p0, p1)

    for t in np.linspace(0, traj.t_total, 50):
        j = np.linalg.norm(traj.jerk(t))
        assert j < 50.0


def test_quintic_bezier_curved_path():
    """Curved path deviates from straight line when curvature > 0."""
    p0 = np.array([0.0, 0.0, 1.0])
    p1 = np.array([1.5, 0.0, 1.0])
    traj = QuinticBezierTrajectory(p0, p1, curvature=0.0)
    t_mid = traj.t_total * 0.5
    p_mid = traj.position(t_mid)
    straight_mid = (p0 + p1) / 2
    deviation = np.linalg.norm(p_mid - straight_mid)
    assert deviation < 0.01


def test_banking_angle_limit():
    """Banking angle is capped at MAX_BANK_DEG."""
    pc = PositionController()
    p = np.array([0.0, 0.0, 1.0])
    v = np.array([0.0, 0.0, 0.0])
    p_desired = np.array([1.5, 0.0, 1.0])
    v_desired = np.array([0.0, 0.0, 0.0])
    a_ff = np.array([10.0, 0.0, 0.0])

    q_desired, thrust = pc.compute(p, v, p_desired, v_desired, a_ff)

    w = np.clip(abs(q_desired[0]), -1.0, 1.0)
    tilt_deg = np.degrees(2 * np.arccos(w))
    assert tilt_deg <= 90.0


def test_heading_forward_facing():
    """Forward-facing heading mode produces yaw rotation."""
    pc = PositionController()
    p = np.array([0.0, 0.0, 1.0])
    v = np.array([1.0, 0.0, 0.0])
    p_desired = np.array([1.5, 0.0, 1.0])
    v_desired = np.array([0.0, 0.0, 0.0])
    a_ff = np.array([0.0, 0.0, 0.0])

    q_desired, _ = pc.compute(p, v, p_desired, v_desired, a_ff,
                               heading_mode="forward_facing")

    assert abs(q_desired[3]) < 0.1


def test_heading_fixed():
    """Fixed heading mode produces minimal yaw rotation (banking may introduce small component)."""
    pc = PositionController()
    p = np.array([0.0, 0.0, 1.0])
    v = np.array([1.0, 0.0, 0.0])
    p_desired = np.array([1.5, 0.0, 1.0])
    v_desired = np.array([0.0, 0.0, 0.0])
    a_ff = np.array([0.0, 0.0, 0.0])

    q_desired, _ = pc.compute(p, v, p_desired, v_desired, a_ff,
                               heading_mode="fixed")

    assert abs(q_desired[3]) < 0.15


def test_backward_compatibility():
    """QuinticBezierTrajectory has same interface as PointToPointTrajectory."""
    p0 = np.array([0.0, 0.0, 1.0])
    p1 = np.array([1.5, 0.0, 1.0])

    traj_old = PointToPointTrajectory(p0, p1)
    traj_new = QuinticBezierTrajectory(p0, p1)

    for t in np.linspace(0, min(traj_old.t_total, traj_new.t_total), 10):
        p_old = traj_old.position(t)
        p_new = traj_new.position(t)
        assert np.linalg.norm(p_old - p_new) < 0.5
