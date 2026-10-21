"""Autonomous navigation among moving obstacles: obstacle model geometry,
scenario fairness, scene physics-inertness, and end-to-end avoidance."""
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
for _p in ("01_simulation/models", "01_simulation", "04_control", "05_voice_interface",
           "02_identification", "03_validation"):
    sys.path.insert(0, str(ROOT / _p))

from obstacles import (MovingObstacle, StaticBox, SCENE_AUTO_PATH,  # noqa: E402
                       default_moving_obstacles, city_static_obstacles, write_scene_auto)
from avoidance import AvoidancePlanner, clearance  # noqa: E402
from run_autonomous import Mission  # noqa: E402


# --- obstacle geometry -----------------------------------------------------
def test_velocity_is_exact_derivative_of_position():
    for o in default_moving_obstacles(3):
        for t in (0.0, 1.7, 9.3):
            eps = 1e-6
            fd = (o.position(t + eps) - o.position(t - eps)) / (2 * eps)
            assert np.allclose(o.velocity(t), fd, atol=1e-6), o.name


def test_sphere_signed_gap():
    s = MovingObstacle("s", "sphere", 0.5, (0.0, 0.0, 1.0))
    gap, n, _ = s.query(np.array([2.0, 0.0, 1.0]), 0.0)
    assert gap == pytest.approx(1.5) and np.allclose(n, [1, 0, 0])
    gap, _, _ = s.query(np.array([0.2, 0.0, 1.0]), 0.0)
    assert gap < 0                                   # inside


def test_cylinder_gap_radial_vertical_and_inside():
    c = MovingObstacle("c", "cylinder", 0.4, (0.0, 0.0, 0.0), z_lo=0.0, z_hi=2.0)
    gap, n, _ = c.query(np.array([1.4, 0.0, 1.0]), 0.0)          # beside the pillar
    assert gap == pytest.approx(1.0) and np.allclose(n, [1, 0, 0])
    gap, n, _ = c.query(np.array([0.0, 0.0, 3.0]), 0.0)          # above it
    assert gap == pytest.approx(1.0) and np.allclose(n, [0, 0, 1])
    gap, _, _ = c.query(np.array([0.1, 0.0, 1.0]), 0.0)
    assert gap < 0


def test_box_gap_and_inside_exit_direction():
    b = StaticBox("b", (0, 0, 0), (2, 2, 3))
    gap, n, _ = b.query(np.array([4.0, 1.0, 1.0]), 0.0)
    assert gap == pytest.approx(2.0) and np.allclose(n, [1, 0, 0])
    gap, n, _ = b.query(np.array([1.8, 1.0, 1.5]), 0.0)           # inside, nearest face +x
    assert gap == pytest.approx(-0.2) and np.allclose(n, [1, 0, 0])


# --- scenario fairness: a passable gap must always exist -------------------
@pytest.mark.parametrize("seed", range(25))
def test_movers_never_pinch_against_buildings(seed):
    statics = city_static_obstacles()
    ts = np.linspace(0.0, 140.0, 700)
    for o in default_moving_obstacles(seed):
        need = 0.85 if o.shape == "cylinder" else 1.1   # sphere: drone fits between it and a wall
        for t in ts:
            p = o.position(t)
            if o.shape == "cylinder":
                p = np.array([p[0], p[1], 1.2])
            worst = min(b.query(p, 0.0)[0] for b in statics) - o.radius
            assert worst >= need, f"{o.name} gets within {worst:.2f} m of a building at t={t:.1f}"


# --- the generated scene must stay physics-inert ---------------------------
def test_scene_auto_is_physics_inert_vs_scene():
    write_scene_auto(default_moving_obstacles(0))
    m_s = mujoco.MjModel.from_xml_path(str(ROOT / "01_simulation/models/scene.xml"))
    m_a = mujoco.MjModel.from_xml_path(str(SCENE_AUTO_PATH))
    assert (m_a.nq, m_a.nv, m_a.nu, m_a.njnt) == (m_s.nq, m_s.nv, m_s.nu, m_s.njnt)
    assert m_a.nmocap == len(default_moving_obstacles(0))
    d_s, d_a = mujoco.MjData(m_s), mujoco.MjData(m_a)
    rng = np.random.default_rng(7)
    for i in range(1500):
        u = rng.uniform(5.0, 12.0, size=m_s.nu)
        d_s.ctrl[:], d_a.ctrl[:] = u, u
        d_a.mocap_pos[:] = rng.uniform(-3, 3, size=d_a.mocap_pos.shape)  # obstacles roam freely
        mujoco.mj_step(m_s, d_s)
        mujoco.mj_step(m_a, d_a)
        assert np.array_equal(d_s.qpos, d_a.qpos) and np.array_equal(d_s.qvel, d_a.qvel), \
            f"scene_auto changed the drone's physics at step {i}"


# --- planner behaviour -----------------------------------------------------
def test_without_obstacles_the_path_is_straight():
    wps = [np.array([2.0, 0.0, 1.0])]
    pl = AvoidancePlanner(wps, [], p_start=np.array([0.0, 0.0, 1.0]), loop=False)
    pts = []
    for k in range(6000):
        p, _, _ = pl.step(k * 0.002, pl.pr, np.zeros(3), 0.002)
        pts.append(p)
    pts = np.array(pts)
    assert np.linalg.norm(pts[-1] - wps[0]) < 0.5
    assert np.max(np.abs(pts[:, 1])) < 1e-6          # never leaves the line


def test_planner_curves_around_a_blocking_obstacle():
    wall = MovingObstacle("blocker", "cylinder", 0.4, (1.5, 0.0, 0.0), z_lo=0.0, z_hi=3.0)
    pl = AvoidancePlanner([np.array([3.0, 0.0, 1.0])], [wall],
                          p_start=np.array([0.0, 0.0, 1.0]), loop=False)
    pts = []
    for k in range(9000):
        p, _, _ = pl.step(k * 0.002, pl.pr, np.zeros(3), 0.002)
        pts.append(p)
    pts = np.array(pts)
    assert np.linalg.norm(pts[-1] - np.array([3.0, 0.0, 1.0])) < 0.6       # got past it
    assert np.max(np.abs(pts[:, 1])) > 0.5                                  # by going around
    assert min(wall.query(q, 0.0)[0] for q in pts) > 0.3                    # never close


# --- end to end through the real cascade -----------------------------------
def test_avoiding_mission_is_collision_free_and_straight_line_is_not():
    avoid = Mission(1, avoid=True).run(30.0)
    straight = Mission(1, avoid=False).run(30.0)
    sa, sb = avoid.summary(), straight.summary()
    assert sa["collision_time_s"] == 0.0 and sa["min_clearance_m"] > 0.1
    assert not sa["diverged"] and sa["max_tilt_deg"] < 30
    assert sa["waypoints_reached"] >= 2
    assert sa["mean_leg_path_ratio"] > 1.05                  # genuinely curved
    assert sb["collision_time_s"] > 0.0                      # the scenario is not trivial
    assert sb["min_clearance_m"] < 0.0


def test_default_flightcontroller_is_unchanged_by_autonomy_hook():
    from flight import FlightController
    fc = FlightController()
    assert fc.reference is None
