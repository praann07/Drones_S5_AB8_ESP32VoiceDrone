"""Moving + static obstacles for the autonomous-navigation demo.

Single source of truth, like city.py: the planner (04_control/avoidance.py),
the mission runner and the generated MuJoCo scene (scene_auto.xml) all read
the SAME obstacle definitions, so what the planner avoids is exactly what
is drawn.

Moving obstacles follow closed-form harmonic motion
    p(t) = center + amp * sin(omega * t + phase)        (per axis)
so position AND velocity are exact (the planner uses velocity for
time-to-collision prediction) and every run is deterministic. A pillar
patrolling a line, an orbiting sphere and a Lissajous sphere are all just
different (amp, omega, phase) choices.

In MuJoCo they are MOCAP bodies (no mass, no joints) in their own scene
file with contype=conaffinity=0 - visual only, exactly like the city - so
the validated physics scene (scene.xml / quad.xml) is never modified and
tests/test_obstacles.py proves the drone's open-loop physics is unchanged.
Avoidance is the planner's job, not the contact solver's.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

MODELS_DIR = Path(__file__).resolve().parent
SCENE_AUTO_PATH = MODELS_DIR / "scene_auto.xml"


def _unit(v: np.ndarray, fallback=(0.0, 0.0, 1.0)) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else np.asarray(fallback, dtype=float)


@dataclass(frozen=True)
class MovingObstacle:
    """A sphere or a vertical pillar (cylinder spanning z_lo..z_hi) whose
    centre follows center + amp*sin(omega*t + phase)."""

    name: str
    shape: str                    # "sphere" | "cylinder"
    radius: float
    center: tuple                 # (x, y, z) - for a cylinder z is ignored
    amp: tuple = (0.0, 0.0, 0.0)
    omega: tuple = (0.0, 0.0, 0.0)
    phase: tuple = (0.0, 0.0, 0.0)
    z_lo: float = 0.0             # cylinder only
    z_hi: float = 2.2             # cylinder only
    rgba: tuple = (0.9, 0.2, 0.2, 1.0)

    def position(self, t: float) -> np.ndarray:
        c, a = np.asarray(self.center, float), np.asarray(self.amp, float)
        w, ph = np.asarray(self.omega, float), np.asarray(self.phase, float)
        return c + a * np.sin(w * t + ph)

    def velocity(self, t: float) -> np.ndarray:
        a = np.asarray(self.amp, float)
        w, ph = np.asarray(self.omega, float), np.asarray(self.phase, float)
        return a * w * np.cos(w * t + ph)

    def max_speed(self) -> float:
        return float(np.linalg.norm(np.asarray(self.amp) * np.asarray(self.omega)))

    def query(self, p: np.ndarray, t: float):
        """(signed gap to the surface [m], outward unit normal at the
        nearest surface point, obstacle velocity). Negative gap = inside."""
        c = self.position(t)
        v = self.velocity(t)
        if self.shape == "sphere":
            d = p - c
            n = _unit(d)
            return float(np.linalg.norm(d)) - self.radius, n, v
        # vertical pillar: radial distance in xy, plus vertical overshoot
        dxy = p[:2] - c[:2]
        rho = float(np.linalg.norm(dxy))
        radial = dxy / rho if rho > 1e-9 else np.array([1.0, 0.0])
        dr = rho - self.radius
        dz_up, dz_dn = p[2] - self.z_hi, self.z_lo - p[2]
        dz = max(dz_up, dz_dn)
        if dr > 0.0 or dz > 0.0:
            ex, ez = max(dr, 0.0), max(dz, 0.0)
            dist = float(np.hypot(ex, ez))
            nz = 1.0 if dz_up >= dz_dn else -1.0
            n = _unit(np.array([radial[0] * ex, radial[1] * ex, nz * ez]))
            return dist, n, v
        # inside: leave through the nearest face
        if dr >= dz:
            return dr, np.array([radial[0], radial[1], 0.0]), v
        return dz, np.array([0.0, 0.0, 1.0 if dz_up >= dz_dn else -1.0]), v


@dataclass(frozen=True)
class StaticBox:
    """An axis-aligned static obstacle (a city building) in the same
    interface as MovingObstacle."""

    name: str
    lo: tuple
    hi: tuple

    def position(self, t: float) -> np.ndarray:
        return 0.5 * (np.asarray(self.lo) + np.asarray(self.hi))

    def velocity(self, t: float) -> np.ndarray:
        return np.zeros(3)

    def query(self, p: np.ndarray, t: float):
        lo, hi = np.asarray(self.lo, float), np.asarray(self.hi, float)
        closest = np.clip(p, lo, hi)
        diff = p - closest
        dist = float(np.linalg.norm(diff))
        if dist > 1e-9:
            return dist, diff / dist, np.zeros(3)
        # inside the box: exit through the nearest face
        pen = np.concatenate([p - lo, hi - p])           # distance to each face
        k = int(np.argmin(pen))
        n = np.zeros(3)
        n[k % 3] = -1.0 if k < 3 else 1.0
        return -float(pen[k]), n, np.zeros(3)


def city_static_obstacles(near_xy: float = 7.0) -> list[StaticBox]:
    """City buildings/tower within ``near_xy`` metres of the origin as
    boxes (far corner districts can never matter inside the arena)."""
    from city import buildings
    out = []
    for b in buildings():
        if abs(b.x) - b.half_x > near_xy or abs(b.y) - b.half_y > near_xy:
            continue
        out.append(StaticBox(b.name, (b.x - b.half_x, b.y - b.half_y, 0.0),
                             (b.x + b.half_x, b.y + b.half_y, b.height)))
    return out


# --- Mission scenario --------------------------------------------------------
ARENA_HALF_M = 3.6
CRUISE_Z_M = 1.2


def default_waypoints() -> list[np.ndarray]:
    """A patrol loop around the plaza. The east leg passes straight through
    the obstacle tower and every leg is crossed by a moving obstacle, so a
    straight-line flight cannot complete it."""
    return [np.array(w, float) for w in
            ((-2.8, -2.8, 1.2), (2.8, -2.8, 1.2), (2.8, 2.8, 1.5), (-2.8, 2.8, 1.0))]


def default_moving_obstacles(seed: int = 0) -> list[MovingObstacle]:
    """Four moving obstacles; ``seed`` shifts their phases so different
    seeds give different encounters while staying deterministic.

    Fairness constraint (a real design bug found twice while tuning): a mover
    must never seal a corridor. A pillar's extreme must leave >= 0.85 m to the
    nearest building (the open plaza side stays passable); a free-flying
    sphere must leave >= 1.1 m (drone diameter 0.4 + two 0.35 m keep-outs),
    otherwise there is NO passable gap at that instant and no reactive
    planner could succeed - the correct behaviour there would be to wait.
    The pillars' patrol ranges are chosen against block_s / block_n (faces at
    y = -4.05 / +4.05); tests/test_obstacles.py enforces it."""
    r = np.random.default_rng(seed)
    ph = lambda: float(r.uniform(0.0, 2.0 * np.pi))  # noqa: E731
    return [
        # pillar patrolling north-south across the south leg
        MovingObstacle("bus", "cylinder", 0.35, (0.0, -1.9, 0.0),
                       amp=(0.0, 0.9, 0.0), omega=(0.0, 0.45, 0.0),
                       phase=(0.0, ph(), 0.0), rgba=(0.90, 0.25, 0.20, 1.0)),
        # pillar patrolling across the north leg
        MovingObstacle("truck", "cylinder", 0.32, (-1.0, 1.9, 0.0),
                       amp=(0.0, 0.9, 0.0), omega=(0.0, 0.5, 0.0),
                       phase=(0.0, ph(), 0.0), rgba=(0.95, 0.55, 0.10, 1.0)),
        # sphere orbiting the plaza centre at flight height
        MovingObstacle("orbiter", "sphere", 0.35, (-0.8, 0.0, 1.25),
                       amp=(1.5, 1.5, 0.0), omega=(0.5, 0.5, 0.0),
                       phase=(ph() + np.pi / 2, ph(), 0.0), rgba=(0.85, 0.15, 0.55, 1.0)),
        # sphere on a Lissajous path between the plaza and the tower
        MovingObstacle("drifter", "sphere", 0.30, (0.0, 0.0, 1.3),
                       amp=(0.9, 1.5, 0.25), omega=(0.4, 0.6, 0.5),
                       phase=(ph(), ph(), ph()), rgba=(0.60, 0.20, 0.85, 1.0)),
    ]


# --- Generated MuJoCo scene --------------------------------------------------
def scene_auto_xml(obstacles: list[MovingObstacle]) -> str:
    """scene.xml + one mocap body per moving obstacle (visual only)."""
    bodies = []
    for o in obstacles:
        c = np.asarray(o.center, float)
        if o.shape == "sphere":
            geom = f'<geom type="sphere" size="{o.radius}"'
            pos = c
        else:
            half = 0.5 * (o.z_hi - o.z_lo)
            geom = f'<geom type="cylinder" size="{o.radius} {half}"'
            pos = np.array([c[0], c[1], o.z_lo + half])
        rgba = " ".join(f"{v:g}" for v in o.rgba)
        bodies.append(
            f'    <body name="{o.name}" mocap="true" pos="{pos[0]:g} {pos[1]:g} {pos[2]:g}">\n'
            f'      {geom} rgba="{rgba}" contype="0" conaffinity="0" mass="0"/>\n'
            f'    </body>')
    return (
        '<mujoco model="quad_scene_auto">\n'
        '  <!-- GENERATED by obstacles.py::write_scene_auto - do not edit.\n'
        '       scene.xml + moving obstacles as mocap bodies (no mass, no joints,\n'
        '       no collisions): visual only, physics identical to scene.xml. -->\n'
        '  <include file="scene.xml"/>\n'
        '  <worldbody>\n'
        '    <camera name="arena" pos="0 0 10.5" xyaxes="1 0 0 0 1 0" fovy="62"/>\n'
        '    <camera name="arena_side" pos="-7.5 -8.5 6.0" xyaxes="0.75 -0.66 0  0.37 0.42 0.83"/>\n'
        + "\n".join(bodies) + "\n"
        '  </worldbody>\n'
        '</mujoco>\n')


def write_scene_auto(obstacles: list[MovingObstacle] | None = None) -> Path:
    """(Re)write scene_auto.xml. Initial mocap poses are overwritten every
    step by the mission runner, so only the shapes/colours matter here."""
    obstacles = obstacles if obstacles is not None else default_moving_obstacles(0)
    SCENE_AUTO_PATH.write_text(scene_auto_xml(obstacles), encoding="utf-8")
    return SCENE_AUTO_PATH


if __name__ == "__main__":
    print("wrote", write_scene_auto())
