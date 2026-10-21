"""Stage E real-time flight controller (docs/ENGINEERING_PLAN.md Stage E
tasks 3-4). Wraps the ground-truth MuJoCo plant driven THROUGH the Stage D
cascade (outer position loop -> inner quaternion attitude loop), never
bypassing it (docs/CLAUDE.md rules). One ``step()`` advances the plant by
one control period (dt = 0.002 s) exactly as closed_loop_sim.rollout_*
does, so the identified-model-derived gains stay valid.

The plant is MuJoCo but it is accessed only via the cascade controllers,
satisfying the "never wire Stage E directly to the ground-truth simulator,
bypassing the Stage D cascade" rule.

The controller LOADS the frozen model artifact written by
03_validation.run_validation.freeze_selected_model() rather than
re-fitting SINDy from raw excitation parquet files on every launch. This
matters: the whole point of the staged-gate discipline in docs/CLAUDE.md
is that a specific, validated model is what gets to drive a controller or
a demo. Re-fitting cold at Stage E runtime would mean the flight
controller is silently running "whatever STLSQ produces this time" rather
than the exact model that passed Stage B/C - refitting is deterministic
given fixed data (docs/CLAUDE.md "Seeds are deterministic"), so in
practice it would agree, but that is an accident of reproducibility, not
a guarantee, and it costs several seconds of avoidable refitting on every
single launch. If the frozen artifact is missing, this fails loudly
rather than silently falling back to a fresh fit.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
for _p in ("01_simulation", "01_simulation/models", "04_control",
           "02_identification", "03_validation"):
    sys.path.insert(0, str(ROOT / _p))

from sim_driver import C_ROT, C_TRANS, SCENE_PATH  # noqa: E402
from controller import QuaternionController, gains_from_identified_model  # noqa: E402
from position_controller import PositionController, KI_POS_DEFAULT  # noqa: E402
from trajectory import (QuinticBezierTrajectory, CruiseTrajectory, CruiseTurnTrajectory,  # noqa: E402
                        PathTrajectory, MAX_ACCEL_MPS2)
from route_planner import RoutePlanner, CLEARANCE_M  # noqa: E402
from commands import target_offset, parse_goto, GOTO_COMMAND, LAND_COMMAND  # noqa: E402
from sindy_fit import SINDyModel  # noqa: E402
from world import CityWorld  # noqa: E402
from closed_loop_sim import quat_rotate_vector  # noqa: E402

DT = 0.002
# Landing / surface limits (heights are of the body centre above the surface
# below, on top of foot_z, the body-centre height when resting on the feet).
MIN_CLEARANCE_M = 0.3      # "down" never goes lower than this; only "land" does
LAND_APPROACH_M = 0.25     # fly to this height above the surface, then descend
LAND_DESCENT_MPS = 0.3     # final descent speed
TAKEOFF_CLEARANCE_M = 0.6  # height gained before moving off after a landing
MOTOR_SPINDOWN_S = 0.4     # thrust ramps to zero over this after touchdown
IDENTITY_Q = np.array([1.0, 0.0, 0.0, 0.0])
CONTINUOUS_COMMANDS = ("forward", "back", "left", "right")
GEOFENCE_MARGIN_M = 1.0   # continuous flight stops this far inside the map edge
ROUTE_SPEED_MPS = 2.0     # "go to <place>" cruise speed (open-field legs are long)
FROZEN_MODEL_PATH = ROOT / "data" / "processed" / "sindy_fitted_model.npz"


def load_frozen_sindy() -> tuple[SINDyModel, np.ndarray]:
    """Load the Stage C-frozen, gate-passed SINDy model + its hover
    Jacobian. Raises rather than silently refitting: a missing artifact
    means Stage C hasn't been (re-)run since the last identification
    change, which is a configuration error the user must fix, not paper
    over with a surprise multi-second cold refit."""
    if not FROZEN_MODEL_PATH.exists():
        raise RuntimeError(
            f"Frozen model artifact not found at {FROZEN_MODEL_PATH}. "
            "Run `03_validation/run_validation.py` first — it fits, "
            "validates against the Stage B/C gates, and freezes the "
            "selected model for Stage E to load. Stage E refuses to "
            "silently refit an unvalidated model at runtime."
        )
    # allow_pickle=True: this file is written only by our own
    # freeze_selected_model() (object-dtype feature_names array needs it
    # to round-trip) and lives under this project's own data/processed/,
    # never fetched from an untrusted or external source.
    data = np.load(FROZEN_MODEL_PATH, allow_pickle=True)
    sindy = SINDyModel(threshold=float(data["threshold"]))
    sindy.coef_ = data["coef"]
    return sindy, data["A_sindy"]


def _sensor_slice(model: "mujoco.MjModel", name: str) -> slice:
    """(start, start+dim) into data.sensordata for a named sensor, looked
    up by id rather than a hardcoded offset - stays correct even if
    quad.xml's <sensor> block is ever reordered."""
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if sid < 0:
        raise RuntimeError(f"quad.xml has no sensor named {name!r} - "
                            "the IMU <sensor> block must define it.")
    adr = model.sensor_adr[sid]
    dim = model.sensor_dim[sid]
    return slice(adr, adr + dim)


class FlightController:
    def __init__(self, sensor_noise: dict | None = None, seed: int = 0,
                 robust: bool = True, scene_path=None, world=None,
                 continuous: bool = False):
        """sensor_noise (default None = ideal sensors, bit-identical to
        before): optional dict of 1-sigma noise / constant-bias terms the
        CONTROLLER sees, applied only inside measured_state() - the public
        position()/velocity()/attitude()/angular_velocity() accessors stay
        noiseless so logged "truth" is still truth. Keys: gyro_std
        (rad/s), gyro_bias (rad/s, 3-vector), att_std (rad, small-angle
        quaternion noise), pos_std (m), vel_std (m/s). Used by
        run_robustness.py to probe the closed loop, not by the demo.

        robust (default True): enables the two disturbance-rejection
        additions - outer-loop integral action (position_controller.py)
        and yaw-priority rotor desaturation (mixer.py). Both are inert in
        nominal flight (verified: gauntlet/trials/preemption unchanged).
        robust=False reproduces the original Stage D/E gated cascade
        exactly, which is what run_robustness.py compares against.

        continuous (default False = the fixed 1.5 m steps every gate was
        measured with): forward/back/left/right keep flying in that
        direction until a building (world.clamp), the map edge (geofence)
        or the next command; stop/hover then brake to a halt ahead instead
        of turning back. Used by the live demo."""
        self.robust = robust
        self.continuous = continuous
        # obstacle geometry the planner checks against (world.py); must match scene_path
        self.world = world if world is not None else CityWorld()
        # Optional autonomy hook: an object with step(t, p, v, dt) -> (p_d, v_d, a_ff)
        # (04_control/avoidance.py). When set it REPLACES the dispatched
        # trajectory as the outer loop's reference; the cascade below it is
        # unchanged. None (default) = voice/waypoint behaviour exactly as before.
        self.reference = None
        self._noise = dict(sensor_noise or {})
        self._rng = np.random.default_rng(seed)
        self.wind_force = np.zeros(3)   # world-frame N, constant disturbance
        self.wind_fn = None             # optional f(sim_time_s) -> 3-vector N
        sindy, A_sindy = load_frozen_sindy()
        # omega_dot rows, rotor-thrust columns (u1..u4 = library cols 7:11,
        # same slice as run_validation.linearize_sindy_at_hover's B).
        Kq, Komega = gains_from_identified_model(A_sindy, sindy.coef_[3:6, 7:11])
        self.inner = QuaternionController(Kq, Komega, yaw_priority=robust)
        self.outer = PositionController(ki=KI_POS_DEFAULT if robust else 0.0)
        # SCENE_PATH (not MODEL_PATH/quad.xml): adds a ground plane, sky,
        # and lights for actual visibility in the live viewer / offscreen
        # renderer, with byte-identical physics (see sim_driver.py).
        self.model = mujoco.MjModel.from_xml_path(str(scene_path or SCENE_PATH))
        self.data = mujoco.MjData(self.model)
        # quad.xml declares a real IMU (<sensor> block on the "imu" site:
        # framepos/framequat/velocimeter/gyro) - the controller reads
        # state through THESE, not through data.qpos/qvel directly, so
        # "the controller uses the IMU" is literally true, not just
        # modeled-but-unused. Slices are looked up once here, not every
        # step. position/attitude/angular_velocity sensors are direct,
        # noiseless measurements of the exact same quantities qpos/qvel
        # already held, so this changes WHERE the numbers come from, not
        # their value. linear_velocity (the "velocimeter") is the one
        # real exception - see velocity() below.
        self._sens_pos = _sensor_slice(self.model, "position")
        self._sens_att = _sensor_slice(self.model, "attitude")
        self._sens_vel = _sensor_slice(self.model, "linear_velocity")
        self._sens_gyro = _sensor_slice(self.model, "angular_velocity")
        self.traj = None
        self.t_traj = 0.0
        self.command = "hover"
        feet = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"foot{i}")
                for i in range(1, 5)]
        self._feet = {g for g in feet if g >= 0}
        # body-centre height above a surface when standing on the feet
        self.foot_z = (float(min(-self.model.geom_pos[g][2] + self.model.geom_size[g][0]
                                 for g in self._feet)) if self._feet else 0.0)
        # geofence = the floor plane the map is printed on, minus a margin
        g = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        if g >= 0 and self.model.geom_size[g][0] > 0:
            c, h = self.model.geom_pos[g][:2], self.model.geom_size[g][:2]
            self.fence = (c - h + GEOFENCE_MARGIN_M, c + h - GEOFENCE_MARGIN_M)
        else:
            self.fence = None
        self.reset()

    def _run_out_target(self, p0: np.ndarray, command: str) -> np.ndarray:
        """Far end of a continuous move: as far as the geofence allows along
        the command's direction (world.clamp then stops it at a building)."""
        d = target_offset(command)
        d = d / np.linalg.norm(d)
        if self.fence is None:
            return p0 + 50.0 * d
        lo, hi = self.fence
        dist = np.inf
        for k in range(2):
            if d[k] > 1e-9:
                dist = min(dist, (hi[k] - p0[k]) / d[k])
            elif d[k] < -1e-9:
                dist = min(dist, (lo[k] - p0[k]) / d[k])
        return p0 + max(dist, 0.0) * d

    def _braking_target(self, p0: np.ndarray, v0: np.ndarray) -> np.ndarray:
        """Where the drone can come to rest ahead of it at the planner's
        deceleration limit, instead of turning back to where 'stop' was heard."""
        v = v0.copy()
        v[2] = 0.0
        speed = float(np.linalg.norm(v))
        return p0 + v * speed / (2.0 * MAX_ACCEL_MPS2)

    def go_to(self, place: str) -> dict:
        """Fly a curved route around the buildings to just outside `place`
        (a name from world.places()), and hover there. Returns
        {"ok", "target", "length_m", "eta_s", "plan_ms", "route"}."""
        t0 = time.perf_counter()
        self._resume = None
        polygon = self.world.places()[place]
        p0 = self.position()
        if self.phase != "flying":
            # lift off straight up first, then plan from the air
            z = self.surface[0] + self.foot_z + TAKEOFF_CLEARANCE_M
            self.outer.reset_integral()
            self.phase, self._descent_t, self._touch_t = "flying", None, None
            self.traj = QuinticBezierTrajectory(p0, np.array([p0[0], p0[1], max(z, 1.0)]))
            self.t_traj = 0.0
            self.command = "goto"
            self._resume = lambda: self.go_to(place)
            print(f"[route] lifting off, then flying to {place}", flush=True)
            return {"ok": True, "target": None, "length_m": 0.0, "eta_s": 0.0,
                    "plan_ms": 0.0, "route": None}
        if self.fence is None:
            return {"ok": False, "plan_ms": 0.0}
        if getattr(self, "_planner", None) is None:
            self._planner = RoutePlanner(self.world, self.fence)
        z = float(self.traj.position(self.traj.t_total)[2])
        wp = self._planner.plan_to_polygon(p0[:2], polygon, z)
        plan_ms = 1000.0 * (time.perf_counter() - t0)
        if wp is None:
            print(f"[route] no way through to {place} at z={z:.2f} m", flush=True)
            return {"ok": False, "plan_ms": plan_ms}
        v0 = self.velocity()
        W = np.column_stack([wp, np.full(len(wp), z)])
        clear = self._planner.clearance(z)
        for smoothing in (0.15, 0.05, 0.0):           # rounder corners if they stay clear
            traj = PathTrajectory(W, u0=float(np.linalg.norm(v0[:2])), v_max=ROUTE_SPEED_MPS,
                                  smoothing=smoothing)
            pts = traj.samples()[1:-1]
            if all(clear[self._planner.cell(*p[:2])] >= 0.8 * CLEARANCE_M for p in pts):
                break
        self.traj = traj
        self.t_traj = 0.0
        self.command = "goto"
        print(f"[route] to {place}: {traj.distance:.1f} m around the buildings "
              f"(straight line {np.linalg.norm(wp[-1] - p0[:2]):.1f} m), ~{traj.t_total:.0f} s, "
              f"planned in {plan_ms:.0f} ms", flush=True)
        return {"ok": True, "target": traj.p1, "length_m": traj.distance, "eta_s": traj.t_total,
                "plan_ms": plan_ms, "route": traj.samples()}

    def reset(self, p0: np.ndarray | None = None):
        mujoco.mj_resetData(self.model, self.data)
        if p0 is not None:
            self.data.qpos[0:3] = p0
        mujoco.mj_forward(self.model, self.data)
        self.outer.reset_integral()
        self.traj = QuinticBezierTrajectory(self.position(), self.position())
        self.t_traj = 0.0
        self.command = "hover"
        self.phase = "flying"        # flying | landing | landed
        self.surface = self.world.surface_under(*self.position()[:2])
        self._descent_t = None       # seconds into the final descent, while landing
        self._touch_t = None         # seconds since first foot contact, while landing
        self._resume = None          # continuous move to start once a lift-off leg ends

    def status(self) -> str:
        """One-line flight state for the HUD."""
        if self.phase == "landed":
            return f"LANDED on {self.surface[1]}"
        if self.phase == "landing":
            return f"LANDING on {self.surface[1]}"
        return "FLYING"

    def feet_on_surface(self) -> bool:
        c = self.data.contact
        for i in range(self.data.ncon):
            if c[i].geom1 in self._feet or c[i].geom2 in self._feet:
                return True
        return False

    def position(self) -> np.ndarray:
        """World-frame position, read from the IMU's `framepos` sensor
        (not qpos directly) - numerically identical here since the site
        sits at the body origin with no rotation offset, but this is now
        genuinely the sensor reading, not a bypass of it."""
        return self.data.sensordata[self._sens_pos].copy()

    def velocity(self) -> np.ndarray:
        """World-frame velocity. The IMU's `velocimeter` sensor - like a
        real one - reports velocity in the BODY frame, not world frame,
        so it's rotated into world frame here using the (also
        sensor-sourced) attitude - exactly what a real flight computer
        does with an IMU-derived body-frame velocity estimate. This is
        the one accessor where "read the sensor" isn't just a relabeling
        of the same number - the frame transform is real work."""
        v_body = self.data.sensordata[self._sens_vel]
        return quat_rotate_vector(self.attitude(), v_body)

    def attitude(self) -> np.ndarray:
        """Hamilton scalar-first quaternion, read from the IMU's
        `framequat` sensor (not qpos directly) - numerically identical
        here (the site has no rotation offset from the body), but now a
        genuine sensor reading."""
        return self.data.sensordata[self._sens_att].copy()

    def angular_velocity(self) -> np.ndarray:
        """Body-frame angular velocity, read from the IMU's `gyro`
        sensor - the one IMU channel a real gyroscope actually measures
        directly, and already body-frame here exactly like qvel[3:6]
        was, so no transform needed."""
        return self.data.sensordata[self._sens_gyro].copy()

    def measured_state(self):
        """(p, v, q, omega) as the CONTROLLER sees them: the IMU readings
        above plus any configured noise/bias (see __init__). With no noise
        configured this is exactly the four accessors, unchanged."""
        p, v = self.position(), self.velocity()
        q, omega = self.attitude(), self.angular_velocity()
        n = self._noise
        if not n:
            return p, v, q, omega
        r = self._rng
        if n.get("pos_std"):
            p = p + r.normal(0.0, n["pos_std"], 3)
        if n.get("vel_std"):
            v = v + r.normal(0.0, n["vel_std"], 3)
        if n.get("att_std"):
            from controller import quat_multiply
            dq = np.concatenate(([1.0], 0.5 * r.normal(0.0, n["att_std"], 3)))
            q = quat_multiply(q, dq / np.linalg.norm(dq))
        if n.get("gyro_std"):
            omega = omega + r.normal(0.0, n["gyro_std"], 3)
        if "gyro_bias" in n:
            omega = omega + np.asarray(n["gyro_bias"], dtype=float)
        return p, v, q, omega

    def set_payload_scale(self, scale: float) -> None:
        """Scale the plant's mass AND inertia by `scale` (uniform density
        change, e.g. 1.2 = 20% heavier) WITHOUT telling the controller -
        it keeps assuming the identified 1.0 kg. Isolates model mismatch."""
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "quad")
        self.model.body_mass[bid] = 1.0 * scale
        self.model.body_inertia[bid] = np.array([0.003375, 0.003375, 0.006750]) * scale
        mujoco.mj_setConst(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def acceleration(self) -> np.ndarray:
        """World-frame acceleration computed from current thrust vector,
        gravity, and drag. Returns zeros when the drone is not producing
        thrust (e.g., at reset) to avoid passing invalid initial conditions
        to the trajectory."""
        thrust = float(np.sum(self.data.ctrl))
        if thrust < 0.1:
            return np.zeros(3)
        body_z = quat_rotate_vector(self.attitude(), np.array([0.0, 0.0, 1.0]))
        return (thrust / 1.0) * body_z - np.array([0.0, 0.0, 9.81]) - (C_TRANS / 1.0) * self.velocity()

    def dispatch(self, command: str, measure_traj_latency: bool = True,
                 target_position: np.ndarray | None = None) -> dict:
        """Begin a flight to the waypoint for ``command`` (offset applied
        from the current position). Returns the trajectory-generation
        latency in ms (and 0.0 if not measured).

        Preserves current velocity and acceleration for smooth transitions
        between commands (no full stops).

        If target_position is provided, fly directly to that world-frame
        position instead of using the command's fixed offset."""
        t0 = time.perf_counter()
        self._resume = None
        p0 = self.position()
        v0 = self.velocity()
        a0 = self.acceleration()
        if self.phase == "landed" and command in ("hover", "stop", "down", LAND_COMMAND):
            # already on a surface: these mean "stay put", motors stay off
            return {"trajectory": 0.0, "target": p0}
        if command == LAND_COMMAND:
            # Descend onto whatever is under the drone: ground, pad or a roof.
            self.surface = self.world.surface_under(p0[0], p0[1])
            p1 = np.array([p0[0], p0[1], self.surface[0] + self.foot_z + LAND_APPROACH_M])
            self.phase, self._descent_t, self._touch_t = "landing", None, None
            print(f"[land] descending onto {self.surface[1]} "
                  f"(surface at z={self.surface[0]:.2f} m)", flush=True)
        else:
            taking_off = self.phase != "flying"
            self.phase, self._descent_t, self._touch_t = "flying", None, None
            if target_position is not None:
                p1_raw = target_position
            elif self.continuous and command in CONTINUOUS_COMMANDS and not taking_off:
                p1_raw = self._run_out_target(p0, command)
                # keep the COMMANDED altitude (the drone may still be settling
                # into it, e.g. right after a lift-off)
                p1_raw[2] = self.traj.position(self.traj.t_total)[2]
                print(f"[go] {command}: flying until a building, the map edge or "
                      f"'stop' (up to {np.linalg.norm(p1_raw - p0):.1f} m)", flush=True)
            elif self.continuous and command in ("stop", "hover") and not taking_off:
                p1_raw = self._braking_target(p0, v0)
            else:
                p1_raw = p0 + target_offset(command)
            if taking_off:
                self.outer.reset_integral()
                p1_raw = p1_raw.copy()
                p1_raw[2] = max(p1_raw[2], self.surface[0] + self.foot_z + TAKEOFF_CLEARANCE_M)
            if self.continuous and taking_off and command in CONTINUOUS_COMMANDS:
                self._resume = lambda c=command: self.dispatch(c, measure_traj_latency=False)
            # braking ("stop"/"hover") cannot be shortened below the stopping
            # distance, so it only gets the enter-a-building check
            braking = self.continuous and command in ("stop", "hover") and not taking_off
            p1 = self.world.clamp(p0, p1_raw, sweep=not braking)
            if not np.allclose(p1, p1_raw, atol=1e-6):
                print(f"[safety] {command!r} clamped - target would enter a "
                      f"building; stopping short at {p1.round(3).tolist()} "
                      f"instead of {p1_raw.round(3).tolist()}", flush=True)
            # never below the surface underneath (ground or a roof); "land" goes lower
            z_min = self.world.surface_under(p1[0], p1[1])[0] + self.foot_z + MIN_CLEARANCE_M
            if p1[2] < z_min:
                print(f"[safety] {command!r} would go below {MIN_CLEARANCE_M} m above "
                      f"the surface - holding at z={z_min:.2f} m (say 'land' to land)",
                      flush=True)
                p1 = p1.copy()
                p1[2] = z_min

        # Curved, velocity/acceleration-continuous path (carries v0/a0
        # forward as the new leg's boundary conditions) so chained commands
        # don't force a dead stop at the boundary - "forward" then "left"
        # sweeps through the turn instead of stopping, reversing, restarting.
        # A curve that starts from nonzero v0 can bulge outward past the
        # straight p0->p1 line while it bleeds off that carried velocity,
        # so it's re-checked against the city independently of the
        # straight-line clamp above; if it would clip a building anyway,
        # fall back to a from-rest curve that hugs the straight line.
        cruise = (self.continuous and target_position is None and self.phase == "flying"
                  and not taking_off and command in CONTINUOUS_COMMANDS + ("stop", "hover"))
        if cruise:
            # straight line (already clamped clear of buildings), starting at
            # the speed the drone already has along it - no creeping off
            start = np.array([p0[0], p0[1], p1[2]])   # level line at the target height
            d = p1 - start
            n = float(np.linalg.norm(d))
            u0 = float(v0 @ d / n) if n > 1e-9 else 0.0
            traj = CruiseTrajectory(start, p1, u0=u0)
            # A change of direction keeps the sideways velocity and sweeps a
            # rounded corner instead of an exact right angle. Campus world
            # only (path_collides there samples positions), and only if the
            # arc clears every building, else the straight line stays.
            if self.world.name == "campus":
                turn = CruiseTurnTrajectory(start, p1, v0)
                if turn.is_turn and turn.settles and not self.world.path_collides(turn):
                    traj = turn
        else:
            traj = QuinticBezierTrajectory(p0, p1, v0=v0, a0=a0)
        if not cruise and self.world.path_collides(traj):
            print(f"[safety] {command!r} curved approach would clip a "
                  f"building - using a from-rest path instead", flush=True)
            traj = QuinticBezierTrajectory(p0, p1)
        self.traj = traj
        self.t_traj = 0.0
        self.command = command
        dt_ms = 1000.0 * (time.perf_counter() - t0) if measure_traj_latency else 0.0
        return {"trajectory": dt_ms, "target": p1}

    def step(self) -> float:
        """Advance one control period. Returns the per-step control latency
        in ms. Reads current state, runs outer -> inner, applies rotor
        commands to MuJoCo, steps physics."""
        t0 = time.perf_counter()
        if self.phase == "landed":
            # resting on a surface, motors off; contact holds the drone up
            self.data.ctrl[:] = 0.0
            self._apply_forces()
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            self.t_traj += DT
            return 1000.0 * (time.perf_counter() - t0)
        p, v, q, omega = self.measured_state()

        if (self._resume is not None and self.phase == "flying"
                and self.t_traj >= self.traj.t_total):
            then, self._resume = self._resume, None
            then()
        if self.reference is not None:
            p_d, v_d, a_ff = self.reference.step(self.data.time, p, v, DT)
        else:
            t = self.t_traj
            p_d = self.traj.position(t)
            v_d = self.traj.velocity(t)
            a_ff = self.traj.acceleration(t)
        if self.phase == "landing":
            if self._descent_t is None and self.t_traj >= self.traj.t_total:
                self._descent_t = 0.0        # over the spot: start the final descent
            if self._descent_t is not None:
                ground = self.surface[0] + self.foot_z - 0.03  # aim just below contact
                z = max(self.traj.p1[2] - LAND_DESCENT_MPS * self._descent_t, ground)
                p_d = np.array([self.traj.p1[0], self.traj.p1[1], z])
                v_d = np.array([0.0, 0.0, -LAND_DESCENT_MPS if z > ground else 0.0])
                a_ff = np.zeros(3)
                self._descent_t += DT
        # heading_mode stays "fixed": "forward_facing" is implemented and
        # unit-tested in isolation (position_controller.py), but wiring it
        # in here was tried and measured - it destabilizes the cascade
        # (some trials tumbled, position error 30-50m) because the inner
        # loop's gains (gains_from_identified_model) and the outer/inner
        # bandwidth separation were derived and gated under Stage D's
        # explicit "yaw is never commanded" assumption; sustained yaw
        # tracking was never part of what was validated. Enabling it live
        # needs its own inner-loop re-validation first, not a flip here.
        q_d, thrust_d = self.outer.compute(p, v, p_d, v_d, a_ff,
                                            heading_mode="fixed")
        u, _ = self.inner.compute(q, omega, q_d, np.zeros(3), thrust_d)

        if self.phase == "landing":
            if self._touch_t is None and self.feet_on_surface():
                self._touch_t = 0.0
                self.outer.reset_integral()
            if self._touch_t is not None:    # touched down: spin the motors down
                u = u * max(0.0, 1.0 - self._touch_t / MOTOR_SPINDOWN_S)
                self._touch_t += DT
                if self._touch_t >= MOTOR_SPINDOWN_S:
                    self.phase = "landed"
                    print(f"[land] touched down on {self.surface[1]}, motors off", flush=True)

        self.data.ctrl[:] = u
        self._apply_forces()
        mujoco.mj_step(self.model, self.data)
        # mj_step() computes sensors from the PRE-integration state (its
        # own internal mj_forward runs before qpos/qvel are advanced), so
        # data.sensordata is one full step stale relative to the qpos/qvel
        # it just integrated to. mj_forward() here recomputes sensors
        # (cheap - no integration) from the
        # now-current qpos/qvel, so the NEXT position()/velocity()/etc.
        # call reads a fresh IMU reading, not a one-step-lagged one.
        mujoco.mj_forward(self.model, self.data)
        self.t_traj += DT
        return 1000.0 * (time.perf_counter() - t0)

    def _apply_forces(self) -> None:
        """Aerodynamic drag plus any injected wind, as applied generalized forces."""
        self.data.qfrc_applied[0:3] = -C_TRANS * self.data.qvel[0:3] + self.wind_force
        if self.wind_fn is not None:
            self.data.qfrc_applied[0:3] += self.wind_fn(self.data.time)
        self.data.qfrc_applied[3:6] = -C_ROT * self.data.qvel[3:6]

    def attitude_error_deg(self, q_target: np.ndarray) -> float:
        from controller import quat_multiply, quat_conjugate
        qe = quat_multiply(quat_conjugate(q_target), self.attitude())
        w = float(np.clip(abs(qe[0]), -1.0, 1.0))
        return float(np.degrees(2 * np.arccos(w)))
