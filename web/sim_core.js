// Browser/Node port of the flight stack: the SAME cascade as
// 05_voice_interface/flight.py (quintic Bezier planner -> PD(+I) position
// loop -> quaternion P^2 attitude loop with gains derived from the
// identified SINDy model -> inverse mixer -> rigid-body plant).
//
// Every number comes from params.js, which export_web_sim.py writes from
// the Python sources (frozen SINDy artifact, controller.py, mixer.py,
// trajectory.py, city.py). Nothing is re-tuned here.
//
// Plant: the 6-DOF rigid body MuJoCo integrates for quad.xml (mass,
// diagonal inertia, 4 thrust actuators with yaw reaction, linear drag),
// RK4 with the control held over each physics step like mj_step. Checked
// against the Python/MuJoCo cascade by tests/test_web_sim_parity.py.

// ---------------------------------------------------------------- vectors
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);

// Hamilton, scalar-first [w,x,y,z] (docs/TDD.md section 1)
export function qmul(a, b) {
  const [w1, x1, y1, z1] = a, [w2, x2, y2, z2] = b;
  return [
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
  ];
}
const qconj = (q) => [q[0], -q[1], -q[2], -q[3]];
export function qrot(q, v) {
  return qmul(qmul(q, [0, v[0], v[1], v[2]]), qconj(q)).slice(1);
}
const qnormalize = (q) => { const n = Math.hypot(...q); return q.map((c) => c / n); };

// ---------------------------------------------------------------- mixer
export function makeMixer(P) {
  const { ARM: a, K_M: k } = P.mixer;
  // Rows [T, tau_x, tau_y, tau_z], columns u1..u4 (mixer.py::mixer_matrix)
  const M = [[1, 1, 1, 1], [a, -a, -a, a], [-a, a, -a, a], [k, k, -k, -k]];
  const Minv = P.mixer.M_INV;
  const mul = (w) => Minv.map((row) => row[0] * w[0] + row[1] * w[1] + row[2] * w[2] + row[3] * w[3]);
  const umax = P.mixer.U_MAX;
  const feasible = (u) => u.every((x) => x >= 0 && x <= umax);
  const clip = (u) => u.map((x) => Math.min(Math.max(x, 0), umax));
  function inverseMix(T, tau) {
    const u = mul([T, tau[0], tau[1], tau[2]]);
    return [clip(u), !feasible(u)];
  }
  // mixer.py::inverse_mix_yaw_priority - shed yaw, then roll/pitch, keep thrust
  function inverseMixYawPriority(T, tau) {
    const u = mul([T, tau[0], tau[1], tau[2]]);
    if (feasible(u)) return [u, false];
    const mix = (sxy, sz) => mul([T, tau[0] * sxy, tau[1] * sxy, tau[2] * sz]);
    if (feasible(mix(1, 0))) {
      let lo = 0, hi = 1;
      for (let i = 0; i < 30; i++) { const m = 0.5 * (lo + hi); if (feasible(mix(1, m))) lo = m; else hi = m; }
      return [mix(1, lo), true];
    }
    if (feasible(mix(0, 0))) {
      let lo = 0, hi = 1;
      for (let i = 0; i < 30; i++) { const m = 0.5 * (lo + hi); if (feasible(mix(m, 0))) lo = m; else hi = m; }
      return [mix(lo, 0), true];
    }
    return [clip(mix(0, 0)), true];
  }
  const wrench = (u) => M.map((row) => row[0] * u[0] + row[1] * u[1] + row[2] * u[2] + row[3] * u[3]);
  return { inverseMix, inverseMixYawPriority, wrench };
}

// ---------------------------------------------------------------- inner loop
export function attitudeTorque(Kq, Ko, q, omega, qd) {
  let qe = qmul(qconj(qd), q);
  if (qe[0] < 0) qe = qe.map((c) => -c);   // short way round (controller.py)
  return [0, 1, 2].map((i) => -Kq[i] * qe[i + 1] - Ko[i] * omega[i]);
}
export function attitudeErrorDeg(q, qd) {
  const qe = qmul(qconj(qd), q);
  return (2 * Math.acos(Math.min(Math.abs(qe[0]), 1)) * 180) / Math.PI;
}

// ---------------------------------------------------------------- outer loop
function minimalRotationZTo(v) {
  const n = norm(v); v = scale(v, 1 / n);
  const axis = cross([0, 0, 1], v);
  const s = norm(axis), c = v[2];
  if (s < 1e-9) return c > 0 ? [1, 0, 0, 0] : [0, 1, 0, 0];
  const ang = Math.atan2(s, c);
  const k = Math.sin(ang / 2) / s;
  return [Math.cos(ang / 2), axis[0] * k, axis[1] * k, axis[2] * k];
}

export class PositionController {
  constructor(P, ki) { this.P = P.outer; this.ki = ki; this.dt = P.DT; this.int = [0, 0, 0]; this.mass = P.MASS; }
  compute(p, v, pd, vd, aff) {
    const O = this.P;
    const ep = sub(pd, p), ev = sub(vd, v);
    let a = add(add(scale(ep, O.KP), scale(ev, O.KD)), aff);
    if (this.ki > 0) {
      if (norm(ep) < O.I_ACTIVE_RADIUS_M && norm(vd) < O.I_ACTIVE_SPEED_MPS && norm(v) < O.I_ACTIVE_VEL_MPS) {
        const lim = O.I_MAX_ACCEL_MPS2 / this.ki;
        this.int = this.int.map((x, i) => Math.min(Math.max(x + ep[i] * this.dt, -lim), lim));
      }
      a = add(a, scale(this.int, this.ki));
    }
    const aVert = a[2] + O.GRAVITY;
    const lat = Math.hypot(a[0], a[1]);
    const maxLat = Math.max(aVert, 0) * Math.tan((O.MAX_BANK_DEG * Math.PI) / 180);
    if (lat > maxLat && lat > 1e-9) { const s = maxLat / lat; a = [a[0] * s, a[1] * s, a[2]]; }
    const f = add(a, [0, 0, O.GRAVITY]);
    return [minimalRotationZTo(f), this.mass * norm(f)];
  }
}

// ---------------------------------------------------------------- planner
function bez(t, P) {
  const u = 1 - t;
  const c = [u ** 5, 5 * u ** 4 * t, 10 * u ** 3 * t ** 2, 10 * u ** 2 * t ** 3, 5 * u * t ** 4, t ** 5];
  return [0, 1, 2].map((i) => c.reduce((s, ci, k) => s + ci * P[k][i], 0));
}
function bez1(t, P) {
  const u = 1 - t, d = (a, b) => sub(P[a], P[b]);
  const terms = [[5 * u ** 4, d(1, 0)], [20 * u ** 3 * t, d(2, 1)], [10 * u ** 2 * t ** 2, d(3, 2)],
    [10 * u * t ** 3, d(4, 3)], [5 * t ** 4, d(5, 4)]];
  return terms.reduce((s, [c, v]) => add(s, scale(v, c)), [0, 0, 0]);
}
function bez2(t, P) {
  const u = 1 - t, d2 = (a, b, c) => add(sub(P[a], scale(P[b], 2)), P[c]);
  const terms = [[20 * u ** 3, d2(2, 1, 0)], [30 * u ** 2 * t, d2(3, 2, 1)], [30 * u * t ** 2, d2(4, 3, 2)],
    [20 * t ** 3, d2(5, 4, 3)]];
  return terms.reduce((s, [c, v]) => add(s, scale(v, c)), [0, 0, 0]);
}
function bez3(t, P) {
  const u = 1 - t;
  const d3 = (a, b, c, d) => add(sub(P[a], scale(P[b], 3)), sub(scale(P[c], 3), P[d]));
  const terms = [[60 * u ** 2, d3(3, 2, 1, 0)], [120 * u * t, d3(4, 3, 2, 1)], [60 * t ** 2, d3(5, 4, 3, 2)]];
  return terms.reduce((s, [c, v]) => add(s, scale(v, c)), [0, 0, 0]);
}

// trajectory.py::QuinticBezierTrajectory, same bisection and constraint set
export class QuinticBezier {
  constructor(P, p0, p1, v0 = [0, 0, 0], a0 = [0, 0, 0]) {
    const L = P.traj;
    this.L = L; this.p0 = p0; this.p1 = p1; this.a0 = a0;
    const vn = norm(v0);
    this.v0 = vn > 0.5 * L.MAX_VELOCITY_MPS ? scale(v0, (0.5 * L.MAX_VELOCITY_MPS) / vn) : v0;
    this.distance = norm(sub(p1, p0));
    this.T = this.duration();
    this.cp = this.points(this.T);
  }
  points(T) {
    const { p0, p1, v0, a0 } = this;
    return [p0, add(p0, scale(v0, T / 5)), add(add(p0, scale(v0, (2 * T) / 5)), scale(a0, (T * T) / 20)), p1, p1, p1];
  }
  feasible(T) {
    if (T < 1e-9) return false;
    const P = this.points(T), L = this.L;
    for (let k = 0; k < 100; k++) {
      const t = k / 99;
      if (norm(bez1(t, P)) / T > L.MAX_VELOCITY_MPS) return false;
      const a = scale(bez2(t, P), 1 / (T * T));
      if (norm(a) > L.MAX_ACCEL_MPS2) return false;
      if (norm(bez3(t, P)) / T ** 3 > L.MAX_JERK_MPS3) return false;
      if (norm(add(a, [0, 0, 9.81])) > L.MAX_THRUST_N) return false;
    }
    return true;
  }
  duration() {
    if (this.distance < 1e-9) return 0;
    let lo = this.distance / this.L.MAX_VELOCITY_MPS, hi = 10 * lo + 1;
    for (let i = 0; i < 50; i++) { const T = 0.5 * (lo + hi); if (this.feasible(T)) hi = T; else lo = T; }
    return hi;
  }
  tau(t) { return Math.min(Math.max(t / this.T, 0), 1); }
  position(t) { return this.T < 1e-9 ? this.p0 : bez(this.tau(t), this.cp); }
  velocity(t) { return this.T < 1e-9 ? [0, 0, 0] : scale(bez1(this.tau(t), this.cp), 1 / this.T); }
  acceleration(t) { return this.T < 1e-9 ? [0, 0, 0] : scale(bez2(this.tau(t), this.cp), 1 / this.T ** 2); }
}

// ---------------------------------------------------------------- safety
// city.py::clamp_target_to_safe_zone (slab ray-vs-AABB), same margin
export function clampToSafeZone(P, p0, p1) {
  const d = sub(p1, p0), len = norm(d);
  if (len < 1e-9) return p1;
  let best = 1;
  for (const b of P.city.filter((o) => o.kind === "building" || o.kind === "tower")) {
    const lo = [b.x - b.half_x, b.y - b.half_y, 0], hi = [b.x + b.half_x, b.y + b.half_y, b.height];
    let te = 0, tx = 1, hit = true;
    for (let i = 0; i < 3; i++) {
      if (Math.abs(d[i]) < 1e-9) { if (p0[i] < lo[i] || p0[i] > hi[i]) { hit = false; break; } continue; }
      let t1 = (lo[i] - p0[i]) / d[i], t2 = (hi[i] - p0[i]) / d[i];
      if (t1 > t2) [t1, t2] = [t2, t1];
      te = Math.max(te, t1); tx = Math.min(tx, t2);
      if (te > tx) { hit = false; break; }
    }
    if (!hit || te > 1 || tx < 0) continue;
    best = Math.min(best, Math.max(0, te - P.safety.SAFETY_MARGIN_M / len));
  }
  return best >= 1 - 1e-9 ? p1 : add(p0, scale(d, best));
}

// Sampled stand-in for city.py::check_bezier_collision (that one subdivides
// the control polygon; sampling 200 points of a <=3 m curve at 0.3 m
// margin is equivalent at this scale).
function curveHitsCity(P, traj) {
  const m = P.safety.SAFETY_MARGIN_M + P.safety.DRONE_RADIUS_M;
  const bs = P.city.filter((o) => o.kind === "building" || o.kind === "tower");
  for (let k = 0; k <= 200; k++) {
    const p = traj.position((k / 200) * traj.T);
    for (const b of bs) {
      if (Math.abs(p[0] - b.x) < b.half_x + m && Math.abs(p[1] - b.y) < b.half_y + m && p[2] < b.height + m) return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------- classifier
// difflib.SequenceMatcher(None, a, b).ratio() (Ratcliff/Obershelp)
function matches(a, b) {
  if (!a.length || !b.length) return 0;
  let best = 0, ia = 0, ib = 0;
  for (let i = 0; i < a.length; i++) for (let j = 0; j < b.length; j++) {
    let k = 0; while (i + k < a.length && j + k < b.length && a[i + k] === b[j + k]) k++;
    if (k > best) { best = k; ia = i; ib = j; }
  }
  if (!best) return 0;
  return best + matches(a.slice(0, ia), b.slice(0, ib)) + matches(a.slice(ia + best), b.slice(ib + best));
}
export const ratio = (a, b) => (2 * matches(a, b)) / (a.length + b.length);
function closest(word, options, cutoff) {
  let best = null, br = -1;
  for (const o of options) { const r = ratio(word, o); if (r >= cutoff && (r > br || (r === br && o > best))) { br = r; best = o; } }
  return best ? [best, br] : null;
}
// commands.py::classify (synonyms -> whole phrase -> globally best token)
export function classify(P, transcript) {
  const V = P.vocab;
  const text = (transcript || "").toLowerCase().trim();
  if (!text) return null;
  const toks = text.split(/\s+/);
  for (const t of toks) if (V.synonyms[t]) return V.synonyms[t];
  const whole = closest(text, V.commands, V.CUTOFF);
  if (whole) return whole[0];
  let bestCmd = null, bestR = 0;
  for (const t of toks) {
    const h = closest(t, V.commands, t.length < V.SHORT_TOKEN_LEN ? V.SHORT_TOKEN_CUTOFF : V.CUTOFF);
    if (h && h[1] > bestR) { bestR = h[1]; bestCmd = h[0]; }
  }
  return bestCmd;
}

// ---------------------------------------------------------------- RNG
// mulberry32 - seeded so a noise run is repeatable
export function rng(seed) {
  let a = seed >>> 0;
  const u = () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  return { u, normal: () => Math.sqrt(-2 * Math.log(u() + 1e-300)) * Math.cos(2 * Math.PI * u()) };
}

// ---------------------------------------------------------------- flight
export class Flight {
  /** opts: gains ("fixed" | "asbuilt"), robust (bool), controlEvery (int,
   *  physics steps per control update), physicsDt (s), gyroStd (rad/s),
   *  wind ([N,N,N]), payload (mass/inertia scale on the PLANT only), seed. */
  constructor(P, opts = {}) {
    this.P = P;
    this.o = { gains: "fixed", robust: true, controlEvery: 1, physicsDt: P.DT, gyroStd: 0, wind: [0, 0, 0], payload: 1, seed: 0, ...opts };
    this.mixer = makeMixer(P);
    this.rand = rng(this.o.seed);
    this.reset();
  }
  reset() {
    const P = this.P;
    this.t = 0; this.k = 0; this.tTraj = 0;
    this.p = [...P.HOVER_POS]; this.v = [0, 0, 0]; this.q = [1, 0, 0, 0]; this.w = [0, 0, 0];
    this.u = [0, 0, 0, 0]; this.sat = false;   // MuJoCo ctrl is 0 until the first step
    this.outer = new PositionController(P, this.o.robust ? P.outer.KI_DEFAULT : 0);
    this.traj = new QuinticBezier(P, [...this.p], [...this.p]);
    this.command = "hover"; this.qd = [1, 0, 0, 0]; this.target = [...this.p];
  }
  // flight.py::acceleration - thrust-model estimate, zero before any thrust
  accel() {
    const T = this.u[0] + this.u[1] + this.u[2] + this.u[3];
    if (T < 0.1) return [0, 0, 0];
    return sub(add(scale(qrot(this.q, [0, 0, 1]), T / this.P.MASS), [0, 0, -this.P.GRAVITY]), scale(this.v, this.P.C_TRANS / this.P.MASS));
  }
  setOption(k, v) { this.o[k] = v; if (k === "robust") this.outer.ki = v ? this.P.outer.KI_DEFAULT : 0; if (k === "seed") this.rand = rng(v); }
  gains() { const g = this.o.gains === "asbuilt" ? this.P.gains_asbuilt : this.P.gains; return [g.Kq, g.Komega]; }

  dispatch(command, targetPosition = null) {
    const off = this.P.vocab.offsets[command];
    const p1raw = targetPosition || add(this.p, off || [0, 0, 0]);
    const p1 = clampToSafeZone(this.P, this.p, p1raw);
    let traj = new QuinticBezier(this.P, [...this.p], p1, [...this.v], this.accel());
    if (curveHitsCity(this.P, traj)) traj = new QuinticBezier(this.P, [...this.p], p1);
    this.traj = traj; this.tTraj = 0; this.command = command; this.target = p1;
    return { target: p1, clamped: norm(sub(p1, p1raw)) > 1e-6, duration: traj.T };
  }

  control() {
    const n = this.o.gyroStd > 0 ? [0, 1, 2].map(() => this.o.gyroStd * this.rand.normal()) : [0, 0, 0];
    const wMeas = add(this.w, n);
    const t = this.tTraj;
    const [qd, T] = this.outer.compute(this.p, this.v, this.traj.position(t), this.traj.velocity(t), this.traj.acceleration(t));
    const [Kq, Ko] = this.gains();
    const tau = attitudeTorque(Kq, Ko, this.q, wMeas, qd);
    [this.u, this.sat] = this.o.robust ? this.mixer.inverseMixYawPriority(T, tau) : this.mixer.inverseMix(T, tau);
    this.qd = qd;
  }

  deriv(s, u) {
    const P = this.P, m = P.MASS * this.o.payload, I = P.INERTIA.map((x) => x * this.o.payload);
    const [, tx, ty, tz] = this.mixer.wrench(u);
    const T = u[0] + u[1] + u[2] + u[3];
    const { p, v, q, w } = s;
    const f = add(add(scale(qrot(q, [0, 0, T]), 1), scale(v, -P.C_TRANS)), this.o.wind);
    const a = add(scale(f, 1 / m), [0, 0, -P.GRAVITY]);
    const Iw = [I[0] * w[0], I[1] * w[1], I[2] * w[2]];
    const gyro = cross(w, Iw);
    const tau = [tx - P.C_ROT * w[0] - gyro[0], ty - P.C_ROT * w[1] - gyro[1], tz - P.C_ROT * w[2] - gyro[2]];
    const wd = [tau[0] / I[0], tau[1] / I[1], tau[2] / I[2]];
    const qd = qmul(q, [0, w[0], w[1], w[2]]).map((c) => 0.5 * c);
    return { p: v, v: a, q: qd, w: wd };
  }

  integrate(h) {
    const s0 = { p: this.p, v: this.v, q: this.q, w: this.w }, u = this.u;
    const shift = (s, d, c) => ({ p: add(s.p, scale(d.p, c)), v: add(s.v, scale(d.v, c)), q: s.q.map((x, i) => x + d.q[i] * c), w: add(s.w, scale(d.w, c)) });
    const k1 = this.deriv(s0, u), k2 = this.deriv(shift(s0, k1, h / 2), u), k3 = this.deriv(shift(s0, k2, h / 2), u), k4 = this.deriv(shift(s0, k3, h), u);
    const comb = (key) => (key === "q" ? s0.q.map((x, i) => x + (h / 6) * (k1.q[i] + 2 * k2.q[i] + 2 * k3.q[i] + k4.q[i]))
      : add(s0[key], scale(add(add(k1[key], scale(k2[key], 2)), add(scale(k3[key], 2), k4[key])), h / 6)));
    this.p = comb("p"); this.v = comb("v"); this.q = qnormalize(comb("q")); this.w = comb("w");
  }

  /** Advance one physics step (control re-evaluated every controlEvery steps). */
  step() {
    if (this.k % this.o.controlEvery === 0) this.control();
    const h = this.o.physicsDt;
    this.integrate(h);
    this.k += 1; this.t += h; this.tTraj += h;
  }

  diverged() { return !this.p.every(Number.isFinite) || Math.abs(this.p[2]) > 50 || attitudeErrorDeg(this.q, [1, 0, 0, 0]) > 120; }
}
