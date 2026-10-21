## Attempt 2 · `drone_mujoco` — A Voice-Controlled Drone Whose Controller Is Built From Learned Physics

**In one sentence:** we let a computer *discover* the equation that governs how a drone rotates, by watching the drone fly, and then *calculated* the controller from that equation, so that a person can say "go to the central library" and the drone flies there across a scale model of our own campus.

<p align="center">
  <a href="https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/blob/drone_mujoco/docs/recordings/campus_demo.mp4">
    <img src="https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/drone_mujoco/docs/recordings/campus_demo.gif" alt="Campus demo: take-off, cruise, stop, turn, routed flight to a building, landing" width="95%">
  </a>
</p>

<p align="center"><i>The real simulation, 31 seconds (click for the MP4). Left: chase camera. Right: bird's-eye view. Bottom: live speed. Each <b>"stop"</b> brings the drone to rest in <b>0.92 s over 0.50 m</b>.</i></p>

---

### 1. The problem, in plain words

A quadrotor has four propellers and no steering wheel. The only things we can change are **four numbers: how hard each propeller pushes**. Everything the drone does (tilting, turning, moving forward, holding still) has to be achieved by choosing those four numbers, hundreds of times per second.

A **controller** is the rule that picks them. To design one well you need a **model**: an equation that predicts what the drone will do for a given push. The usual approach is to write the model down from textbook physics and measured parameters (mass, inertia, drag). That has two weaknesses:

- the textbook parameters are never exactly right for the real machine, and
- the controller then depends on numbers that nobody checked against the machine's actual behaviour.

**Our approach is the reverse.** Record how the drone actually behaves, let an algorithm find the equation, check that equation hard, and only then build the controller from it. This is what *data-driven control* means.

### 2. The idea, in five steps

| Step | What we do | In plain words |
|---|---|---|
| **A. Excite** | Push the four propellers with random and sweeping signals, with no controller, and record everything. | Shake the drone in every way, so the recording contains the physics. |
| **B. Identify** | Fit **SINDy** (and, for comparison, **DMDc**) to the recording. | Ask a computer: "what short equation explains this motion?" |
| **C. Validate** | Test the equation on flights it has never seen, including 11-second blind predictions. | Try to break it. Keep it only if it survives. |
| **D. Control** | Compute the controller's gains **from the learned equation**. | The equation sets the settings, not trial and error. |
| **E. Command** | Add speech recognition, route planning and the campus map. | Say a place, and the drone flies there. |

Each stage had a pass/fail gate, and the next stage did not start until the previous one passed.

### 3. The mathematics, step by step

This section is for a reader who wants the actual equations. Every symbol is defined where it first appears.

#### 3.1 What is being modelled

The drone's **state** is its tilt angles `θ = (θx, θy, θz)` and its rotation rates `ω = (ωx, ωy, ωz)`. The **input** is the four propeller thrusts `u = (u1, u2, u3, u4)`.

For a rigid body, physics gives the structure (Euler's rotation equation):

```
 θ̇  ≈  ω                                         (angle changes at the rotation rate)
 I·ω̇ =  M·u  −  ω × (I·ω)  −  c·ω               (torque from propellers, gyroscopic coupling, drag)
```

`I` is the inertia, `M` maps propeller thrusts to torques, and `c` is the drag coefficient. **The controller does not use these textbook values.** SINDy has to find them from data.

#### 3.2 SINDy: finding a short equation (Sparse Identification of Nonlinear Dynamics)

We record the state, the input and the rate of change of the state at thousands of moments, and stack them:

```
 Ẋ  ≈  Θ(X, U) · Ξ
```

- `Ẋ` holds the measured rates of change (one row per moment, one column per state variable: 6 columns).
- `Θ(X, U)` is a **library**: a table of 26 candidate terms evaluated at every moment (a constant, `θ`, `ω`, `u`, the products `ωi·ωj` and `θi·ωj`).
- `Ξ` is the unknown 26 × 6 coefficient matrix. **That is 156 numbers, and SINDy keeps 44.**

The library is chosen with physics in mind: it contains only terms that rigid-body dynamics could produce, and leaves out terms with no physical reason to exist (`θ·θ`, `u·u`, `θ·u`, `ω·u`). A blind polynomial would have 66 terms; ours has 26, so the method discovers physics instead of fitting noise.

**Why "sparse"?** Ordinary least squares uses every candidate a little and gives a messy answer. SINDy uses **STLSQ** (sequentially thresholded least squares):

```
 repeat until nothing changes:
     Ξ  ←  argmin ‖ Ẋ − Θ·Ξ ‖²              fit all surviving terms
     set every coefficient with |Ξij| < λ to 0   drop the unimportant ones
```

The result is a short, readable equation. Two practical points matter for correctness:

1. **Smooth before differentiating.** `Ẋ` is not measured; it is computed from samples, and differentiating raw data multiplies noise. A **Savitzky-Golay filter** (a local polynomial fit over a sliding window) is applied first.
2. **Never pair samples across a reset.** Attitude is held in local coordinates and re-referenced at segment boundaries. A pair that straddles a reset has a fake jump of about 2.6 rad against about 0.002 rad of real motion; squared, one such pair outweighs a million real ones.

**Evidence that it found physics, not just a curve.** The simulator's true parameters are known, so the learned coefficients can be compared with them: control effectiveness, drag, the gyroscopic term and the kinematic term all agree **within 2–6 %**, and a test enforces this.

#### 3.3 DMDc: the linear comparison (Dynamic Mode Decomposition with control)

DMDc fits the simplest possible model on the same data:

```
 x(k+1)  =  A·x(k)  +  B·u(k)          A, B from a least-squares fit:  [A B] = X₊ · [X; U]†
```

It is fast and has exactly computable stability, but it cannot represent products of states. So it works **near hover** (R² 0.9926) and degrades to a one-step NRMSE of about **0.70** away from it (1.0 means doing nothing). It is reported as valid near hover only, and it serves as a fair baseline for SINDy.

#### 3.4 How the learned equation is judged

Two scores are used, both on **held-out trials** (separate recordings never shown to the fit, not a slice of one recording):

```
 R²     = 1 − Σ(y − ŷ)² / Σ(y − ȳ)²         1.0 is perfect; measures the fraction of behaviour explained
 NRMSE  = RMS(ŷ − y) / RMS(Δy)              error relative to how much the state changes in one step;
                                            a model that does nothing scores exactly 1.0
```

The tests, from easiest to hardest:

| Test | Result for SINDy |
|---|---|
| One-step prediction, held-out | **R² 0.9946**, NRMSE **0.0786** |
| 11-second blind rollout, full flight envelope | **0 of 7 diverged**, mean final error 0.095 (limit 0.5) |
| Coefficients vs the simulator's true values | within **2–6 %** |
| Noise ablation (add gyro noise) | holds to about 1 rad/s, knee at 2 rad/s, fails by 4 rad/s |
| Sparsity ablation (raise the threshold λ) | stable above 0.2, collapses at 1.0 |
| Head to head with DMDc in DMDc's own regime | SINDy 0.0789 vs DMDc 0.0825 (SINDy matches it, and also works everywhere else) |

**Eigenvalues.** Linearising at hover gives six poles; three sit on the stability boundary. That is the correct answer, not a defect: a free rigid body has no restoring force, so `θ̇ = ω` gives a marginal pole for each axis. The other three are strictly stable.

#### 3.5 From the learned equation to the controller

The controller is a **cascade**: a slow outer loop (position) wrapped around a fast inner loop (attitude).

**Inner loop (uses the learned model).** Orientation is represented by a **quaternion** (four numbers with unit length), which has no gimbal lock and no 359°→0° jumps. The error quaternion `qe = q_wanted ⊗ q*` has a vector part `qe,vec` that is approximately half the angle error. The control law is

```
 τ  =  − Kq · qe,vec  −  Kω · ω                  (a spring on the angle, a damper on the rate)
```

Near hover each axis, as identified by SINDy, behaves like `ω̇ = a·ω + b·τ`, where `a` is negative (it equals −drag/inertia) and `b` is the control effectiveness (1/inertia). Substituting the control law gives the closed loop

```
 ë  +  (b·Kω − a)·ė  +  (b·Kq / 2)·e  =  0       compare with  s² + 2ζωn·s + ωn² = 0
```

so matching coefficients **fixes the gains from the model**:

```
 ωn = 4 / (ζ · Ts)       Kq = 2·ωn² / b       Kω = (2·ζ·ωn + a) / b
```

Only two numbers are chosen by hand: the settling time `Ts = 0.5 s` and the damping ratio `ζ = 0.7`. Everything else comes from the learned `a` and `b`. If the model changes, the gains recompute. Result: the tilt settles in **0.56–0.86 s**, and the learned-model simulation and the full MuJoCo simulation agree to about **0.3 %**.

**Outer loop (ordinary physics, deliberately not learned).** A drone moves by tilting, which points part of its thrust sideways. Translation under a thrust vector is Newton's law, `m·p̈ = R(q)·[0, 0, T] − m·g·ẑ`, a plain double integrator that has nothing for a data-driven method to discover. So the outer loop is a PD law with a gated integral term:

```
 a_cmd = Kp·(p_ref − p) + Kd·(v_ref − v) + Ki·∫(p_ref − p)dt          Kp = ωn², Kd = 2ζωn  (ωn = 2, ζ = 1)
 thrust vector = m·(a_cmd + g·ẑ)     →   its length is total thrust, its direction is the tilt asked of the inner loop
```

The integral acts only when both the planned path and the drone are nearly at rest, which removes steady offsets (wind, wrong mass) without adding lag during a move.

**Mixer.** The four propeller thrusts are recovered from (total thrust, three torques) by inverting the known X-configuration geometry. If a request would push a propeller below 0 N or above 6 N, **yaw torque is sacrificed first** (it is the weakest axis, about five times more costly than roll or pitch) so that total thrust is always kept.

#### 3.6 Smooth paths and braking (interpolation)

- **A\*** finds a route over a grid of the campus that avoids building cells.
- A **cubic smoothing spline** (a B-spline fit) rounds the route's corners. It minimises `Σ(yi − s(ti))² + λ∫(s″)² dt`, trading closeness to the points against curvature.
- A **quintic Bézier** curve (degree 5) is used for each leg so that **position, velocity and acceleration all match at the joins**. Chained commands therefore flow into each other instead of stopping.
- **Braking** follows the kinematic law `d = v² / (2a)`. From 1.0 m/s at a deceleration limit of 1.0 m/s² this is **0.50 m**, and that is exactly what the simulation measured (0.92 s to rest).

### 4. What it can do

| Capability | How it is done | Measured result |
|---|---|---|
| Move in 6 directions, hover, land | Fuzzy matching of speech to 8 commands plus `land` | **24 / 24** scripted voice trials reached the target |
| Fly to a named building | Place matcher, A\*, spline and Bézier | **31** buildings addressable by voice |
| Chain commands | Bézier legs with matched velocity and acceleration | **5 / 5** chained legs, max error 0.022 m |
| Change its mind mid-flight | New command overrides the old one | Reversal in **0.42 s**, 2.04 m clear of the tower |
| Handle a tough list of maneuvers | Maneuver gauntlet | **67 / 67** passed |
| Survive disturbances | Yaw-priority mixer and gated integral | Within 0.15 m to **2 rad/s** gyro noise, **2 N** wind, **±20 %** wrong mass |
| Never fly into a building | Target clipping, map-edge limit | Also works when "forward" is repeated |

### 5. The world it flies in

The map is a **1:10 scale copy of the Amrita Coimbatore campus**: 86 buildings (31 named) with outlines and names from **OpenStreetMap**, over a real **satellite photo** (Esri World Imagery, about 0.6 m per pixel). Because the photo and the outlines come from different sources, each building is individually shifted onto its own roof by maximising the outline's overlap with photo edges (a Sobel filter). The visual layers have no mass or collisions, and a test proves they do not change the physics.

<table align="center">
  <tr>
    <td align="center"><img src="https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/drone_mujoco/docs/images/campus_map.jpg" alt="Campus map with building outlines" width="100%"><br><sub>Real outlines and names</sub></td>
    <td align="center"><img src="https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/drone_mujoco/docs/images/campus_3d.jpg" alt="3D campus scene on a satellite photo" width="100%"><br><sub>3D scene on the satellite ground</sub></td>
  </tr>
</table>

<p align="center">
  <img src="https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/drone_mujoco/docs/images/stop_braking.png" alt="Speed against time during the scripted flight, showing the drone braking to rest after each stop" width="85%"><br>
  <sub>Speed during the scripted flight: the drone brakes to rest after each "stop".</sub>
</p>

### 6. What is and is not claimed

**Shown, with measurements:** an equation learned from data that matches the true physics within 2–6 %, predicts 11 s of unseen flight, and sets the controller's gains; voice and place commands that reach their targets; robustness limits that were measured, not assumed.

**Not done (stated openly):**
- There is **no state estimator or Kalman filter**; sensors are ideal and noise tolerance was measured by injecting noise.
- There is **no controllability or reachability analysis**; the closest evidence is the eigenvalue study and the 11 s rollouts.
- **DMDc is valid near hover only.**
- **Building heights are estimated** (OpenStreetMap has no floor counts); footprints and names are real.
- Everything is **simulation at 1:10 scale**; there was no hardware flight.

### 7. Where to read more

| If you want… | Read |
|---|---|
| The full report, all results, and how to run it | [`drone_mujoco` README](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/drone_mujoco#readme) |
| Every idea explained from zero, in plain language | [`PROJECT_EXPLAINED.md`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/blob/drone_mujoco/PROJECT_EXPLAINED.md) *(available once pushed)* |
| The controller and sensors in depth | [`CONTROLLER_AND_IMU_EXPLAINED.md`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/blob/drone_mujoco/CONTROLLER_AND_IMU_EXPLAINED.md) |
| The video | [`campus_demo.mp4`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/blob/drone_mujoco/docs/recordings/campus_demo.mp4) |
| To fly it yourself | Double-click `RUN.bat` on the branch, press **1**, then speak or type |

**Main reference:** E. Fresk and G. Nikolakopoulos, *Full Quaternion Based Attitude Control for a Quadrotor*, ECC 2013. SINDy: Brunton, Proctor, Kutz, PNAS 2016. DMDc: Proctor, Brunton, Kutz, SIADS 2016.
