# The Whole Project, Explained From Zero

**Who this is for:** someone who has never seen this project and may not know control theory. Every idea is explained in plain words first, then the maths, then where it lives in the code. You can read it top to bottom, or jump to any part from the contents.

If you only have two minutes, read **Part 1** and **Part 14**.

---

## Contents

1. [The project in one minute](#part-1--the-project-in-one-minute)
2. [The big picture](#part-2--the-big-picture)
3. [Words you will meet](#part-3--words-you-will-meet)
4. [How a quadrotor flies (the physics)](#part-4--how-a-quadrotor-flies)
5. [Stage A: collecting data](#part-5--stage-a-collecting-data)
6. [Stage B: learning the equation (SINDy and DMDc)](#part-6--stage-b-learning-the-equation)
7. [Stage C: checking the learned model](#part-7--stage-c-checking-the-learned-model)
8. [Stage D: the controller](#part-8--stage-d-the-controller)
9. [Smooth paths and braking](#part-9--smooth-paths-and-braking)
10. [Stage E: voice, places and the demo](#part-10--stage-e-voice-places-and-the-demo)
11. [Safety](#part-11--safety)
12. [The campus world](#part-12--the-campus-world)
13. [Sensors, noise, wind and obstacles](#part-13--sensors-noise-wind-and-obstacles)
14. [One command, start to finish](#part-14--one-command-start-to-finish)
15. [Maths cheat sheet](#part-15--maths-cheat-sheet)
16. [How to run it and read the results](#part-16--how-to-run-it-and-read-the-results)
17. [Questions people ask](#part-17--questions-people-ask)
18. [What this project does not do](#part-18--what-this-project-does-not-do)
19. [Where everything is](#part-19--where-everything-is)

---

## Part 1 · The project in one minute

You speak (or type) a command such as **"forward"**, **"stop"** or **"go to the central library"**. A simulated drone, flying over a 1:10 scale copy of the Amrita Coimbatore campus, does it.

What makes this a *data-driven control* project, and not just a drone game:

1. We did **not** write down the drone's rotation equations from a textbook and trust them.
2. We made the simulated drone **wiggle** in many ways, recorded what happened, and asked a computer to **discover the equation** that explains the recording. The method is called **SINDy**.
3. We **tested** the discovered equation hard (on data it had never seen, over long predictions).
4. We then **calculated the controller's settings from that discovered equation**, instead of tuning them by trial and error.
5. We put a voice interface, a campus map and safety checks on top, and measured how well the whole thing works.

The simple summary: **data → equation → controller → voice-controlled flight.**

---

## Part 2 · The big picture

```
 YOU SAY:  "go to central library"
     │
     ▼
 ┌──────────────────────────────────────────────────────────────────┐
 │ 1. HEAR      Vosk turns sound into text (offline, no internet)    │
 │ 2. UNDERSTAND  a fuzzy matcher turns text into a command / place  │
 │ 3. PLAN      A* finds a route around buildings; a spline + Bezier │
 │              curve makes it smooth                                │
 │ 4. SAFETY    clip the target so it is never inside a building     │
 └──────────────────────────────┬───────────────────────────────────┘
                                ▼
 ┌──────────────────────────────────────────────────────────────────┐
 │ OUTER LOOP  "where should I be, how fast?"  (plain physics)       │
 │             position error  →  how much to accelerate             │
 │             →  how far to tilt, and how hard to push up           │
 └──────────────────────────────┬───────────────────────────────────┘
                                ▼
 ┌──────────────────────────────────────────────────────────────────┐
 │ INNER LOOP  "how do I tilt?"  (built on the LEARNED equation)     │
 │             attitude error  →  twisting force (torque)            │
 │ MIXER       thrust + torques  →  four motor thrusts               │
 └──────────────────────────────┬───────────────────────────────────┘
                                ▼
 ┌──────────────────────────────────────────────────────────────────┐
 │ MuJoCo physics, 500 times per second                              │
 │ the drone moves; sensors (IMU) measure it; numbers go back up     │
 └──────────────────────────────────────────────────────────────────┘
```

Two loops, one inside the other, like steering a car. The **outer loop** decides *where to go* ("turn the wheel left a bit"). The **inner loop** makes the machine *actually do it* (the wheels turn). The inner loop must react faster than the outer loop, or the two fight each other.

**The project was built and checked in five stages. Each had a pass/fail gate before the next began:**

| Stage | In plain words | Folder |
|---|---|---|
| **A** | Make the drone wiggle and record it | `01_simulation/` |
| **B** | Find the equation hidden in the recording | `02_identification/` |
| **C** | Try to break that equation; keep it only if it survives | `03_validation/` |
| **D** | Build the controller from the equation | `04_control/` |
| **E** | Add voice, map and the live demo | `05_voice_interface/` |

---

## Part 3 · Words you will meet

| Word | Plain meaning |
|---|---|
| **Quadrotor** | A drone with four propellers. |
| **Attitude** | Which way the drone is tilted (roll, pitch, yaw). Not where it is, but how it is oriented. |
| **Roll, pitch, yaw** | Roll: tipping left/right. Pitch: nose up/down. Yaw: spinning like a turntable. |
| **Angular velocity (ω, "omega")** | How fast it is rotating, in radians per second. |
| **Torque (τ, "tau")** | A twisting push. Torque causes rotation the way force causes movement. |
| **Thrust** | The upward push of a propeller, in newtons. |
| **State** | The list of numbers that fully describes the drone right now (here: tilt angles and rotation rates). |
| **Control input (u)** | What we are allowed to change: the four motor thrusts. |
| **Model** | An equation that predicts "given the state and what I do, what happens next?" |
| **System identification** | Finding the model from data instead of from theory. |
| **SINDy** | A method that finds a *short, readable* equation from data. |
| **DMDc** | A method that finds a *linear* (straight-line-style) model from data. |
| **Held-out data** | Recordings kept aside and never shown to the learner, used only to test it honestly. |
| **Rollout** | Letting a model predict many steps in a row on its own, without peeking at the truth. Errors pile up, so this is a hard test. |
| **R²** | A score from 0 to 1 (1 is perfect) of how much of the real behaviour the model explains. |
| **NRMSE** | Average prediction error, scaled so different quantities are comparable. Smaller is better. |
| **Eigenvalue / pole** | A number that says whether a system settles down, blows up, or hovers on the edge. |
| **Quaternion** | Four numbers that describe orientation without the problems of angles (explained in Part 8). |
| **PD controller** | "Push in proportion to the error (P) and push back against the speed of change (D)." |
| **Cascade** | Loops stacked inside each other: outer decides, inner executes. |
| **IMU** | The sensor package on a drone: gyroscope (rotation rate) and accelerometer, plus here position and attitude. |
| **ENU / FLU** | Direction conventions: world is East-North-Up; the drone's own body is Forward-Left-Up. |
| **Simulation step** | One tick of the physics clock. Here 0.002 s, so 500 per second. |

---

## Part 4 · How a quadrotor flies

You need only four ideas.

**1. Thrust holds it up.** Four propellers each push up. The drone weighs 1.0 kg, so it needs about 9.81 N in total to hover, which is **2.45 N per propeller** (9.81 ÷ 4).

**2. Unequal thrust makes it rotate.** The propellers sit at the corners of an X.
- More thrust on the left pair than the right pair → it **rolls** to the right.
- More on the back pair than the front pair → it **pitches** forward.
- Two opposite propellers spin clockwise and two counter-clockwise. Speeding up one pair and slowing the other makes a net twist that **yaws** the drone, because each spinning propeller pushes back on the body (its "reaction torque").

**3. Tilting makes it move sideways.** A drone cannot push sideways directly. To go forward it **tips forward**, so part of the upward thrust now points forward. That is the key to the whole cascade:

> To move, the outer loop asks for a **tilt**. The inner loop produces that tilt. So *position is controlled by attitude*.

**4. Yaw is the weak axis.** Roll and pitch use the long arm of the X (0.106 m). Yaw uses only the small propeller reaction (coefficient 0.02 m). So yaw torque is about **five times more expensive** in motor effort. This matters later, in the mixer.

The model in the code (`01_simulation/models/quad.xml`): X-configuration, 1.0 kg, 0.15 m arm length, drag that slows rotation, and four motors that can each push between 0 and 6 N.

---

## Part 5 · Stage A: collecting data

**Goal:** record the drone's behaviour in a form rich enough that a computer can discover the physics from it.

**How:** feed the four motors special signals, with **no controller active**, and record everything.
- **PRBS** (pseudo-random binary sequence): thrust flips randomly between two levels, like a noisy on/off switch. It shakes the system at many speeds at once.
- **Chirp**: a smooth wave that starts slow and gets faster, like a siren sweeping up. It tests one speed range after another.
- These are applied on **each axis** (roll only, pitch only, yaw only) and on **combinations**.

**Why open loop (no controller)?** If a controller were active, the data would mostly show the controller's habits, not the drone's own physics.

**Why wiggle so much?** A model can only learn what it has seen. A drone that only hovered would teach us nothing about how it rotates.

**Passing the gate:** plots must show rich oscillation, not a flat line. Trials are split into **training** and **held-out** *by whole trial*. We never cut one recording into two pieces, because the two pieces would look too alike and the test would be too easy.

**Maths touch-point:** random seeds are derived deterministically per trial (not by Python's `hash()`, which changes between runs). A test (`test_reproducibility.py`) proves two separate processes produce identical data.

---

## Part 6 · Stage B: learning the equation

### 6.1 What we are trying to find

For rotation, physics says roughly:

```
how fast the rotation speed changes  =  (motor torques)
                                       − (gyroscopic coupling between axes)
                                       − (air drag on rotation)
```

We want the computer to find this from recordings. We supply the **recordings** and a **menu of possible ingredients**. It must pick the ingredients that matter and the right amount of each.

### 6.2 SINDy in layman terms

**SINDy** stands for *Sparse Identification of Nonlinear Dynamics*.

Think of a chef tasting a soup and trying to recover the recipe. We give them a **shelf of 26 possible ingredients**. SINDy tries to explain the soup using as **few** ingredients as possible. "Sparse" means *few*.

The 26 ingredients (the **library**, in `02_identification/sindy_fit.py`):

| Group | Ingredients | Why it is on the menu |
|---|---|---|
| a constant | `1` | A bias. A good fit should switch it off. |
| tilt angles | `θx, θy, θz` | Needed for how angle changes. |
| rotation rates | `ωx, ωy, ωz` | Drag: the faster it spins, the more it is slowed. |
| motor thrusts | `u1 … u4` | Control: more thrust, more torque. |
| rate × rate | `ωx·ωx, ωx·ωy, … ` (6 terms) | The gyroscopic cross-coupling between axes. |
| angle × rate | `θi·ωj` (9 terms) | A small correction in how angles evolve. |

Each ingredient is something real rigid-body physics could produce. We **left out** terms physics gives no reason to expect (θ·θ, u·u, θ·u, ω·u). A blind menu would have 66 items; ours has 26. The fewer false suspects, the more the method discovers physics rather than fits noise.

**There are 6 equations to find** (how each of the 3 angles and each of the 3 rotation rates changes). With 26 ingredients each, that is a **6 × 26 grid = 156 coefficients**. **SINDy keeps 44 of them and sets the other 112 to exactly zero.**

### 6.3 The maths, step by step

We record many moments. For each moment *k* we know the state, the thrusts, and the rate of change of the state. Stack all moments into a big table:

```
   Θ  =  [ the 26 ingredients, evaluated at every recorded moment ]     (many rows × 26 columns)
   Ẋ  =  [ the measured rate of change of the state ]                    (many rows × 6 columns)
```

We want coefficients `Ξ` (26 × 6) so that

```
   Ẋ  ≈  Θ · Ξ
```

**Step 1: ordinary least squares.** Pick `Ξ` that makes the squared error `‖Ẋ − ΘΞ‖²` as small as possible. This uses every ingredient a little, so the answer is messy.

**Step 2: STLSQ (sequentially thresholded least squares).** The "clean-up" loop:
1. Fit least squares.
2. Any coefficient smaller than a threshold λ is declared "not important" and **set to zero**.
3. Refit using **only the surviving ingredients**.
4. Repeat until nothing changes.

The result is a short equation you can read.

**Step 3: smooth before differentiating.** We need `Ẋ` (rates of change) but we only measure the state. Differentiating raw data multiplies noise. So we first smooth with a **Savitzky-Golay filter** (it fits a small curve to each short window of data) and differentiate that.

**Step 4: do not fit raw quaternions.** Attitude is handled in **local coordinates** (small angles around a reference) and re-referenced at segment boundaries. We never form "this step to next step" pairs *across* a boundary, because the angle jumps by about 2.6 rad there against about 0.002 rad of real motion. One fake pair like that outweighed a million real ones, and made the model look broken when it was fine.

**Why this is "data-driven control":** the equation was **found**, not assumed.

### 6.4 Did it find real physics? Yes, and we can prove it

The simulator's true physics is known to us. We compared the learned coefficients with the truth (`tests/test_identification_physics.py`):

| Physical thing | Match to the simulator's truth |
|---|---|
| How strongly each motor torques each axis | within 2–6% |
| Rotation drag | within 2–6% |
| Gyroscopic cross-coupling | within 2–6% |
| Angle = integral of rotation rate | within 2–6% |

A curve-fit can score well and still be nonsense. A fit whose **coefficients equal the real physics** is different. That is the evidence.

### 6.5 DMDc in layman terms

**DMDc** (*Dynamic Mode Decomposition with control*) fits the simplest possible model:

```
   next state  =  A · (state now)  +  B · (control now)
```

`A` and `B` are just grids of numbers. It is a **straight-line-style (linear)** model, with no products or curves.

- **Strength:** quick, simple, and gives an exact stability test.
- **Weakness:** real rotation has curved terms (the cross-coupling). A straight-line model only works close to the point where it was fitted, which here is **hovering**.

So DMDc is claimed valid **near hover only**. Measured: DMDc is good near hover (R² 0.9926), and its one-step error rises to an NRMSE of about **0.70** outside it (1.0 is a model that does nothing).

**SINDy and DMDc are not rivals for the same job.** They answer the same question at different levels of ambition. SINDy covers the full flying range. DMDc is the simpler comparison, and also acts as a check on SINDy.

---

## Part 7 · Stage C: checking the learned model

A model that fits its training data proves nothing. Stage C tries to break it. **The rule: no later stage uses a model that has not passed an earlier gate.**

### 7.1 The ladder of tests (easy to hard)

| Test | What it asks | SINDy result |
|---|---|---|
| **One-step prediction** | "From the true state now, predict the next 2 ms." | NRMSE **0.0786**, R² **0.9946** on held-out trials |
| **Short rollout** | "Predict many steps in a row, no peeking." | passes |
| **Long rollout (11 s)** | "Predict 11 seconds by yourself." | **0 of 7 diverged**, mean final error 0.095 (limit 0.5) |

### 7.2 Metrics in plain words

- **R² = 0.9946** means the model explains about **99.5%** of the real behaviour on data it never saw.
- **NRMSE = 0.0786** means the typical error is about **8%** of how much the state actually changes in one step. A model that does nothing scores exactly **1.0** (100% error), so 0.08 is far better than doing nothing.
  - *Lesson learned:* if you scale error by the size of the state, a do-nothing model looks great at 2 ms steps. Scaling by the **change** removes that trap.
- **"Diverged"** means the prediction ran away from reality. None did.

### 7.3 Eigenvalues: will it settle or explode?

Every linear(ised) system has numbers called **eigenvalues** (poles). They decide behaviour:

| Where the eigenvalue sits | What the system does |
|---|---|
| Left of zero (continuous), or inside the unit circle (discrete) | Settles down. Stable. |
| Right of zero, or outside the unit circle | Blows up. Unstable. |
| Exactly on the boundary | Neither settles nor explodes. Marginal. |

**3 of 6 poles are marginal, and that is correct.** A free-floating rigid body has no spring pulling it back to a particular angle: if it tilts, it stays tilted (angle just accumulates rotation rate). So a marginal pole is the physically right answer, not a bug. The other three are genuinely stable.

*Lesson learned:* an early version tested "strictly stable" and wrongly flagged SINDy as unstable, briefly flipping the choice to DMDc for the wrong reason. A small tolerance fixed it.

### 7.4 Ablations: turning knobs to see what matters

An **ablation** removes or worsens one thing at a time.

| Ablation | What we did | What we found |
|---|---|---|
| **Sparsity** | Raise the "drop small terms" threshold | R² stays high above 0.2 and collapses at 1.0 (to 0.588). So 44 terms is a sensible, not arbitrary, choice. |
| **Noise** | Add random noise to the rotation measurements | Fine to about 1 rad/s, knee at 2.0, breakdown at 4.0. The model is robust until noise is far larger than the signal. |

*Lesson learned:* the first noise range was too gentle (0 to 0.1 rad/s) and showed a flat line. A test that cannot fail proves nothing, so the range was widened until the real breaking point appeared.

### 7.5 Head to head, fairly

In **DMDc's own best conditions (near hover)**: SINDy 0.0789 vs DMDc 0.0825. SINDy matches DMDc there, and also works everywhere else. **SINDy is selected** and frozen to `data/processed/sindy_fitted_model.npz`. The flight controller loads exactly that file, so what flies is provably the model that passed.

---

## Part 8 · Stage D: the controller

### 8.1 Why two loops

We want to control **position**, but we can only push with **four motors**. The chain of cause and effect is:

```
 motors → torques → rotation → tilt → sideways push → velocity → position
```

Each link is slower than the one before. So we control the **fast links with a fast loop** and the **slow links with a slow loop**:

- **Inner loop** (fast): "make my tilt equal the tilt I was asked for."
- **Outer loop** (slow): "to reach that position, what tilt and thrust do I need?"

### 8.2 The inner loop (built on the learned equation)

#### Why quaternions?

Describing tilt with three angles (roll, pitch, yaw) breaks down in some poses: **gimbal lock**, where two axes line up and one degree of freedom vanishes. Maths on angles also has awkward jumps (359° → 0°).

A **quaternion** is four numbers `[w, x, y, z]` with one rule (their squares add to 1). It describes any orientation **with no singularities and no jumps**. Layman's picture: *"rotate by some angle around some axis."* The axis is in `x, y, z` and the angle is hidden in `w`.

The **error quaternion** `qₑ = q_wanted ⊗ q_actual*` says "the single rotation that would take me from where I am to where I want to be." Its **vector part** `qₑ,vec` points along the axis I should rotate about, and its size is **about half the angle error** for small angles.

#### The control law

```
   τ  =  − Kq · qₑ,vec   −   Kω · ω
```

Read it as two instructions:
1. **Rotate toward the target**, harder the further off you are (`Kq` term). That is the spring.
2. **Slow down your spinning** so you do not overshoot (`Kω` term). That is the damper (the shock absorber).

This is called a **P² law** (*Fresk & Nikolakopoulos, 2013*): a proportional push on attitude and a proportional push on rotation rate, with no derivative of noisy data and no integral.

#### Where the gains come from (the "data-driven" part)

Normally you would **tune** `Kq` and `Kω` by trial and error. Here they are **calculated** from the SINDy model.

From the learned equation, for each axis around hover we read two numbers:
- `b`: how strongly the motors torque that axis (control effectiveness, which is 1 ÷ inertia).
- `a`: the drag term. It is a **negative** number (−drag ÷ inertia), because drag slows rotation. The drag already supplies some damping for free, so the controller only adds the rest.

Close to hover each axis behaves like a **mass on a spring with a damper**. Classical second-order design says:

```
   ωn  =  4 / (ζ · Ts)               natural frequency, from the speed we want
   Kq  =  2 · ωn² / b                spring strength
   Kω  =  (2 · ζ · ωn + a) / b       damping strength (a < 0, so this subtracts what drag already gives)
```

We choose only **two numbers by hand**:
- **Ts = 0.5 s** (how quickly the tilt should settle)
- **ζ = 0.7** (damping ratio: 1 is no overshoot but slow, 0.7 is a small, quick overshoot)

Everything else comes from the learned `a` and `b`. **If the model changes, the gains recompute automatically.** That is what "controller built on the identified model" means.

Check (6 setpoint tests, SINDy model vs the real MuJoCo plant): the tilt settles in **0.56–0.86 s**, and the SINDy-driven and MuJoCo-driven responses agree to about **0.3%**.

### 8.3 The mixer: from "what I want" to "four motors"

The controller outputs four things: total **thrust** `T` and three **torques** `τx, τy, τz`. The motors need four **thrusts** `f1…f4`.

Because the geometry is known, this is one linear equation:

```
   [ T  ]       [ 1      1      1      1   ]   [ f1 ]
   [ τx ]  =    [ ARM   −ARM   −ARM   ARM  ] · [ f2 ]
   [ τy ]       [ −ARM   ARM   −ARM   ARM  ]   [ f3 ]
   [ τz ]       [ KM     KM    −KM    −KM  ]   [ f4 ]
```

We **invert** the matrix to get the four motor thrusts from the four wanted numbers. (`ARM = 0.106 m`, `KM = 0.02 m`.)

**Saturation problem:** a motor cannot push less than 0 N or more than 6 N. If the request would need −1 N from a motor, simply clipping it ruins the **total** thrust, and the drone can sink.

**Yaw-priority fix:** if something would clip, **give up yaw torque first** (yaw is the weakest and least important axis), then roll/pitch, and **always keep total thrust**. This is `inverse_mix_yaw_priority`. When nothing clips, it behaves exactly like the plain version.

### 8.4 The outer loop (ordinary physics)

The outer loop does **not** use any learned model, on purpose. Moving in a straight line under a thrust is Newton's second law, `force = mass × acceleration`. There is nothing for a data-driven method to discover. Learning it would add complexity and no insight.

```
   a_cmd  =  Kp · (position error)  +  Kd · (velocity error)  +  Ki · ∫(position error)
```

- **Kp** pulls toward the target (spring).
- **Kd** resists speed (damper). With settling time 2 s and damping 1: `ωn = 2`, `Kp = 4`, `Kd = 4`.
- **Ki**, the **integral**, slowly fixes any steady leftover error from a constant push (wind, or a wrong mass).

**Why the integral is cautious.** An integral that is always on adds lag during a move and caused a **0.196 m overshoot** in early tests. So it only works when **both the planned path and the drone are nearly at rest** (speed under 0.15 m/s, a value picked by testing 0.08 to 1.0 m/s). This is called **anti-windup**.

**Turning acceleration into tilt:** the thrust vector must be `m · (a_cmd + g)`, where `g` is gravity (9.81 m/s² up). Its **size** is the total thrust. Its **direction** is the tilt (a quaternion) handed to the inner loop. The outer loop picks the smallest tilt that gives the wanted acceleration.

### 8.5 Why the two loops do not fight

The inner loop is designed to be several times **faster** than the outer one (2.86 times, measured in the project's bandwidth check). From the outer loop's point of view, the inner loop has already finished its job. That lets us design each independently. This is the **cascade stability** idea.

**Honest note:** we first designed the outer loop as a perfect no-overshoot system. The real cascade, with inner-loop lag and drag, shows about 0.07 m of ringing. We report what was measured, not what the design claimed.

### 8.6 Results of the cascade

All six voice waypoints settle in about **1.87 s** with final error **0.0006 m**. SINDy-driven and MuJoCo-driven cascades match to four decimal places. The flight with the curved demo path takes about 2.95 s to settle (longer path).

---

## Part 9 · Smooth paths and braking

### 9.1 Why not fly in straight lines with sharp corners

A drone cannot turn a corner instantly. A sharp corner needs infinite acceleration. A smooth curve respects what the motors can do.

### 9.2 Finding the route: A\*

For "go to the central library" the drone must avoid buildings. **A\*** ("A-star") is a classic route-finder, the same idea as a maps app.
- The map is a grid. Cells inside buildings are blocked.
- A\* explores outward from the start, always favouring cells that are **both close to the start and look close to the goal**: `score = distance travelled + estimated distance remaining`.
- It returns the **shortest route** that avoids every blocked cell.

The raw route is a jagged chain of waypoints.

### 9.3 Making it smooth: spline and Bezier curves

**Interpolation** means "draw a smooth curve through or near a set of points."

- **Cubic smoothing spline** (a B-spline fit): a flexible ruler bent through the waypoints. It rounds every corner. It is "cubic" because each piece is a cubic polynomial, and pieces join with matching slope and curvature.
- **Quintic Bezier** (a fifth-degree polynomial curve): used for each leg so that **position, velocity and acceleration all match at the join**. That is why chained commands do not stop dead between legs. The drone flows from one into the next.

Every curve is **checked before it is flown**: does it clip a building, and does it stay within the speed and acceleration limits? If not, it is slowed down.

### 9.4 Braking: how "stop" works

"Stop" does **not** turn back to a starting point. It brakes to rest **ahead** of where it is, at a gentle, fixed deceleration of **1.0 m/s²**.

The physics is one line:

```
   stopping distance  =  v² / (2 · a)
                      =  (1.0)² / (2 × 1.0)  =  0.5 m
```

**Measured in the video:** from 1.0 m/s, the drone is at rest in **0.92 s** over **0.50 m**, with altitude held within 2 cm. The measurement matches the formula, which is a good sign that the whole chain behaves like real physics.

---

## Part 10 · Stage E: voice, places and the demo

### 10.1 Hearing: Vosk

**Vosk** is a speech recogniser that runs **offline** on your laptop (no internet, nothing sent anywhere). It turns microphone sound into text. If the Vosk model folder is missing, the demo falls back to typed commands.

### 10.2 Understanding: the fuzzy classifier

Speech recognisers make mistakes ("left" heard as "lift"). So we do not demand an exact word.

1. Clean the text: drop filler words ("go", "please", "a bit").
2. Compare each word with every known command and its **synonyms** (`halt`→stop, `climb`→up, `descend`→down …), scoring **similarity** with Python's `difflib` (a number from 0 to 1 based on shared character sequences).
3. Accept the **best-scoring** match above a cutoff.

*Real bug found and fixed:* an early version returned the **first** word that cleared the cutoff. In "five meters **to** left", the filler word "to" resembled "stop" (score 0.67) and was picked before the perfect "left" (1.0). The fix is to score **every** word and keep the **best**.

**The commands:** `forward`, `back`, `left`, `right`, `up`, `down`, `hover`, `stop`, plus `land` and `go to <building>`.

**Honesty about the screen:** the transcript is shown **even when it is not understood** (`NOT RECOGNIZED`). Silence never hides a mishearing.

### 10.3 Places: "go to <building>"

A **place matcher** maps what you said to one of **31 named campus buildings**. It uses the same fuzzy matching, so "central library" and "the library" both work. The route is then planned with A\* and smoothed as in Part 9.

### 10.4 Moving: continuous motion

`forward`, `left` and the others keep moving until one of three things stops them: a building ahead, the map edge, or your next command. A new command **overrides** the old one mid-flight (called *pre-emption*). We tested this: say "back" 0.7 s into a "forward", and the drone reverses in **0.42 s**, never getting closer than 2.04 m to the tower.

### 10.5 Landing

`land` finds the surface below (ground or a rooftop), descends onto it, and **turns the motors off** at touchdown.

### 10.6 The live demo window

The window shows the 3D campus, a small map, strip-charts of tilt error / position error / control effort, a **LISTENING** line (what it is hearing right now) and a **HEARD LOG** (what it heard and what it did). There are also **clickable buttons** for every command, so a bad microphone never stops a demo.

---

## Part 11 · Safety

| Safety measure | What it prevents |
|---|---|
| **Building clip** (`clamp_target_to_safe_zone`) | A target inside a building. Works even when you say "forward" twice and the second would land the drone in a wall, because it checks the whole straight path with a small margin. |
| **Geofence** | Flying off the map. |
| **Smallest-tilt rule** | Extreme tilts that would cost thrust. |
| **Yaw-priority mixer** | Losing altitude when motors saturate. |
| **Curve checking** | Flying a path that exceeds speed or acceleration limits. |
| **Landing check** | Landing on empty air, or leaving motors spinning on the ground. |

---

## Part 12 · The campus world

The map is a **1:10 scale copy** of the real Amrita Coimbatore campus: **86 buildings**, **31 named** and addressable by voice.

### Where each piece comes from

| Piece | Source |
|---|---|
| **Building outlines and names** | OpenStreetMap (volunteer-drawn map data) |
| **Building heights** | **Estimated** from building type. OpenStreetMap has no floor counts. |
| **Ground picture** | A real satellite photo (Esri World Imagery, about 0.6 m per pixel) painted onto the floor |

### The alignment problem (and fix)

The satellite photo and the OpenStreetMap outlines were made separately, so the outlines **do not line up with the roofs**. The error is not one constant shift: it varies by a few metres from building to building.

The fix (`01_simulation/models/align_buildings.py`):
1. For each building, slide its outline around by up to ±5 m in small steps.
2. At each position, measure how well the outline lies on **edges in the photo** (the edge of a roof is a sharp brightness change; we find it with a **Sobel filter**, which highlights sharp changes).
3. Keep the position with the strongest edges, **but only if the evidence is clear** (strong enough, clearly better than the original, and not at the edge of the search window).

Result: 51 of 80 measurable buildings were moved (median move 2 m). The other 29 stayed where OpenStreetMap put them because the evidence was unclear. The average edge match rose from **38.8 to 62.4**.

**Known leftover:** the photo was taken at a slight angle, so tall buildings show a pale rim around their base. No shift can remove that.

### Is the scenery part of the physics?

**No.** The buildings and ground picture are visual only (no mass, joints or collisions). A test (`test_scene_physics_inert.py`) proves that adding them does not change the drone's motion at all. The drone is kept out of buildings by the **safety clip**, not by physical collision.

---

## Part 13 · Sensors, noise, wind and obstacles

### 13.1 The IMU

The controller does not read the simulator's hidden state directly. It reads **named sensors** (position, orientation, a body-frame velocity sensor, and a gyroscope), the way a real flight computer would. The body-frame velocity is rotated into the world frame before use.

*Real bug found and fixed:* MuJoCo computes sensor values from the state **before** each physics step, so reading straight after the step was 2 ms stale. One extra `mj_forward()` call fixed it.

**Honest limit:** these are **ideal** sensors (no noise by default) and there is **no Kalman filter**. A real drone would need one.

### 13.2 Stress tests (robustness)

Each disturbance was applied alone to a `forward` flight:

| Disturbance | Original controller fails at | With the two fixes, still accurate up to |
|---|---|---|
| Gyro noise | 0.05 rad/s | 2.0 rad/s |
| Constant wind | 1 N | 2 N (fails at 3 N) |
| Wrong payload mass | ±10% | ±20% (fails at +50%) |

**The two fixes**, both found by measurement:
1. **Yaw-priority mixer.** Noise on the yaw gyro made the controller ask for rapid yaw changes. Yaw costs the most motor headroom, so motors hit their limits on about 70% of steps and thrust was lost. Shedding yaw first fixed it.
2. **Anti-windup integral.** A PD controller leaves a steady offset under constant force (a 10% mass error gives 0.1·g/Kp = 0.245 m exactly). A gated integral removes it.

### 13.3 Autonomous patrol among moving obstacles (extra, not in the menu)

The code can also fly a patrol by itself among **four moving obstacles** plus buildings. A planner supplies only the reference path. **The same cascade still does all the controlling.** Result: 5/5 collision-free on development seeds (vs 0/5 for straight-line flight), and **29 of 30** on 30 never-seen seeds. The one failure is a double pinch that a planner with no lookahead cannot foresee.

---

## Part 14 · One command, start to finish

Follow the word **"forward"** all the way through. Times are approximate.

| Step | What happens | Where |
|---|---|---|
| 1 | You say "forward". The microphone gives sound to **Vosk**, which returns the text `forward`. | `voice_input.py` |
| 2 | The **classifier** scores the word against all commands. "forward" scores 1.0. Command = `forward`. | `commands.py` |
| 3 | The command becomes a **target**: 1.5 m ahead of where the drone is *now*. | `commands.py` |
| 4 | The **safety clip** checks the line to the target against all buildings and the map edge. If it would hit one, the target is pulled back. | `city.py` |
| 5 | A smooth **quintic Bezier** trajectory is built from now to the target, within speed and acceleration limits. **About 36 ms in total** from classify to dispatch. | `trajectory.py` |
| 6 | Every **2 ms**, the **outer loop** compares the drone's measured position and velocity with the trajectory and computes a wanted acceleration. | `position_controller.py` |
| 7 | That acceleration becomes a **thrust vector**: its size is total thrust, its direction is the wanted **tilt** (a quaternion). | `position_controller.py` |
| 8 | The **inner loop** compares the measured tilt with the wanted tilt, forms the error quaternion, and outputs **torques** using the gains calculated from the SINDy model. | `controller.py` |
| 9 | The **mixer** turns (thrust, torques) into **four motor thrusts**, shedding yaw first if any would clip. | `mixer.py` |
| 10 | **MuJoCo** applies the four thrusts for 2 ms and moves the drone. | MuJoCo |
| 11 | The **IMU sensors** measure the new state and send it back to step 6. | `flight.py` |
| 12 | After about 1.9–3 s the drone arrives. The **HUD** and the **log** record everything. | `demo.py` |

Steps 6–11 repeat **500 times every second** until the drone stops.

---

## Part 15 · Maths cheat sheet

Every mathematical idea in the project, in one place.

| Idea | Plain meaning | Where it is used |
|---|---|---|
| **Newton's 2nd law** (`F = ma`) | Push harder, accelerate faster. | Outer loop; the whole simulator |
| **Torque and rotation** | A twisting push makes rotation speed change. | Inner loop; the learned equation |
| **Gravity compensation** | Thrust must cancel 9.81 m/s² before it can accelerate. | Outer loop thrust vector |
| **Least squares** | Pick numbers that make the total squared error smallest. | SINDy, DMDc |
| **Sparsity (STLSQ)** | Explain the data with as few ingredients as possible. | SINDy |
| **Savitzky-Golay smoothing** | Fit a tiny curve to each short window to remove noise before differentiating. | Before SINDy |
| **Numerical derivative** | Rate of change estimated from samples. | Building the SINDy target |
| **Linear algebra, pseudo-inverse** | Solve for unknowns in an over-determined system. | DMDc, mixer |
| **R², NRMSE** | How much of reality the model explains, and by how much it misses. | Stage B and C |
| **Eigenvalues / poles** | Whether a system settles, explodes or sits on the edge. | Stage C; controller design |
| **Quaternions** | Orientation as four numbers, free of gimbal lock. | Inner loop |
| **Quaternion multiplication** | Combining or differencing two rotations. | Error quaternion |
| **Second-order system** (`ωn`, `ζ`, `Ts`) | A spring-damper: how fast, and how bouncy. | Both loops' gains |
| **PD / PID control** | Push by error, resist by speed, remove leftover offset. | Outer loop |
| **Anti-windup** | Stop an integral from piling up during a move. | Outer loop |
| **Cascade control** | A fast inner loop inside a slow outer loop. | Whole controller |
| **Matrix inverse** | Undo a known linear mixing. | Mixer |
| **Saturation handling** | What to do when a motor cannot do what is asked. | Yaw-priority mixer |
| **A\* search** | Cheapest route, guided by an estimate of the remaining distance. | Route planner |
| **Cubic spline (B-spline fit)** | A smooth curve bent through points. | Path smoothing |
| **Bezier curves** (quintic) | A smooth curve controlled by anchor points, matching speed and acceleration. | Trajectories |
| **Kinematics (`v²/2a`)** | Distance needed to brake. | "stop" |
| **String similarity** (`difflib`) | How alike two words are. | Voice classifier |
| **Sobel edge detection** | Finding sharp brightness changes in a picture. | Map alignment |
| **Ray vs box intersection** | Where a straight path first meets a building. | Safety clip |
| **Random seeds** | Making "random" data repeat exactly. | Reproducibility |

---

## Part 16 · How to run it and read the results

### Run

Double-click **`RUN.bat`** in the project folder and pick a number:

| Choice | What it does |
|---|---|
| **1** | Flies the drone live on the campus. Speak or type. A commands guide window stays open. |
| **2** | After flying: draws 2D and 3D graphs of the flight you just did. |
| **3** | Prints the drone's mass, inertia, gains and learned model. |
| **4** | Runs the 167 automated tests. |
| **5** | A browser version on your own computer only (localhost). |
| **6** | Rebuilds every result and plot (several minutes). |

**What to say:** `forward`, `back`, `left`, `right`, `up`, `down`, `stop`, `hover`, `land`, or `go to central library`.

### What to look at in the demo window

| On screen | Meaning |
|---|---|
| 3D view | The drone over the campus. |
| Small map | Where you are on the campus. |
| **LISTENING** line | What the microphone is hearing right now. |
| **HEARD LOG** | Every phrase heard and the result (even failures). |
| Strip charts | Tilt error, position error, motor effort, live. |
| Buttons | Click any command instead of speaking. |

### Where the numbers live

| Number | File |
|---|---|
| Identification and validation | `data/processed/validation_plots/`, copied to `results/plots_2d/` |
| Control | `data/processed/control_plots/` |
| Voice trials, gauntlet | `results/RESULTS.md` |
| Braking | `docs/recordings/stop_metrics.json` |
| Demo video | `docs/recordings/campus_demo.mp4` |

---

## Part 17 · Questions people ask

**Why is only the inner loop learned from data?**
Because that is where the interesting physics is. Moving in a line is plain Newton's law with nothing to discover. Rotation has drag, cross-coupling and orientation maths, which is worth learning.

**Is the controller really "built on" the learned model, or is that just a claim?**
Really. Its gains are computed by `gains_from_identified_model()` from the SINDy model's drag and control-effectiveness numbers. Change the model and the gains change.

**How do you know SINDy found real physics and did not just fit a curve?**
Its coefficients match the simulator's true values within 2–6%, and a test pins that. It also predicted 11 seconds of held-out flight without diverging.

**Why 44 of 156?**
There are 6 equations with 26 candidate terms each (156 numbers). SINDy keeps the 44 that matter and sets 112 to exactly zero. We swept the threshold: the fit holds above 0.2 and collapses at 1.0.

**Where do DMDc and SINDy fit together?**
Both learn from the same data. DMDc is the simple linear model, valid near hover only. SINDy is the nonlinear model valid across the full range. DMDc is the fair baseline SINDy is compared against.

**Why quaternions, not roll/pitch/yaw?**
No gimbal lock and no 359°→0° jumps, so the maths stays smooth in every pose.

**Why does a drone tilt to move?**
It can only push along its own up direction. Tilting points part of that push sideways.

**Is there a Kalman filter or state estimator?**
No. We use ideal sensors, and measure noise tolerance by injecting noise. A Kalman filter is listed as future work.

**Are there controllability or reachability results?**
Not as separate analyses. The closest evidence is the eigenvalue study and the 11-second rollouts. Both are listed as future work.

**Is any of this tested on a real drone?**
No. Everything is simulation, at 1:10 scale.

**What happens if the microphone mishears?**
The transcript is always shown. If it is not recognised, nothing flies, and the screen says so. Buttons and typing are always available.

**Why can you trust the video's braking number?**
Because it is measured by the recording script from the simulator's own state, and it matches the physics formula `v²/2a`.

---

## Part 18 · What this project does not do

Being honest is part of the result.

- **No state estimator.** Sensors are ideal.
- **DMDc is hover-only.** It is never claimed beyond that.
- **Building heights are estimated.** Footprints and names are real.
- **The satellite photo leans.** Tall buildings keep a pale rim. 29 of 80 buildings were not moved because the evidence was unclear.
- **Disturbance limits.** 3 N wind and +50% payload still fail.
- **Autonomous patrol assumes perfect obstacle knowledge.** One of 30 unseen tests failed.
- **Speech recognition makes errors.** The small offline model is not perfect.
- **Simulation only.** No hardware flight.

---

## Part 19 · Where everything is

```
.
├── RUN.bat                      the one launcher
├── README.md                    the front page with results
├── PROJECT_EXPLAINED.md         this file
├── 01_simulation/               Stage A: the drone model, data collection, campus scenes
│   └── models/                  quad.xml, make_campus.py, align_buildings.py, city.py
├── 02_identification/           Stage B: sindy_fit.py, dmdc_fit.py, preprocessing.py
├── 03_validation/               Stage C: rollouts, eigenvalues, ablations, model selection
├── 04_control/                  Stage D: controller.py (inner), position_controller.py (outer),
│                                mixer.py, trajectory.py, route_planner.py, avoidance.py
├── 05_voice_interface/          Stage E: commands.py, voice_input.py, flight.py, demo.py,
│                                run_trials.py, record_demo_video.py
├── tests/                       167 automated tests
├── results/                     plots, data, RESULTS.md
├── docs/                        design documents, images, recordings
├── web/                         the browser simulator
├── CONTROLLER_AND_IMU_EXPLAINED.md   a deeper dive on the controller and sensors
├── UNDERSTAND_MY_PROJECT.md          a guided tour of the files
└── CLAUDE.md                    the decision history
```

**Reading order for a newcomer:** this file → `README.md` → `CONTROLLER_AND_IMU_EXPLAINED.md` → `docs/TDD.md` → the code in stage order.

**Team:** Mamidala Som Praneeth Babu (CB.SC.U4AIE24029), C. Karthik (CB.SC.U4AIE24107), Ch. Virinchi (CB.SC.U4AIE24158), K. Jatin Chandra Gupta (CB.SC.U4AIE24162). Amrita Vishwa Vidyapeetham, Coimbatore.

**Main reference:** E. Fresk and G. Nikolakopoulos, *Full Quaternion Based Attitude Control for a Quadrotor*, ECC 2013.
