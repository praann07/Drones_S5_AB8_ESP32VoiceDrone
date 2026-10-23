# Voice-Controlled PX4 Drone with B-Spline Obstacle Avoidance and SINDy Data Logging

Say **"take off"**, **"point library"**, **"fly to library"** and a simulated quadcopter flies a smooth, building-avoiding route there, while QGroundControl shows the route and a logger records flight data for system identification (SINDy).

Everything runs in simulation: **PX4 SITL + Gazebo (x500 quadcopter)** on the ETH Zurich Irchel map, **QGroundControl** as the ground station, and **Python + MAVSDK** for our own code. PX4 does all low-level flight control. Our work is the layer above it: voice commands, path planning, data logging and model fitting.

> This README describes what we built, how to run it, what we tried and dropped, and what is still broken. The "Known issues" and "Left out" sections are there on purpose so nobody repeats our mistakes.

---

## Contents

1. [What the project does](#1-what-the-project-does)
2. [How it fits together](#2-how-it-fits-together)
3. [Repository layout](#3-repository-layout)
4. [How to run it](#4-how-to-run-it)
5. [Voice commands](#5-voice-commands)
6. [Path planning: A* and B-spline](#6-path-planning-a-and-b-spline)
7. [Data logging and SINDy](#7-data-logging-and-sindy)
8. [Using QGroundControl with it](#8-using-qgroundcontrol-with-it)
9. [How the project evolved](#9-how-the-project-evolved)
10. [Known issues and problems you may hit](#10-known-issues-and-problems-you-may-hit)
11. [Left out, abandoned or not finished](#11-left-out-abandoned-or-not-finished)
12. [What you can do with it](#12-what-you-can-do-with-it)
13. [Credits and references](#13-credits-and-references)

---

## 1. What the project does

| Part | What it does |
| --- | --- |
| **Simulation** | PX4 SITL with a Gazebo x500 quadcopter, started headless. |
| **Voice control** | Offline speech recognition (Vosk) turns speech into text, and the text is matched to flight commands. "start" and "stop" are wake and pause words. |
| **Saved points** | "point `<name>`" stores the drone's current position under a name. |
| **Obstacle-avoiding flight** | "fly to `<name>`" plans a route around the buildings in `obstacles.json` using A\* and a cubic B-spline, uploads it to PX4 as a mission, and starts it. |
| **Ground station** | QGroundControl (QGC) shows the drone, the mission route and (optionally) the building outlines. |
| **Data logging** | `data_logger.py` records attitude, velocity, acceleration and angular rate at about 48 Hz to `flight_data.csv`. |
| **System identification** | `fit_sindy.py` / `fit_sindy_clean.py` fit SINDy models to that data. |
| **Operation log** | Every recognised command is appended to `operation_log.csv`. |

---

## 2. How it fits together

```
                 ┌───────────────────────────────┐
                 │  PX4 SITL  +  Gazebo (x500)   │   flight controller + physics
                 └──────┬─────────────┬──────────┘
        MAVLink (UDP)   │             │
   ┌────────────────────┼─────────────┼───────────────────────┐
   │ 14550              │ 14540       │ 14530 (PX4 -o)        │ 14535 (PX4 -u)
   ▼                    ▼             ▼                       ▼
 QGroundControl   data_logger.py   voice_controller_v4   live_hub.py / Foxglove
 (map, plan,      → flight_data    (Vosk → commands →    (optional, see §11)
  mission view)     .csv            planner → MAVSDK)
                        │                  │
                        ▼                  ▼
                   fit_sindy*.py     path_planner  ──►  mission uploaded to PX4
                                     (A* + B-spline)     and shown in QGC
```

PX4 normally exposes one telemetry stream for QGC (14550) and one for onboard programs (14540). We needed more, so we add a second stream by hand at the PX4 prompt:

```
pxh> mavlink start -u 14535 -o 14530 -r 4000 -t 127.0.0.1
```

`-u` is the port **PX4 binds locally**; `-o` is the port PX4 **sends to**. The voice controller listens on **14530** (the `-o` value, *not* the `-u` value). Using different local and remote ports is what avoids the "Address already in use" error we first hit.

Each MAVSDK program also starts its own helper process (`mavsdk_server`) with its own gRPC port, so they do not collide:

| Program | MAVLink port it listens on | Internal gRPC port |
| --- | --- | --- |
| `data_logger.py` | 14540 | 50051 |
| `voice_controller_v4_gemini.py` | 14530 | 50052 |
| `live_hub.py` | 14535 | 50053 |

---

## 3. Repository layout

| File | Purpose |
| --- | --- |
| `voice_controller_v4_gemini.py` | **Main program.** Voice commands, saved points, B-spline missions. |
| `path_planner.py` / `path_planner_2.py` | First planner: one bypass waypoint per building on the straight line, then a spline. (The two files are identical.) |
| `path_planner_v3.py` | **Current planner**: A\* around all inflated buildings, then a cubic B-spline and a collision check. |
| `voice_fixes_patch.py` | Fixes for the voice controller: persistent saved points, home-anchored coordinates, 35-item missions, robust mission start, safer geofence handling. |
| `obstacles.json` | Buildings as boxes in metres, relative to the home position. `obstacles.json.save` and `obstacles1.txt` are earlier copies. |
| `data_logger.py` | Records telemetry to `flight_data.csv`. |
| `flight_data.csv` | Our recording: 47,035 rows, about 17 minutes. |
| `fit_sindy.py` | First SINDy attempt (degree-2 library on 9 variables). |
| `fit_sindy_clean.py` | Second SINDy attempt (small, interpretable model with a held-out test). |
| `operation_log.csv` | Log of voice commands. |
| `voice_control.py`, `voice_control_v2.py`, `voice_controller_v3_newlogic.py` | Earlier voice versions, kept for history (§9). |
| `testing.py` | Minimal MAVSDK connection test. |
| `foxglove_bridge.py`, `live_hub.py`, `live_dashboard.py` | Live-visualisation experiments, not part of the main flow (§11). |
| `docs/images/` | Screenshots used in this README. |

---

## 4. How to run it

### 4.1 What you need

- **Linux** with PX4-Autopilot built for SITL and Gazebo (`gz_x500`). We developed on Ubuntu with QGroundControl on Windows; a native Linux setup works the same way.
- **QGroundControl** installed.
- **Python 3.10 or newer** (the code uses the built-in `anext`).
- A **microphone** that Python can open. If PX4 runs inside WSL, audio must be passed through to Linux (for example with WSLg), or the voice program will not hear you.
- First run downloads the small English Vosk model (`vosk-model-small-en-us-0.15`) into `~/.cache/vosk/`, so it needs internet once.

```bash
pip install mavsdk vosk sounddevice numpy scipy pandas pysindy matplotlib
# only for the optional live-view experiments:
pip install websockets pymavlink foxglove-websocket
```

### 4.2 Start order matters

**Terminal 1: simulation**

```bash
cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500
# wait for the pxh> prompt, then add the extra stream:
pxh> mavlink start -u 14535 -o 14530 -r 4000 -t 127.0.0.1
```

**QGroundControl:** open it. It connects on 14550 by itself.

**Terminal 2: data logger (start this one first, before the voice program)**

```bash
python3 data_logger.py        # Ctrl+C to stop and save
```

**Terminal 3: voice controller**

```bash
python3 voice_controller_v4_gemini.py
```

You should see `Voice Control Connected to PX4!`, then the Vosk model loading, then `--- VOICE CONTROL ACTIVE ---`.

### 4.3 First flight

1. Say **"start"** to begin listening.
2. Say **"take off"**.
3. Say **"point forward"** (or any name) to save where you are, then fly somewhere (for example with "move forward") and say **"fly to forward"**.
4. Say **"stop"** to pause listening, **"exit"** to quit.

Run `python3 fit_sindy_clean.py flight_data.csv` any time after recording to fit SINDy models.

### 4.4 Clean starts (avoid stale state)

PX4 remembers the last mission and geofence, and QGC downloads them when it connects. If QGC shows old routes or asks "Continue mission?": in QGC open **Plan**, click **Clear**, then **Upload**. Details are in §8 and §10.

---

## 5. Voice commands

| Say | Does |
| --- | --- |
| `start` / `stop` | Start or pause listening (the program ignores everything else while paused). |
| `arm`, `disarm` | Arm or disarm the motors. |
| `take off`, `land`, `hold`, `return home` | Basic flight. |
| `point <name>` or `save point <name>` | Save the current position as `<name>`. |
| `go to <name>` or `fly to <name>` | Plan and fly a B-spline route to the saved point. |
| `move forward / backward / left / right` | Fly 2 m/s for 2 s in that direction, then hold. |
| `ascend`, `descend` | Change altitude by 2 m (never below 2 m). |
| `circle maneuver` | Orbit a 10 m circle at 3 m/s, then hold. |
| `pitch sweep`, `roll sweep` | Command a ±10° sine-wave angle at 0.5 Hz for 6 s (used to excite the drone for SINDy). |
| `exit` | Stop the program. |

How a spoken sentence becomes an action:

1. `sounddevice` captures 16 kHz audio and feeds it to Vosk.
2. Vosk returns plain text such as `"fly to forward"`.
3. `dispatch_command()` tests the text against each command in order (string prefix and substring matching).
4. The matching function runs inside a `try/except`, so a failed command prints an error and the program keeps listening.
5. The result is appended to `operation_log.csv`.

Offboard commands (move, sweep) always fall back to `hold` in a `finally` block, so a failed command cannot leave the drone in an uncontrolled state.

**Important:** the matching is exact-string, not fuzzy. See §10 for what that causes.

---

## 6. Path planning: A\* and B-spline

When you say "fly to X", the controller turns the saved GPS point into a route in four steps.

### 6.1 Local frame

The drone's position and the target are converted from latitude and longitude to metres east (x) and north (y) of the home position, where φ is latitude and λ is longitude:

$$x = (\lambda-\lambda_0)\cdot 111320\cos\phi_0, \qquad y = (\phi-\phi_0)\cdot 111320$$

`obstacles.json` uses the same frame: every building is a box `x_min, x_max, y_min, y_max` in metres from home, plus a `height`.

### 6.2 Inflating the buildings

Each box grows by a safety margin $m$ (5 m by default). A point $(x, y)$ is a collision if it is inside any grown box:

$$x_{min}-m \le x \le x_{max}+m \quad\text{and}\quad y_{min}-m \le y \le y_{max}+m$$

### 6.3 A\*: finding a route around everything

**What it is.** A\* is a graph-search algorithm that finds the cheapest path between two points. Here the "graph" is a 1 m grid laid over the area (with 40 m of padding around the buildings, start and goal). Each free cell connects to its 8 neighbours; straight steps cost 1, diagonal steps cost 1.414, and cells inside a grown building are blocked.

**How it works.** It keeps a list of cells to try. At each step it expands the cell with the smallest

$$f(n) = g(n) + h(n)$$

where $g(n)$ is the distance travelled from the start to $n$, and $h(n)$ is the straight-line distance from $n$ to the goal. Because $h$ never overestimates the true remaining distance, the first time the goal is reached the path is the shortest possible on the grid. We also forbid squeezing diagonally between two blocked cells, so the route never clips a corner.

**What we do with the result.** The raw grid route is a staircase with many points. We keep only the corner points where the straight line to the next point would hit a building (a line-of-sight check sampled every 0.5 m). Those few points are the skeleton of the path.

**Why this replaced the old planner.** The first planner (`path_planner.py`) only looked at buildings lying close to the straight start-to-goal line and added one sidestep point per building. It could not weave through several buildings, and the smooth curve through its points could still cut through a corner. A\* sees all buildings at once, so a staggered layout produces an S-curve automatically.

### 6.4 The cubic B-spline: making it smooth

**What it is.** A B-spline is a smooth curve defined by control points. A cubic one (degree 3) is a weighted sum of the points, where the weights $N_{i,3}(u)$ are basis functions that depend on a parameter $u$ running from 0 to 1:

$$C(u) = \sum_{i=0}^{n} N_{i,3}(u)\,P_i, \qquad N_{i,k}(u) = \frac{u-u_i}{u_{i+k}-u_i}N_{i,k-1}(u) + \frac{u_{i+k+1}-u}{u_{i+k+1}-u_{i+1}}N_{i+1,k-1}(u)$$

Cubic means the curve has continuous position, direction and curvature, so a multirotor following it does not stop and turn sharply at each point.

**How we use it.** We add points along each straight piece of the A\* skeleton, at most 8 m apart, so the curve hugs the corners instead of cutting across them. Then SciPy's `splprep(points, s=0, k=3)` builds a cubic spline that passes through every point (`s=0` means no smoothing away from the points). Altitude changes in a straight line with distance travelled $s$ along a path of total length $S$:

$$z(s) = z_0 + \frac{s}{S}(z_1 - z_0)$$

**Checking the result.** The spline is sampled at 400 points and tested against the buildings (grown by half the safety margin). If any sample is inside, the margin increases by 2 m and the whole plan repeats, up to 6 times. The accepted curve is then sampled at **35 evenly spaced values of $u$**.

**Offline test.** On four staggered walls (a layout that forces an S-curve) and on our own `obstacles1.txt`, all 35 points stayed outside every building (path lengths 296 m and 222 m).

<img width="1210" height="660" alt="image" src="https://github.com/user-attachments/assets/7ef408fb-6739-4a3d-bbb8-295d7f6cb2e5" />


### 6.5 Sending it to PX4

The 35 points become 35 `MissionItem`s (fly-through, 3 m acceptance radius, heading left to PX4). The controller then:

1. clears the old mission (`clear_mission`),
2. uploads the new one,
3. waits until the drone is in the air and its position estimate is healthy,
4. starts the mission, retrying up to three times if PX4 answers `DENIED`.

**Speed.** Do not set `speed_m_s` on every item. MAVSDK then inserts a "Change speed" command before each waypoint, and QGC shows 70 items for 35 points . We use NaN for speed and set the cruise speed once with the PX4 parameter `MPC_XY_CRUISE`.

<img width="1400" height="856" alt="image" src="https://github.com/user-attachments/assets/08e39aa5-ada4-4820-9d4f-4c18c0207642" />


---

## 7. Data logging and SINDy

### 7.1 What is recorded

`data_logger.py` samples MAVSDK telemetry about every 0.02 s and writes one row per sample:

| Columns | Meaning |
| --- | --- |
| `timestamp` | seconds since logging started |
| `roll, pitch, yaw` | attitude in degrees |
| `vx, vy, vz` | velocity north, east, down (m/s) |
| `ax, ay, az` | accelerometer reading in the drone's own frame (m/s²). At rest `az` is about −9.8, because the sensor measures the reaction to gravity, not true acceleration. |
| `p, q, r` | angular velocity about roll, pitch and yaw axes (rad/s) |

Our file `flight_data.csv` has **47,035 rows** (median step 0.0208 s, about 48 Hz) covering about 17 minutes. The drone was flown by voice, including roll and pitch sweeps.

### 7.2 What SINDy is doing

**The question.** For the state $x$ (roll, pitch, and so on), find the function $f$ in $\dot x = f(x)$ that makes the drone's motion. Textbook physics would give $f$ from mass and inertia; SINDy (Sparse Identification of Nonlinear Dynamics) tries to find it from the recording instead.

**Step 1: estimate the rates.** The logger gives $x$ at each moment, not $\dot x$. We compute $\dot x$ from the samples (`SmoothedFiniteDifference`: smooth with a Savitzky-Golay filter, then differentiate). Differentiating raw noisy data multiplies the noise, so this step is where most of the error comes from.

**Step 2: build a library of candidate terms.** For every sample, evaluate a table of possible building blocks, for example a constant, each variable, and products of two variables. With variables $(x_1,\dots,x_n)$ and degree 2, the library is

$$\Theta(X) = \begin{bmatrix} 1 & x_1 & \cdots & x_n & x_1^2 & x_1x_2 & \cdots & x_n^2 \end{bmatrix}$$

**Step 3: find the few terms that matter.** We assume each state's rate is a sum of a few library terms:

$$\dot{X} \approx \Theta(X)\,\Xi$$

$\Xi$ is a matrix of coefficients, one column per state. Ordinary least squares would give every term a small nonzero weight. SINDy uses **STLSQ** (sequentially thresholded least squares) to force most to exactly zero:

1. Solve $\Xi = \arg\min \lVert \dot X - \Theta\,\Xi \rVert_2^2 + \alpha\lVert \Xi \rVert_2^2$ (a ridge-regularised least-squares fit; $\alpha = 0.05$).
2. Set every coefficient with $|\Xi_{ij}| < \lambda$ to zero (λ is the threshold).
3. Refit using only the surviving terms. Repeat until nothing changes.

The result is a short equation you can read. The threshold λ is the main knob: too low keeps noise, too high deletes real terms.

### 7.3 Attempt 1: `fit_sindy.py` (what we ran first)

- Variables: 6 states (roll, pitch, yaw, vx, vy, vz) plus `ax, ay, az` treated as control inputs, degree 2 with interactions. That is a library of 55 terms per equation.
- Threshold λ = 0.001.

**Result.** Nearly all 55 terms survive in every equation, with large coefficients (for example `roll' = −9.2 + 4.3·roll + 111·ax + 163·ay + …`). That is a fitted curve, not a discovered law: the threshold was too low to remove anything, and a model with hundreds of terms cannot be read or trusted.

**One equation that did make sense:**

```
(vz)' = 9.317 + 0.962·az + (small terms)
```

At hover `az` is about −9.81, so this gives `vz' ≈ 0`. It matches the physical relation "downward acceleration = gravity + accelerometer reading", with 9.3 close to g and 0.96 close to 1. SINDy found physics here because the relationship is strong and simple.

**Two design mistakes in this attempt.**
- `ax, ay, az` are **measurements**, not inputs. A control input is something we choose (motor thrust, a commanded attitude). Treating a measured output as an input makes the model fit itself.
- Yaw is a heading angle, not a dynamic variable here, and it wraps from +180° to −180° (2 jumps in our file). It adds no physics and creates fake terms such as `roll' ∝ 0.106·yaw·vx`.

### 7.4 Attempt 2: `fit_sindy_clean.py` (what we recommend)

- Variables: roll and pitch in radians, and body rates `p, q` in rad/s. No yaw, no accelerometer.
- Degree 1 (linear), threshold λ = 0.2.
- Trained on the first 70 % of the recording and scored on the last 30 % (a time split, not a random one).

```
(roll)'  =  1.015·p
(pitch)' =  0.997·q
(p)'     = −4.039·roll
(q)'     =  0.676·roll − 2.734·pitch + 0.227·p
```

Only 6 coefficients remain out of 20 possible (5 library terms for each of 4 states).

| State | Held-out one-step R² | Reading |
| --- | --- | --- |
| roll | 0.964 | Found: the rate of roll is the roll rate, $\dot\phi \approx p$ (coefficient 1.015 ≈ 1). |
| pitch | 0.974 | Found: $\dot\theta \approx q$ (coefficient 0.997 ≈ 1). |
| p | 0.26 | Partly: a restoring term $\dot p = -4.04\,\phi$, a spring that pulls roll back to level with natural frequency $\sqrt{4.04}\approx 2.0$ rad/s. |
| q | 0.27 | Partly: restoring terms on pitch and roll, similar in size. |

R² is $1 - \sum(y-\hat y)^2 / \sum(y-\bar y)^2$: 1 is perfect and 0 is no better than predicting the average.

**How to read this honestly.**
- The first two results are the standard kinematic relations (for small angles, angle rate equals body rate), recovered from data alone with coefficients within 2 % of the true value of 1.
- The spring terms are real structure but only explain about a quarter of the angular acceleration. Angular acceleration is the derivative of a noisy gyro signal, so it is noisy, and the thing that actually causes it, the **motor torque, was never logged**.
- PX4's controller was running the whole time. So the equations describe **the drone and PX4's controller together** (a closed loop), not the bare airframe. The "spring" is what PX4's attitude controller does.
- It is not a model you could use to design a new controller yet.

**What would make it better** (not done): log the actuator outputs or the attitude and thrust commands alongside the telemetry, use the sweep segments as the training data, and fit the rate equations $\dot\omega = f(\omega, \text{torque})$ with torque as a true input.

Run it with:

```bash
python3 fit_sindy_clean.py flight_data.csv
```

---

## 8. Using QGroundControl with it

QGC has two views. **Fly** is for flying; **Plan** is for building and editing missions. Menu names vary a little between QGC versions.

### 8.1 Reading your mission in Plan view

| Area | What it is |
| --- | --- |
| Top bar: **Open, Save, Upload, Clear**, menu | Open or save a `.plan` file; **Upload** sends the plan to the drone; **Clear** empties it; the menu has **Download** (read the mission back from the drone) and KML export. |
| Left tools: **Takeoff, Waypoint, ROI, Pattern, Return** | Tools that add items to the plan. |
| Right panel: **Plan Info, Defaults, Mission Items** | Defaults set altitude and speed for new items; Mission Items is the numbered list. |
| Numbered dots on the map | Item numbers from the list. |
| Bottom chart | Altitude profile along the route. |
| Blue pin icon, top right | Switches layer between Mission, GeoFence and Rally points. |

### 8.2 Add and edit waypoints by hand (no code)

1. Plan view, click **Waypoint**, click the map. Each click adds the next numbered waypoint.
2. **Drag** a dot to move it. **Click** it to open its card in the right panel: altitude, hold time, camera actions. The card's menu offers Insert, Delete, Edit position (type exact latitude and longitude) and Change command.
3. **Insert between two points** by clicking the small circle on the middle of the line.
4. Click **Upload**, switch to Fly view, slide **Start Mission**.

### 8.3 Edit a route the voice controller created

1. Say "fly to X". The route is uploaded and starts.
2. In Plan view use the menu to **Download** from the vehicle. The points and list appear.
3. Drag or delete points. QGC joins waypoints with straight lines, so the path looks smooth only while the points are dense. Delete points and it becomes angular.
4. **Upload**, then **Continue Mission** in Fly view.

### 8.4 Save a route and reuse it

Build or download a route, click **Save**, name it (for example `irchel_loop.plan`). Next session click **Open**, **Upload**, **Start Mission**. A plan file also stores any geofence and rally points you drew.

### 8.5 Showing buildings in QGC

Switch to the **GeoFence** layer, add a **Polygon Fence**, draw it around a building, and set it to **Exclusion**. Do **not** use Inclusion for buildings: it caused the "Geofence: approaching or outside geofence" warning and denied mission starts (§10).

<img width="1021" height="543" alt="image" src="https://github.com/user-attachments/assets/f7963fdc-adeb-4dd3-862d-30827c3cbc7f" />


### 8.6 The "Continue mission?" prompt and "Not in Mission mode"

PX4 stores the mission and how far it got. QGC reads it on connect and offers to continue. The voice controller replaces the stored mission every time it plans, and whichever program uploaded last decides what the drone flies. A voice "fly to" overwrites a mission you made by hand.

---

## 9. How the project evolved

| Stage | Files | What it did | Why we moved on |
| --- | --- | --- | --- |
| 1. Connect | `testing.py`, `voice_control.py` | Connect to PX4; voice for take off, land, hold. | Needed more commands and a separate port for each program (fixed with `mavlink start -u 14535 -o 14530`). |
| 2. Full command set | `voice_control_v2.py` | Arm, disarm, move pulses (Offboard velocity), altitude steps, orbit, roll and pitch sweeps, saved points, straight-line "go to". | Straight lines flew over buildings (`docs/images/01_straight_line_goto.png`). |
| 3. Planning | `voice_controller_v3_newlogic.py`, `path_planner.py` | B-spline route around buildings from `obstacles.json`; operation log. | Mission list showed stale routes; start was denied; buildings looked wrong. |
| 4. QGC overlay | `voice_controller_v4_gemini.py` | Tried drawing the buildings in QGC through the geofence upload. | Triggered geofence failsafes (§10). |
| 5. Better planner | `path_planner_v3.py`, `voice_fixes_patch.py` | A\* around all buildings, home-anchored frame, 35-item missions, saved points kept between runs, robust start. | Current state. |

<img width="1388" height="868" alt="image" src="https://github.com/user-attachments/assets/f25796c1-fa6c-4002-9475-c9ad70eabef0" />

<img width="1390" height="868" alt="image" src="https://github.com/user-attachments/assets/688ef97c-8d9e-4f9c-b694-b4dffa8db663" />

In parallel we logged data (`data_logger.py`) and tried SINDy (`fit_sindy.py`, then `fit_sindy_clean.py`).

---

## 10. Known issues and problems you may hit

Each of these cost us time. The first group is fixed or has a safe workaround; the second group is still open.

### Fixed or avoidable

| Problem | Cause | What to do |
| --- | --- | --- |
| `Geofence: approaching or outside geofence`, and `start_mission` returns `DENIED` | v4 uploads each building as an **Inclusion** fence, so PX4 treats it as "stay inside this box". Home is outside it. | Draw buildings as **Exclusion** in QGC or skip the fence upload. For demos, `param set GF_ACTION 0` at the `pxh>` prompt makes PX4 only warn. Do not add a large "master" inclusion box: that triggered a failsafe in our tests. |
| Old route and buildings appear when QGC opens | PX4 keeps the last mission and fence; QGC downloads them on connect. | **Clear** then **Upload** in Plan view, or call `clear_mission()` and `clear_geofence()` at start. |
| 70 items for 35 waypoints | A speed on every item makes MAVSDK add a "Change speed" before each one. | Speed = NaN, set `MPC_XY_CRUISE` once. |
| Buildings drift and the drone does not return to the same point | Coordinates were measured from wherever the drone was at the time. Saved points also kept absolute altitude while missions use relative altitude. | Anchor to the **home** position; save relative altitude (`voice_fixes_patch.py`). |
| Saved points disappear every run | They lived only in memory. | Saved to `saved_points.json` on every save and loaded at start. |
| `start_mission` denied the first time, or "Not in Mission mode" | The mission was started right after upload, before the drone was airborne and its position estimate was ready. | Upload, wait for in-air and position OK, then start, retrying up to 3 times. |
| `GeofenceData.__init__() missing 1 required positional argument: 'circles'` | In our MAVSDK version, `GeofenceData` takes polygons **and** circles. | `GeofenceData(polygons, [])`. |
| `Address already in use` | Two programs bound the same MAVLink port. | One port and one gRPC port per program (§2). |

### Still open

| Problem | Effect | Workaround or idea |
| --- | --- | --- |
| **Saying "ascend" or "descend" pauses listening.** | The stop test is `"end" in text`, and both words contain "end". Neither command has ever been logged as executed in `operation_log.csv`. | Match whole words (for example `text.split()`), or use "up"/"down". |
| **Speech recognition mishears point names.** | Names such as "eight" were logged as "eat", "date", "aid"; "left" as "being", "see", "year". A misheard "fly to" name finds no saved point, and the controller replies that it is missing. | Use names Vosk hears reliably, or add fuzzy matching (compare the heard word with saved names by similarity and accept the best above a threshold). |
| **The operation log records failures as `EXECUTED`.** | The log line is written after the command function returns, even when the function printed "No saved point named …". It cannot be used to count successes. | Have each command return success or failure and log that. |
| **Substring matching can fire on the wrong command.** | `"arm" in text` also matches words like "alarm"; `"land"` matches "island". | Whole-word matching. |
| **`data_logger.py` overwrites `flight_data.csv` every run** (it opens the file in write mode). | A second run erases the previous recording. | Rename or copy the file after each session, or change the file name to include a timestamp. |
| **`data_logger.py` kills every `mavsdk_server` process** at start and exit (`pkill -9`). | If the voice controller is already running, the logger kills its backend and voice control stops responding. | Start the logger **first**. |
| **Buildings in `obstacles.json` are hand-written boxes.** | They are not the real Gazebo building positions, so the planner avoids imaginary buildings. See the orange boxes in `docs/images/02_bspline_mission_flying.png`. | Measure real building corners (QGC map or the Gazebo world file) and enter them in metres from home. |
| **Sweep altitude drift.** | Attitude sweeps use a fixed thrust guess (`HOVER_THRUST_GUESS = 0.5`), so the drone climbs or sinks a little during a 6 s sweep. | Keep sweeps short, or estimate the real hover thrust. |
| **QGC draws straight lines between waypoints.** | A downloaded route is only as smooth as its point spacing. | Keep points dense, or save the voice-generated plan rather than thinning it. |

---

## 11. Left out, abandoned or not finished

Included so that nobody assumes these work:

- **Foxglove streaming** (`foxglove_bridge.py`, `live_hub.py`). Streamed attitude and position over WebSocket on port 8765 for Foxglove Studio. Abandoned because the extra programs competed for the same MAVLink ports as the logger (14540) and PX4's own local port (14535), and the benefit was small next to QGC. Treat the code as an unfinished experiment.
- **Live matplotlib dashboard** (`live_dashboard.py`). Plots roll and pitch live from port 14550 using the TkAgg backend (written for WSL). It shares a port with QGC and was not kept.
- **Full geofence rendering of buildings** (`upload_obstacles_to_qgc` in v4). Replaced by drawing exclusion zones in QGC, or no fence at all.
- **SINDy as a controller or predictor.** We fitted models and read them. We did **not** build a controller from them, run multi-second predictions, or compare against a physics model. Reason: PX4 flies the drone, and the motor inputs needed for a true model were not logged.
- **Motor or thrust logging.** `data_logger.py` records motion but not the commanded inputs. This is the biggest gap for system identification.
- **Real obstacle sensing.** Buildings are typed in; the drone senses nothing.
- **Real hardware.** Everything is simulation.
- **Fuzzy speech matching.** The matching is exact-string today (§10).
- **Automated tests.** There are none; we tested by flying in SITL and checking planner output offline.
- **Dynamic replanning.** A route is planned once per "fly to"; if the world changes during the flight, nothing replans.
- **Altitude-aware avoidance.** The planner routes around the footprint of a building at any altitude and does not fly over a building even when the drone is higher than it (`height` in `obstacles.json` is stored but not used for planning).

---

## 12. What you can do with it

- **Fly by voice:** take off, save places, and fly to them with obstacle avoidance.
- **Add your own obstacles:** edit `obstacles.json` (metres from home) and say "fly to X".
- **Edit routes visually:** download the generated route into QGC Plan view, drag or delete points, upload.
- **Collect data:** run `data_logger.py` during flights, use "pitch sweep" and "roll sweep" to excite the drone.
- **Fit models:** run `fit_sindy_clean.py` and compare with your own variable choices and thresholds.
- **Extend it:** natural next steps are fuzzy speech matching, whole-word command matching, logging actuator commands, real building coordinates, and using the fitted model for a controller.

---

## 13. Credits and references

- PX4 Autopilot and PX4 SITL, Gazebo, QGroundControl, MAVSDK-Python, Vosk, SciPy (`splprep`/`splev`), PySINDy.
- S. L. Brunton, J. L. Proctor, J. N. Kutz, *Discovering governing equations from data by sparse identification of nonlinear dynamical systems*, PNAS 2016 (SINDy).
- P. E. Hart, N. J. Nilsson, B. Raphael, *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*, 1968 (A\*).
- C. de Boor, *A Practical Guide to Splines*, 1978 (B-splines).
- Parts of the code and debugging were developed with help from AI assistants (Claude and Gemini); all code was run and tested by the authors in simulation.
