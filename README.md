<p align="center">
  <img src="amrita_logo.png" alt="Amrita Vishwa Vidyapeetham" width="420"/>
</p>

# Voice-Controlled Drone — Four Simulation Models

**Data Driven Control of Drones** · B.Tech Artificial Intelligence Engineering
School of Artificial Intelligence, Amrita Vishwa Vidyapeetham, Coimbatore

<a id="team"></a>
## Team

### Faculty Supervisor

| Role | Name | Email |
|---|---|---|
| Faculty Supervisor | Prof. S. Sunil Kumar | s_sunilkumar@cb.amrita.edu |

### Team Members

| Name | Roll number | Email |
|---|---|---|
| C. Karthik | CB.SC.U4AIE24107 | cb.sc.u4aie24107@cb.students.amrita.edu |
| Ch. Virinchi | CB.SC.U4AIE24158 | cb.sc.u4aie24158@cb.students.amrita.edu |
| K. Jatin Chandra Gupta | CB.SC.U4AIE24162 | cb.sc.u4aie24162@cb.students.amrita.edu |
| M. Som Praneeth Babu | CB.SC.U4AIE24029 | cb.sc.u4aie24029@cb.students.amrita.edu |

<a id="abstract"></a>
## Abstract

This project builds a drone that can be flown by voice, and tests the idea in four different simulators: PyBullet, MuJoCo, Gazebo and MATLAB. In every one of them a person speaks (or types) a short command such as "forward", "stop" or "go to the library", and a simulated quadrotor carries it out. The four models share one voice front end and differ in what they study. The PyBullet model concentrates on understanding messy speech, exact moves and stopping safely before an obstacle. The MuJoCo and Gazebo models identify the drone's rotational dynamics from its own flight data (with SINDy, sparse identification of nonlinear dynamics) and calculate the attitude controller from the learned equation instead of tuning it by hand; DMDc (dynamic mode decomposition with control) is used as a linear check. The MATLAB model adds a command interface, an obstacle-stop check and telemetry analysis in a 3D city. In the MuJoCo model the learned equation predicts held-out data with R² ≈ 0.995, and the drone flies a 1:10 scale model of the Amrita Coimbatore campus, going to any of 31 named buildings by voice. All results are from simulation; there was no hardware flight.

<a id="introduction"></a>
## Introduction

A quadrotor has four propellers and no steering wheel. Tilting, turning, moving and holding still all come from choosing four thrust values hundreds of times per second, so a controller is needed between what a person wants and what the motors do. Two practical questions follow. How does a person tell the drone what to do in plain words? And how do we know the controller matches the real physics of the vehicle?

This repository answers both in stages. The first half is the **voice interface**: speech is turned into text, the text is matched fuzzily to a list of known commands, and a command is accepted only when it is close enough, so a mis-heard word does nothing rather than the wrong thing. The second half is the **flight side**: depending on the model, the drone is moved by a simple kinematic model, by a controller built from an identified equation, or by a command interface with telemetry. A recognised command is always treated as a request, and each model checks it against obstacles, map edges and building cells before the drone moves.

We built four separate models on purpose. Each simulator has different strengths, and comparing them shows which design choices matter. The sections below first explain the voice recognition common to all models, then give each model its own write-up: the problem, the idea, the mathematics, what it can do, the world it flies in, and what is and is not claimed.

## Contents

- [Team](#team)
- [Abstract](#abstract)
- [Introduction](#introduction)

1. [Voice Recognition (common to all models)](#voice-recognition)
2. [Model 1 · PyBullet — exact moves and safe stopping in a city](#model-1)
3. [Model 2 · MuJoCo — a controller built from learned physics, on a campus map](#model-2)
4. [Model 3 · Gazebo — learned-physics controller, summary](#model-3)
5. [Model 4 · MATLAB — Drone Flight Lab with telemetry analysis](#model-4)

---

<a id="voice-recognition"></a>
## Voice Recognition (common to all models)

Every model in this repository is driven the same way: a person **speaks (or types, or presses a button)**, and the drone does what was asked. The voice pipeline is explained once here; the model sections below only describe what is different about them.

### The pipeline

| Stage | What happens | In plain words |
|---|---|---|
| **1. Listen** | A microphone records a phrase. | Hear the person. |
| **2. Transcribe** | A speech-to-text service turns the audio into text (the PyBullet model uses Google's Web Speech API through the Python `SpeechRecognition` library). | Audio becomes words. |
| **3. Match** | The text is matched, fuzzily, to the list of commands the drone knows. | Work out what the person *meant*, not just what was heard. |
| **4. Accept or ignore** | A match is accepted only if it is close enough; anything else is ignored rather than guessed. | Better to do nothing than the wrong thing. |
| **5. Act** | The command becomes a flight action (move, turn, hover, land, return home, or fly to a named place), and the safety logic decides whether it is allowed. | Fly it, if it is safe. |

### Why matching must be fuzzy

A speech recogniser returns its *most likely* transcript, not what the speaker meant. With an accent, a noisy room or a short word, "forward" can come back as "for this" and "right" as "write". So the text is never compared for exact equality. Every known command phrase is scored against what was heard, and the best one is accepted only if its score reaches a threshold:

```math
k^{*} = \arg\max_k\, s(\text{heard},\,c_k), \qquad \text{accept only if } s(\text{heard},\,c_{k^{*}}) \ge \tau
```

Here `c_k` is the k-th command phrase, `s` is a similarity between 0 and 1, and `τ` is the acceptance threshold. The similarity used is the standard character-matching ratio (the one behind Python's `difflib`):

```math
s(a,b) = \frac{2\,M(a,b)}{|a|+|b|}
```

`M` is the number of matching characters and `|a|`, `|b|` are the lengths of the two phrases. For example, "forwad" against "forward" has `M = 6` and a total length of 13, so `s = 0.923`, which is close enough to be repaired to *forward*.

### What the commands look like

- **Movement and control:** take off, forward, backward, left, right, up, down, hover, land, return home (the exact list differs slightly per model).
- **Amounts (PyBullet model):** "forward 3 meters", "turn 90 degrees", digits or number words ("twenty five"), metres or feet, degrees or radians.
- **Places (MuJoCo and Gazebo models):** the same fuzzy idea is applied to the 31 named campus buildings, so "go to the central library" becomes a target position.
- **Safety first:** a recognised command is still only a *request*. Each model checks it against obstacles, map edges and building cells before the drone moves.

### Limits common to every model

- Speech-to-text needs a working microphone and, for the Google service, an **internet connection**.
- In a noisy room the recogniser also returns phrases that are not commands; the drone ignores them.
- Everything here is **simulation**; there was no hardware flight.

---

### The four models at a glance

| | **Model 1 · PyBullet** | **Model 2 · MuJoCo** | **Model 3 · Gazebo** | **Model 4 · MATLAB** |
|---|---|---|---|---|
| **Focus** | Understanding messy speech, exact moves, safe stopping | Controller built from a learned equation | Controller built from a learned equation (summary) | Command interface, obstacle stop, telemetry analysis |
| **Flight model** | Kinematic (velocity with limited acceleration) | Full rotor thrust and attitude physics | Learned-physics controller | Simulated drone state in a 3D city |
| **World** | Generated city plaza with towers and traffic | 1:10 scale Amrita Coimbatore campus | Campus scale model | 3D city-like environment |
| **Obstacle handling** | Ray test, brakes 0.40 m before a safety zone | Route planning (A\*) around building cells | Route planning (A\*) around building cells | Configured obstacle-stop check |
| **Voice** | Google Web Speech, top-3 guesses, spelling repair | Fuzzy match of commands and places | Fuzzy match of commands and places | `LISTEN TO VOICE` button with a Windows PowerShell helper |

---

<a id="model-1"></a>
## Model 1 · `Drone_pybullet` — A Voice-Controlled Drone That Understands Messy Speech and Stops Before It Hits Anything

**In one sentence:** we built a quadcopter in a generated city that you fly by *speaking* (or typing, or pressing keys): it turns imperfect speech-to-text into exact commands, flies precisely as far as you ask, brakes to a safe stop in front of obstacles, and draws its own flight as live graphs in your browser.

<p align="center">
  <img src="https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/Drone_pybullet/pybullet_voice_drone/screenshots/city_overview.png" alt="The city plaza from above: helipad, kiosk towers, trees, ring road with traffic, skyline" width="95%">
</p>

<p align="center"><i>The real simulation window. The plaza has a helipad, three kiosk towers (the obstacles), trees, a ring road with moving traffic, pedestrians and a skyline. Everything, including the textures, is generated in Python.</i></p>

---

### 1. The problem, in plain words

A voice-controlled drone has to solve three problems at once.

- **Speech-to-text is not a command.** The speech service returns its *most likely* transcript, not what you meant. With an accent, a noisy room or a short word you get "for this" instead of "forward", or "write" instead of "right".
- **Words have to become safe, predictable motion.** "Forward 3 meters" must mean three metres, not "somewhere around three", and the drone must never fly into something just because it was told to.
- **The operator has to see what is happening and why.** If the drone stops, you need to know whether it was told to, or whether an obstacle held it.

**Our approach is to make the command-and-safety layer the product.** The flight model is deliberately simple (*kinematic*: commanded velocities with a limited acceleration, not simulated rotor thrust), so all the effort goes into the parts a voice operator actually meets: understanding speech, flying exact moves, stopping safely, and showing the state clearly. Simulating thrust and attitude control is what the MuJoCo and Gazebo attempts in this repository do.

### 2. The idea, in five steps

| Step | What we do | In plain words |
|---|---|---|
| **A. Listen** | A microphone thread records phrases; a second thread sends each one to Google's speech service in Indian English and asks for the **top three** guesses. | Hear the phrase, and keep the runner-up guesses. |
| **B. Understand** | A parser picks the first guess that is a command, joins split words, repairs one-letter slips and reads numbers and units. | "For word three feet" becomes *forward, 0.91 m*. |
| **C. Decide** | The flight state decides whether the command is allowed. A ray cast along the direction of travel checks every obstacle's safety zone. | Look ahead before moving. |
| **D. Move** | A kinematic flight model follows the command with limited acceleration and slows down smoothly as a measured goal or an obstacle gets close. | Arrive exactly, and brake early enough. |
| **E. Show** | A 3D city, four cameras, labels on the drone, a sensor ray, a flight trail, a CSV log and live graphs in the browser. | Make the state visible. |

### 3. The mathematics, step by step

This section is for a reader who wants the actual rules. Every symbol is defined where it first appears.

#### 3.1 What is being modelled

The drone's state is its position `p = (x, y, z)` in metres and its heading `ψ` (yaw). The world frame has `+x` forward at `ψ = 0`, `+y` left, `+z` up. A command sets a *wanted* velocity in the drone's own frame, `(f, l, u)` = (forward, left, up). It is rotated into the world frame by the heading:

```
 want_x = f·cos ψ − l·sin ψ
 want_y = f·sin ψ + l·cos ψ
 want_z = u
```

The real velocity `v` follows the wanted velocity, but is only allowed to change by `a·Δt` per step, with `a = 3 m/s²`:

```
 v  ←  v + clip(want − v, −a·Δt, +a·Δt)        (per axis, every frame)
 p  ←  p + v·Δt
```

Heading turns the same way with a rate limit (45°/s, reached with 180°/s²). The speeds are 1.5 m/s horizontally and 1.0 m/s vertically. **There is no thrust or attitude model**: the drone's lean while moving (8° per m/s, at most 15°) is purely visual.

#### 3.2 Turning speech into a command

The speech service returns ranked guesses `g₁, g₂, g₃`. The listener acts on the **first guess that parses as a command**, not simply the first guess, so `["for this", "forward", ...]` still means forward. Each guess is cleaned in four steps:

1. **Whole words only.** The text is split into lowercase words and numbers, so "island" is never read as "land", and "go to the left" still works.
2. **Joins.** Split command words are merged: "take off" → `takeoff`, "for word" / "four ward" → `forward`, "back ward" → `backward`.
3. **Spelling repair.** A word of five letters or more that is not already known is compared with every command word using difflib's similarity ratio:

```
 ratio = 2·M / T          M = characters in matching blocks,  T = total characters of both words
 repair the word only if  ratio ≥ 0.85
```

   For example "forwad" → *forward* has `M = 6`, `T = 13`, so the ratio is 0.923 and it is repaired. "island" against "landing" scores 0.615 and is left alone; "uphill" against "hover" scores 0.182. Words of four letters or fewer are never repaired, because they sit too close to too many other words.
4. **Rules for the intent.** The first command word wins, except that "home" beats everything ("go back home" returns home instead of flying backwards), and a rotate verb (*rotate, turn, spin, yaw*) looks at **all later words** for a side, so "turn to the left" means rotate left, never slide left.

**Amounts.** The first number in the phrase is the amount, written as digits ("2.5") or words ("twenty five", "one hundred eighty"). Distances are metres unless the unit is feet (`× 0.3048`); turns are degrees unless the unit is radians (`× 180/π`). Moves are limited to 0.1 – 50 m and turns to 720°.

**Not losing the next command.** The listener thread only records (a phrase ends after 0.6 s of silence); a worker thread transcribes. A phrase spoken while the previous one is still being transcribed is queued (up to six), not lost.

#### 3.3 Exact moves, turns, take-off and return home

A *measured* command stores how much is **remaining**, `r`, and flies it with a speed that fades near the goal:

```
 move:    speed = clip(2·r, 0.15, v_cmd)           finished when r ≤ 0.02 m
 turn:    rate  = clip(2·r, 0.15 rad/s, 45°/s)     finished when r ≤ 0.005 rad (0.3°)
 home:    speed = clip(1.5·d, 0.15, 1.5 m/s)       finished when d ≤ 0.1 m, then lands
```

Because `ṙ = −2r` once the speed has faded, the remaining distance shrinks exponentially with a time constant of 0.5 s, so the drone arrives smoothly and the 0.15 m/s floor stops it creeping forever. A move that is pinned (map edge, ceiling, blocked) for 1.5 s is abandoned; a move that takes longer than `3·distance/speed + 10 s` times out. Both end with the drone hovering.

Take-off and landing are the same idea on the vertical axis:

```
 take-off:  up = clip(2·(2.0 − z),  0.1, 1.0)       climbs to 2.0 m
 landing:   up = −clip(2·(z − 0.31), 0.15, 1.0)     descends to the pad (body centre 0.31 m)
```

The four flight states (`LANDED → TAKEOFF → FLYING → LANDING`) decide what is allowed: on the ground, only "take off" or straight up is accepted, and a manual command during take-off or landing cancels the autopilot.

#### 3.4 Seeing obstacles: the safety zone and the ray

The drone is not a point. Its footprint including the propeller tips has a radius `R = 1.32 m`, and the flight keeps a clearance `s = 0.25 m` from anything. So each obstacle box (centre `c`, half-sizes `h`) is **grown** into a *safety zone*:

```
 zone = [ c − h − g ,  c + h + g ]       g = (R + s, R + s, 0.30 + s)  =  (1.57, 1.57, 0.55) m
```

The vertical growth is smaller (0.30 m, skids to rotor discs). The **gap** to a zone is the straight-line distance from the drone to the zone's surface (0 once inside):

```
 gap(p) = sqrt( Σ_axes  max(a − p, 0, p − b)² )          [a, b] = the zone's range on that axis
```

Every frame a ray is cast from the drone along its direction of travel `d` and tested against every zone with the **slab method**: on each axis compute `t₁ = (a − o)/d` and `t₂ = (b − o)/d`; the ray enters the box at `t_near = max(min(t₁,t₂))` over the axes and leaves at `t_far = min(max(t₁,t₂))`. It hits only if `t_near < t_far` and `t_near` is within the **detection range of 2.0 m**. Rays that only graze a face or an edge count as misses.

#### 3.5 Stopping in front of an obstacle (braking)

Braking from speed `v` at deceleration `a_b` takes the distance `v²/(2·a_b)`. The drone must be able to stop **0.4 m before the zone** (the stop buffer), so with `t_hit` the ray distance to the zone, the fastest speed it is allowed right now is

```
 v_allowed = sqrt( 2 · a_b · (t_hit − 0.4) )            a_b = 2 m/s²
```

The commanded velocity is scaled down to `v_allowed` whenever that is smaller than the commanded speed, and the real velocity then follows it down at the flight model's acceleration limit, so the drone starts to slow exactly when it has to and not before. If `v_allowed < 0.1 m/s` the drone is **held**: the command is cancelled (so it cannot restart on its own when the drone is later turned away), the status line turns red, the tower pulses red and the rear lights flash. Moving away, or in any clear direction, works as normal. `a_b = 2` is deliberately gentler than the 3 m/s² the flight model can change speed by, so the planned stop is always achievable.

**A worked check, against the real run.** The first tower is at `x = 3.5` with half-width `0.7`, so its face is at `x = 2.8`. The zone grows by `1.57`, so the zone face is at `x = 2.8 − 1.57 = 1.23`. Stopping `0.4 m` before it gives `x = 0.83`, and the measured stop position was `x = 0.83`. From the 1.5 m/s cruise, braking starts when `t_hit − 0.4 = 1.5²/(2·2) = 0.56`, i.e. about 0.96 m before the zone face (at about `x = 0.27`).

#### 3.6 The optional alternative: steering around

With `--on-obstacle steer` the drone does not stop. Once per encounter it picks **left or right, whichever needs the smaller sideways shift to clear everything within the planning horizon (4 m)**, and is free and inside the world; otherwise it climbs over, otherwise it stops. It strafes at 2 m/s while its forward speed fades, and resumes the original command once the ray is clear by an extra 0.1 m (a hysteresis margin that stops it flicking between the two behaviours). It prefers the side it used last unless the other side is 0.5 m cheaper.

**The safety net (both modes).** Motion is integrated **one axis at a time**. If a step along one axis would put the drone inside an obstacle (while it was not already inside), that axis's velocity is zeroed and the step is cancelled, so the drone slides along a surface instead of through it. In the measured runs this net was never needed in stop mode.

#### 3.7 Keeping the window fast

Everything above is plain Python, so it is tested without a window. The cost is in the window: an early version managed about 11 frames per second. Every kind of PyBullet call was timed in the real window with the full city loaded:

| Call | Cost |
|---|---|
| Move a body | under 0.1 ms |
| One physics step | 1 – 2 ms |
| Rewrite a text label or change a colour | several ms (3 – 6 ms measured, and higher while the scene moves) |

So the design **moves things every frame but rewrites text and colours rarely**: the four labels are *attached* to the drone (the window moves them for free) and rewritten at most twice a second, one at a time; rotor discs and the glow use a few fixed levels; traffic updates at 60 Hz; the physics step runs every third frame. The loop now runs at about **110 frames per second**, using about **2.6 ms** of each frame.

#### 3.8 The live graphs

Each frame the loop hands the drone's state to a `Dashboard` object, which stores one sample ten times a second and one event for every command (accepted, refused or not recognised). A small web server (Python standard library, `127.0.0.1` only) hands them to a page that draws on canvases and asks for new data five times a second:

```
 sample = { t, x, y, z, horizontal speed, vertical speed, heading, gap to nearest zone, state label }
 event  = { t, kind (command | rejected), what was said, what the drone made of it }
```

The page shows altitude, speed, **the gap to the nearest obstacle with the 0.4 m stop band and the 2 m detection band** (so braking and holding are visible), position against the geofence, a top-down map with the trail coloured by speed, and a command timeline with a red cross where a phrase was not recognised. Storing a sample takes 0.03 – 0.05 ms, which is 0.4 – 0.5 ms for each second of flight.

<p align="center">
  <img src="https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/Drone_pybullet/pybullet_voice_drone/screenshots/dashboard.png" alt="The live graphs after a scripted flight: altitude, speed, distance to the nearest obstacle, position, map and command log" width="85%"><br>
  <sub>The graphs after a scripted flight: take off, forward (held 0.4 m in front of the first tower), an unrecognised phrase, back 3 m, turn, left 2 m, up 1 m, go home.</sub>
</p>

### 4. What it can do

| Capability | How it is done | Measured result |
|---|---|---|
| Understand imperfect speech | Top-3 guesses, joins, spelling repair, whole-word rules | Parser covered by 16 unit tests; the first guess that is a command wins |
| Fly an exact distance or angle | Measured moves with a fading speed | **3 m** in 3.25 s, error **1.6 cm**; **90°** in 2.9 s, error **0.13°** |
| Take off and land | Vertical speed fades near 2.0 m and near the pad | Take-off in 2.78 s |
| Stop before an obstacle | Ray test and braking law | Holds **0.40 m** outside the safety zone at `x = 0.83`; peak deceleration 3.0 m/s²; safety net used **0** times |
| Never enter a safety zone | Same, tried from a random start in the plaza, a random heading and a random direction, 20 s each | **190** randomised runs, **0** zone entries, 27 of them held at an obstacle |
| Go around instead (optional) | Left/right choice, strafe, resume | 9.83 s, closest approach **0.70 m** |
| Return home and land | Measured go-home, then landing | Open ground: 10.05 s, **9.6 cm** from the pad centre; through the towers: 13.1 s, **9.5 cm** |
| Accept the keyboard at any time | Held keys steer; releasing them hovers; a voice command carries on while no key is held | Covered by unit tests |
| Show its state | Labels, four cameras, floor marker, sensor ray, trail, safety zones | The state colour tells you why it stopped |
| Show live graphs and log every flight | Browser dashboard and a CSV file | 0.4 – 0.5 ms of loop time per second of flight |

### 5. The world it flies in

The scene is a paved plaza with a helipad, tree planters, benches and lamp posts, surrounded by a ring road with **9 cars** and crosswalks, **10 pedestrians** on the sidewalk, and a skyline of textured buildings. The buildings are **taller on the side away from the sun**, so none of them casts a shadow on the plaza. The obstacles are three kiosk towers whose collision boxes are *exactly* the boxes the obstacle logic uses, and each grows a red halo as the drone nears. One sun at about 52° elevation and a 2048-pixel shadow map make height easy to judge.

The textures and meshes are generated in pure Python (the PNG writer, the textures tiled every 2 m, the building facades with a repeating 2.5 m × 3.2 m window bay): nothing is downloaded. The drone is one PyBullet multibody loaded from `drone.urdf`: landing skids, a camera gimbal, a GPS puck, red/green/white navigation lights (the white one strobes) and four counter-rotating propellers with translucent rotor discs that appear as they spin up. Because it is a single multibody, one call moves every part at the same instant.

<table align="center">
  <tr>
    <td align="center"><img src="https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/Drone_pybullet/pybullet_voice_drone/screenshots/city_obstacle_stop.png" alt="The drone held in front of the first tower, which glows red" width="100%"><br><sub>Held in front of a tower (it glows red)</sub></td>
    <td align="center"><img src="https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/Drone_pybullet/pybullet_voice_drone/screenshots/city_chase.png" alt="The chase camera, behind the drone" width="100%"><br><sub>The chase camera</sub></td>
    <td align="center"><img src="https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/Drone_pybullet/pybullet_voice_drone/screenshots/city_top_view.png" alt="The top camera, like a mission map" width="100%"><br><sub>The top camera, like a mission map</sub></td>
  </tr>
</table>

Press `C` (or say "camera") to cycle **follow** (keeps whatever angle and zoom you set with the mouse), **chase** (behind the drone, turning with it), **top** (straight down) and **fixed**.

### 6. What is and is not claimed

**Shown, with measurements** (the flight-model numbers come from the real controller run headless at 60 steps per second with fixed random seeds, and a second run gave identical values): exact moves and turns (centimetre and fraction-of-a-degree errors), stopping 0.40 m before a safety zone with a braking law that reproduces the measured stop position, 190 randomised runs with no safety-zone entry, return home within 10 cm, a main loop of about 110 frames per second, and 145 headless unit tests that need no window, microphone or internet.

**Not done (stated openly):**
- The flight model is **kinematic**: there is no rotor thrust, attitude control, drag or wind.
- **Speech recognition needs internet**, and everything the microphone hears, including background talk, is sent to Google's service. In a noisy room it also returns phrases that are not commands; the drone then ignores them and the log shows them as not recognised.
- The **obstacle set is fixed in code** (three kiosk towers). There is no map building, and the obstacles do not move.
- The drone knows the obstacles' exact positions (**perfect sensing**); there is no sensor model or estimator.
- The speed of the window depends on the graphics hardware (all the timings above are from one laptop with integrated graphics). `--scene simple` is provided for slow machines.
- Everything is **simulation**; there was no hardware flight.

### 7. Where to read more

| If you want… | Read |
|---|---|
| The full report, all measurements and how to run it | [`Drone_pybullet` README](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/Drone_pybullet#readme) |
| The short usage guide, the command list and every option | [`pybullet_voice_drone/README.md`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/blob/Drone_pybullet/pybullet_voice_drone/README.md) |
| The code | `pybullet_voice_drone/drone_core.py` (parser, flight model, obstacle logic, voice, main loop), `scene_city.py`, `scene_assets.py`, `dashboard.py`, `dashboard.html` |
| To fly it yourself | In `pybullet_voice_drone/`, run `python obstacles.py`, then say or type *take off* (add `--console` to type) |
| To check it | `python -m unittest -v` in the same folder (145 tests, no window or microphone needed) |

**References:** E. Coumans and Y. Bai, *PyBullet, a Python module for physics simulation for games, robotics and machine learning*, 2016 – 2021. A. Zhang, *SpeechRecognition* (Python library), used with the Google Web Speech API. T. L. Kay and J. T. Kajiya, *Ray tracing complex scenes*, SIGGRAPH 1986 (the slab method for the ray test). J. W. Ratcliff and D. E. Metzener, *Pattern matching: the gestalt approach*, Dr. Dobb's Journal, 1988 (the similarity ratio behind Python's `difflib`, used for the spelling repair).

---

<a id="model-2"></a>
## Model 2 · `drone_mujoco` — A Voice-Controlled Drone Whose Controller Is Built From Learned Physics

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

It is fast and has exactly computable stability, but it cannot represent products of states. So it works **near hover** (R² 0.9926) and degrades to **R² 0.70** away from it. It is reported as valid near hover only, and it serves as a fair baseline for SINDy.

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
| Every idea explained from zero, in plain language | [`PROJECT_EXPLAINED.md`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/blob/drone_mujoco/PROJECT_EXPLAINED.md) |
| The controller and sensors in depth | [`CONTROLLER_AND_IMU_EXPLAINED.md`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/blob/drone_mujoco/CONTROLLER_AND_IMU_EXPLAINED.md) |
| The video | [`campus_demo.mp4`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/blob/drone_mujoco/docs/recordings/campus_demo.mp4) |
| To fly it yourself | Double-click `RUN.bat` on the branch, press **1**, then speak or type |

**Main reference:** E. Fresk and G. Nikolakopoulos, *Full Quaternion Based Attitude Control for a Quadrotor*, ECC 2013. SINDy: Brunton, Proctor, Kutz, PNAS 2016. DMDc: Proctor, Brunton, Kutz, SIADS 2016.

---
<a id="model-3"></a>
## Model 3 · `drone_gazebo`
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
11. [Left out, abandoned](#11-left-out-abandoned-or-not-finished)
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

## 11. Left out, abandoned

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

<a id="model-4"></a>
## Model 4 · MATLAB Drone Flight Lab — A Voice-Controlled Drone Simulation in a 3D City

**In one sentence:** we built a MATLAB simulation in which a user can issue a small set of flight commands through buttons or voice input, watch the drone move through a city-like environment, stop movement when the configured obstacle check detects an unsafe region, and inspect the recorded flight through analysis plots.

---

### 1. The problem, in plain words

A quadrotor does not have a steering wheel. It changes direction and altitude by changing the forces and torques generated by its rotors. In a real drone, a flight controller repeatedly reads the state of the vehicle and changes its control inputs. In a simulation, the same broad idea can be represented by updating a virtual drone's state and visualizing its response.

A command such as **TAKE OFF** requests an increase in altitude. **TURN LEFT**, **TURN RIGHT**, and **BACKWARD** request directional movement. **HOVER** asks the drone to hold position, while **LAND** and **RETURN TO HOME** request the corresponding end-of-flight actions.

A command is only a request. It is important to distinguish the requested target from the actual simulated state: a drone can take time to respond, may be blocked by an obstacle check, or may not reach its target before the simulation ends. That difference is why logging and plots matter.

There are three practical challenges:

- **Command interpretation:** convert a button press or recognized voice command into a flight action.
- **Safe movement:** stop the simulated drone if the configured obstacle-safety check identifies that the requested movement would enter a protected obstacle region.
- **Verification:** record what happened and compare the actual flight state with the requested target instead of relying only on the visual animation.

**Our approach** is to combine the command interface, 3D visualization, a simplified obstacle-stop check, and telemetry-based analysis in one MATLAB simulation.

### 2. The idea, in five steps

| Step | What happens | In plain words |
|---|---|---|
| **A. Command** | The user selects a supported button or invokes voice input. | Tell the simulated drone what to do. |
| **B. Interpret** | The command is processed by the simulation. | Turn the request into a flight action or target. |
| **C. Check** | The requested movement is checked against configured obstacle regions. | Stop rather than continue into a detected protected region. |
| **D. Simulate** | The drone state and 3D scene are updated. | Watch the requested action play out in the virtual city. |
| **E. Analyse** | State values are logged and plotted. | Check the trajectory, tracking, errors, and motion after the run. |

The intended obstacle response is a **stop**, not an automatic route around the obstacle. The results depend on the source code, the configured obstacle geometry, and the telemetry that was recorded during the run.

The five steps form a single workflow: input creates a request, the simulation checks and processes the request, the virtual state changes, and recorded data helps us assess what happened. A visual animation is useful for demonstration, while the numerical data is needed for a more careful evaluation.

### 3. The mathematics, step by step

This section explains the quantities used to analyse the flight. These are standard state and error definitions; they should not be read as proof that the simulation uses a particular advanced controller or physical model.

#### 3.1 Position and target

The drone's position can be represented by a three-dimensional vector:

```text
p(t) = [ x(t), y(t), z(t) ]ᵀ
```

Here:

- `x(t)` is the first horizontal coordinate.
- `y(t)` is the second horizontal coordinate.
- `z(t)` is altitude under the simulation's coordinate convention.

The requested target position is

```text
p_target(t) = [ x_target(t), y_target(t), z_target(t) ]ᵀ
```

The actual position describes where the simulated drone is at a particular sample. The target position describes where the command or target logic is asking it to go. They are not interchangeable, and plotting them separately is what makes tracking performance visible.

The exact meaning of the horizontal axes and the positive direction of each command should be checked against the coordinate convention used in the MATLAB source.

#### 3.2 Position tracking error

To see how far the drone is from its requested target, define the error as

```text
e(t) = p_target(t) − p_actual(t)
```

Per axis:

```text
e_x = x_target − x_actual
e_y = y_target − y_actual
e_z = z_target − z_actual
```

An error close to zero means that the actual position is close to the target on that axis. A positive or negative component indicates the direction of the difference under this definition.

The three-dimensional error magnitude is

```text
e_3D = sqrt(e_x² + e_y² + e_z²)
```

Because it is a magnitude, `e_3D` must be nonnegative. If a graph of 3D error dips below zero, the plotted variable is not the magnitude defined above.

A useful summary of tracking error can include:

- **Maximum error:** the largest 3D error during the run.
- **Mean error:** the average 3D error across the recorded samples.
- **Root mean square error (RMSE):** a measure that gives larger errors more influence.

For samples \(e_1,\ldots,e_N\), the RMSE of the 3D error magnitude can be written as

```text
RMSE_3D = sqrt( (1/N) · Σ e_3D(k)² )
```

These measures should only be reported after calculating them from actual telemetry. A plot by itself does not establish a numerical RMSE.

![Axis Wise and 3D Tracking Error](https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/Drones_Matlab/DroneFlightLab_MATLAB/outputs/axis_wise_and_3d_tracking_error.png)

#### 3.3 Velocity and total speed

Velocity describes the rate of change of position:

```text
v(t) = [ v_x(t), v_y(t), v_z(t) ]ᵀ
```

The magnitude of the velocity is the total speed:

```text
speed = sqrt(v_x² + v_y² + v_z²)
```

Individual velocity components may be negative when the drone moves in the negative direction along an axis. Total speed cannot be negative.

A simple discrete-time position relationship is

```text
p(k+1) ≈ p(k) + v(k) · Δt
```

where `Δt` is the simulation time step. This is a general relationship for interpreting sampled position and velocity; the exact state-update method must be confirmed from the code.

Velocity helps distinguish between two situations that may look similar in a screenshot: a drone passing through a target position at speed, and a drone arriving and settling near the target. To evaluate a hover or landing action, inspect both position and velocity rather than position alone.

![Velocity Components and Total Speed](https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/Drones_Matlab/DroneFlightLab_MATLAB/outputs/velocity_components_and_total_speed.png)

#### 3.4 Actual versus target plots

The position-tracking output plots actual and target values against time. It is normally split into three panels:

- East or first horizontal coordinate.
- North or second horizontal coordinate.
- Altitude.

The actual curve should respond to command changes and, when the simulated controller is functioning as intended, move toward the target. Delays and transitions may occur during takeoff, directional movement, return-to-home, or landing.

If actual and target curves overlap, that suggests small tracking error at the scale of the graph. It does not automatically prove perfect tracking: axes may hide small differences, and the logged data and calculation should still be checked.

![East Position Actual vs Target](https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/Drones_Matlab/DroneFlightLab_MATLAB/outputs/east_position_actual_vs_target.png)

![North Position Actual vs Target](https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/Drones_Matlab/DroneFlightLab_MATLAB/outputs/north_position_actual_vs_target.png)

![Altitude Actual vs Target](https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/Drones_Matlab/DroneFlightLab_MATLAB/outputs/altitude_actual_vs_target.png)

#### 3.5 Attitude and motor outputs

If the simulation logs them, attitude plots can display roll, pitch, and yaw. These angles describe the simulated orientation according to the source's convention. Motor-output plots show the values recorded by the simulation; they should not be interpreted as calibrated physical motor speeds or thrust unless a validated relationship is implemented.

A sudden attitude change may coincide with a command transition or correction, but it should be interpreted alongside position, velocity, and the command timeline. No numerical attitude limit or motor-performance claim should be made unless it is defined and verified in the actual code.

![Drone Attitude Roll Pitch Yaw](https://raw.githubusercontent.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/Drones_Matlab/DroneFlightLab_MATLAB/outputs/drone_attitude_roll_pitch_yaw.png)

#### 3.6 Why data alignment matters

The time vector and each telemetry array must refer to the same samples. If `logT` contains 100 samples but `logPos` contains 95, plotting them against each other will fail or produce misleading results. The position and target arrays should also have the expected number of columns, usually three for the three coordinates.

Before calculating a metric, check:

- The arrays are populated, not empty.
- The time and state arrays have compatible row counts.
- The values use the same coordinate system and units.
- The target and actual arrays are aligned in time.
- Any missing or invalid samples are handled deliberately.

### 4. What it can do

| Capability | How it is used | What to verify |
|---|---|---|
| Take off | `TAKE OFF` command | Altitude increases from the initial state. |
| Turn left | `TURN LEFT` command | Position changes in the intended direction. |
| Turn right | `TURN RIGHT` command | Position changes in the intended direction. |
| Move backward | `BACKWARD` command | Movement follows the code's coordinate convention. |
| Hover | `HOVER` command | The drone attempts to hold or settle near the requested position. |
| Land | `LAND` command | Altitude decreases toward the landing level. |
| Return home | `RETURN TO HOME` command | The requested target or movement heads toward the configured home position. |
| Voice input | `LISTEN TO VOICE` listener trigger and the Windows helper | A supported spoken command is recognized and passed to the simulation. |
| Obstacle stop | Configured obstacle-safety check | The simulated movement stops when the check detects an unsafe request. |
| Flight analysis | Telemetry and plotting code | Graphs are generated from populated, correctly aligned data. |

`LISTEN TO VOICE` is a listener trigger, not an additional flight command. Voice recognition depends on the microphone, Windows permissions, speech-recognition setup, and the helper script.

#### 4.1 Command behaviour in more detail

- **TAKE OFF:** the simulation requests an increase in altitude. Check the altitude curve and the 3D scene to verify the response.
- **TURN LEFT / TURN RIGHT:** these request horizontal movement according to the source's direction convention. Do not infer compass heading from the button label alone.
- **BACKWARD:** requests movement opposite the defined forward direction. The actual coordinate change depends on how forward and backward are mapped in the source.
- **HOVER:** requests a hold at the current or configured target, depending on the implementation. Position and speed plots help determine whether the drone settles.
- **LAND:** requests descent to the landing level. Check whether the simulation stops the descent at the intended floor or ground height.
- **RETURN TO HOME:** requests movement toward the configured home point. Confirm the home coordinates and the target updates in the source.
- **LISTEN TO VOICE:** starts or invokes the voice-listening flow; the recognized speech should be one of the supported flight commands.

### 5. The world it flies in

The simulation presents a three-dimensional, city-like environment containing buildings and configured obstacle regions. This scene gives the user a visual reference for the drone's movement and helps demonstrate the obstacle-stop behavior.

The screenshot of the simulation is intentionally not embedded in this Markdown file. If you want to show it in a repository later, add the image separately and insert a relative image link at this point.

The environment and safety check should use consistent coordinates. A visual building and a numerical obstacle region are not necessarily the same thing unless both are configured and checked by the simulation code.

#### 5.1 Visual environment versus collision geometry

The scene shown on screen is a visualization. The obstacle check uses the geometry and rules implemented in the code. To interpret a stop event correctly, the visible obstacle and the checked obstacle region need to correspond to each other.

If the visual building has a different footprint from the configured safety region, the drone may appear to stop too early or too late relative to the rendered building. This should be checked by inspecting the geometry definitions rather than by relying only on the screenshot.

#### 5.2 The meaning of an obstacle stop

The obstacle-stop behavior is deliberately different from obstacle avoidance:

- **Stop behaviour:** detect a movement that violates the configured safety condition and stop the simulated motion.
- **Avoidance behaviour:** calculate a different direction or route and continue moving around the obstacle.

This project description covers the first behaviour. It should not be described as A*, LiDAR-based planning, automatic detouring, or complete 3D collision avoidance unless those functions are actually implemented in the submitted source.

### 6. What is and is not claimed

**The project is intended to demonstrate:**

- A MATLAB-based 3D drone simulation.
- A limited set of manual flight commands.
- A Windows PowerShell voice-input helper where the local setup supports it.
- Simulated movement with a configured obstacle-stop response.
- Flight telemetry and plots for examining trajectory, target tracking, error, velocity, attitude, obstacle clearance, and motor outputs when those data are available.

**The following should not be claimed without additional source code or test evidence:**

- A validated real-world drone flight controller.
- Certified or guaranteed collision avoidance.
- Automatic obstacle detouring or global path planning.
- A particular advanced control method, such as SINDy-based model discovery, quaternion backstepping, or a Kalman filter, unless it is actually implemented in the submitted MATLAB source.
- Successful physical-drone hardware tests.
- Numerical performance or safety metrics that have not been calculated from recorded data.

#### 6.1 Reading the output graphs correctly

The recommended analysis outputs are:

1. **3D trajectory:** the path through the three position coordinates.
2. **Position tracking:** actual versus target position for each axis.
3. **Axis-wise and 3D target error:** the difference between target and actual position.
4. **Velocity and total speed:** motion along each axis and the speed magnitude.
5. **Drone attitude:** roll, pitch, and yaw, if logged.
6. **Obstacle clearance:** an estimate based on the available obstacle geometry and position data.
7. **Motor outputs:** the simulated values, if logged.

These plots are useful only if they use actual data from the simulation. A blank plot or an error such as `Unrecognized function or variable 'logT'` means the plotting code cannot access the required variable. It does not mean the flight data was successfully plotted.

#### 6.2 Telemetry variables

The graph scripts discussed for this project refer to variables such as:

| Variable | Intended content |
|---|---|
| `logT` | Simulation timestamps |
| `logPos` | Actual position samples |
| `logTarget` | Target position samples |
| `logVel` | Velocity samples |
| `logAtt` | Attitude samples |
| `logMot` | Simulated motor-output samples |

Variable availability depends on the version of the main MATLAB function and where the arrays are defined. If these variables are local to `DroneFlightLab_2026`, a separate script in the base workspace will not automatically see them. The plotting code must be run in the owning function's workspace or the populated data must be exported after logging.

A workspace check can be made with:

```matlab
whos logT logPos logTarget logVel logAtt logMot
```

Do not create empty arrays merely to silence an undefined-variable error: the graphs need the real recorded samples.

#### 6.3 Suggested output interpretation table

| Output | What it helps answer | Basic correctness check |
|---|---|---|
| 3D trajectory | Where did the drone travel? | The path agrees with the executed commands and recorded positions. |
| Actual vs target | Did the simulated drone follow its target? | Time samples align and both curves use the same units. |
| 3D error | How far was the drone from the target? | Error magnitude is nonnegative and matches the position differences. |
| Velocity and speed | How fast and in which direction did it move? | Speed is nonnegative and matches the velocity components. |
| Attitude | How did the orientation change? | Values correspond to the recorded attitude convention and command timing. |
| Obstacle clearance | How close was it to configured obstacle regions? | Geometry, units, and safety margin match the source. |
| Motor outputs | What simulated motor values were recorded? | Values come from the run, not manually created placeholders. |

### 7. Where to read more

| If you want… | Open or check |
|---|---|
| Main simulation and controls | `DroneFlightLab_2026.m` (or the exact main `.m` file in the submitted version) |
| Voice-input helper | `listen_voice.ps1` |
| The key output graph scripts | The graph `.m` file included in your project folder |
| MATLAB function lookup | Run `which DroneFlightLab_2026 -all` in the MATLAB Command Window |
| Check whether telemetry exists | Run `whos logT logPos logTarget logVel logAtt logMot` after a simulation run |
| MATLAB plotting documentation | https://www.mathworks.com/help/matlab/creating_plots/ |
| MATLAB language documentation | https://www.mathworks.com/help/matlab/ |

**Main project reference:** the submitted MATLAB source and its associated helper files. Any implementation-specific behaviour, numerical results, or safety claims should be confirmed against those files and measured simulation output.

#### 7.1 Quick troubleshooting guide

| Symptom | Likely reason | What to check |
|---|---|---|
| MATLAB cannot find the main function | Wrong current folder or filename | Use `cd(...)` and `which DroneFlightLab_2026 -all`. |
| Old buttons appear | MATLAB resolved an older copy | Check all paths, then run `clear functions`, `close all force`, and `rehash`. |
| `logT` or `logPos` is undefined | Telemetry is local to a function or has not been recorded | Run the simulation, inspect the workspace, and export the populated arrays or plot inside the function. |
| Graph is blank | Empty arrays or no recorded samples | Check array sizes and confirm that a flight was run. |
| Plot errors due to vector lengths | Time and telemetry samples do not align | Compare `numel(logT)` and the number of rows in each data array. |
| Voice command is not recognized | Microphone, permissions, or listener setup issue | Test microphone access and inspect the PowerShell / MATLAB error output. |
| Drone stops near an obstacle | Safety check may have flagged the protected region | Inspect obstacle geometry and the configured safety margin; do not assume the stop is a bug without checking. |

**Main reference:** the submitted MATLAB source and its associated helper files. Any implementation-specific behaviour, numerical results, or safety claims should be confirmed against those files and measured simulation output.
