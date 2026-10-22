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
## Model 3 · `drone_gazebo` — A Voice-Controlled Drone with a Controller Built from Learned Physics

*Oct 22, 2026 · @Jatin*

**In one sentence:** a computer discovers the equation that governs how a drone rotates by watching it fly, the controller is calculated from that equation, and a person can say "go to the central library" and the drone flies there across a scale model of the campus.

---
### 1. The Problem, in Plain Words

A quadrotor has four propellers and no steering wheel. The only things we can change are four numbers: how hard each propeller pushes. Tilting, turning, moving forward and holding still all come from choosing those four numbers hundreds of times per second.

A controller is the rule that picks them, and to design one well you need a model: an equation that predicts what the drone will do for a given push. The usual approach writes the model from textbook physics and measured parameters (mass, inertia, drag). That has two weaknesses:

- the textbook parameters are never exactly right for the real machine, and
- the controller then depends on numbers nobody checked against the machine's actual behaviour.

Our approach is the reverse. Record how the drone actually behaves, let an algorithm find the equation, check that equation hard, and only then build the controller from it. This is data-driven control. On top of it sits the part a person sees: speaking to the drone.

### 2. The Idea, in Five Steps

| Step | What we do | In plain words |
| --- | --- | --- |
| A. Excite | Push the four propellers with random and sweeping signals, with no controller, and record everything. | Shake the drone in every way, so the recording contains the physics. |
| B. Identify | Fit **SINDy**. | Ask a computer: what short equation explains this motion? |
| C. Validate | Test the equation on flights it has never seen, including 11-second blind predictions. | Try to break it. Keep it only if it survives. |
| D. Control | Compute the controller's gains from the learned equation. | The equation sets the settings, not trial and error. |
| E. Command | Add speech recognition, command and place matching, route planning and the campus map. | Say a place, and the drone flies there. |

Each stage had a pass or fail gate, and the next stage did not start until the previous one passed.

### 3. The Mathematics, Step by Step

Every symbol is defined where it first appears.

#### 3.1 What is being modelled

The drone's state is its tilt angles θ and rotation rates ω. The input is the four propeller thrusts u. For a rigid body, physics gives the structure (Euler's rotation equation).

```math
\dot{\theta} \approx \omega, \qquad I\,\dot{\omega} = M u - \omega \times (I\omega) - c\,\omega
```

I is the inertia, M maps propeller thrusts to torques, and c is the drag coefficient. The controller does not use these textbook values. SINDy has to find them from data.

#### 3.2 SINDy: finding a short equation

SINDy stands for Sparse Identification of Nonlinear Dynamics. We record the state, the input and the rate of change of the state at thousands of moments, and stack them.

```math
\dot{X} \approx \Theta(X,U)\,\Xi
```

- Ẋ holds the measured rates of change: one row per moment, one column per state variable (6 columns).
- Θ(X, U) is a library: a table of 26 candidate terms evaluated at every moment (a constant, θ, ω, u, and the products ωᵢωⱼ and θᵢωⱼ).
- Ξ is the unknown 26 by 6 coefficient matrix. That is 156 numbers, and SINDy keeps 44.

The library is chosen with physics in mind. It contains only terms rigid-body dynamics could produce and leaves out terms with no physical reason to exist (θθ, uu, θu, ωu). A blind polynomial would have 66 terms; ours has 26, so the method discovers physics instead of fitting noise.

Why sparse? Ordinary least squares uses every candidate a little and gives a messy answer. SINDy uses STLSQ (sequentially thresholded least squares): fit, drop every coefficient smaller than λ, and refit on the survivors until nothing changes.

```math
\Xi \leftarrow \arg\min_{\Xi} \lVert \dot{X} - \Theta\,\Xi \rVert_2^2, \qquad \Xi_{ij} \leftarrow 0 \;\text{ if }\; |\Xi_{ij}| < \lambda
```

Two practical points matter for correctness:

1. Smooth before differentiating. Ẋ is not measured; it is computed from samples, and differentiating raw data multiplies noise. A Savitzky-Golay filter (a local polynomial fit over a sliding window) is applied first.
2. Never pair samples across a reset. Attitude is held in local coordinates and re-referenced at segment boundaries. A pair that straddles a reset has a fake jump of about 2.6 rad against about 0.002 rad of real motion; squared, one such pair outweighs a million real ones.

Evidence that it found physics, not just a curve: the simulator's true parameters are known, so the learned coefficients can be compared with them. Control effectiveness, drag, the gyroscopic term and the kinematic term all agree within 2 to 6 percent, and a test enforces this.

#### 3.3 How the learned equation is judged

Both scores are measured on held-out trials: separate recordings never shown to the fit, not a slice of one recording. R² is the fraction of behaviour explained, with 1.0 perfect. NRMSE is the error relative to how much the state changes in one step, so a model that does nothing scores exactly 1.0.

```math
R^2 = 1 - \frac{\sum (y-\hat{y})^2}{\sum (y-\bar{y})^2}, \qquad \text{NRMSE} = \frac{\mathrm{RMS}(\hat{y}-y)}{\mathrm{RMS}(\Delta y)}
```

| Test | Result for SINDy |
| --- | --- |
| One-step prediction, held-out | R² 0.9946, NRMSE 0.0786 |
| 11-second blind rollout, full flight envelope | 0 of 7 diverged, mean final error 0.095 (limit 0.5) |
| Coefficients against the simulator's true values | within 2 to 6 percent |
| Noise ablation (add gyro noise) | holds to about 1 rad/s, knee at 2 rad/s, fails by 4 rad/s |
| Sparsity ablation (raise the threshold λ) | stable above 0.2, collapses at 1.0 |
| Head to head with DMDc in DMDc's own regime | SINDy 0.0789 against DMDc 0.0825 |

Eigenvalues: linearising at hover gives six poles, and three sit on the stability boundary. That is the correct answer, not a defect. A free rigid body has no restoring force, so θ̇ = ω gives a marginal pole for each axis. The other three are strictly stable.

#### 3.4 From the learned equation to the controller

The controller is a cascade: a slow outer loop (position) wrapped around a fast inner loop (attitude). The inner loop uses the learned model. Orientation is a quaternion q (four numbers of unit length), which has no gimbal lock and no 359 to 0 degree jumps. The error quaternion has a vector part that is about half the angle error, and the control law is a spring on the angle and a damper on the rate.

```math
q_e = q_{want}\otimes q^{*}, \qquad \tau = -K_q\,q_{e,vec} - K_\omega\,\omega
```

Near hover each axis, as identified by SINDy, behaves like ω̇ = aω + bτ. Here a is negative (it equals minus drag over inertia) and b is the control effectiveness (one over inertia). Substituting the control law gives the closed loop, which can be compared with the standard second-order form.

```math
\ddot{e} + (bK_\omega - a)\,\dot{e} + \tfrac{b K_q}{2}\,e = 0 \quad\text{compare with}\quad s^2 + 2\zeta\omega_n s + \omega_n^2 = 0
```

Matching coefficients fixes the gains from the model.

```math
\omega_n = \frac{4}{\zeta T_s}, \qquad K_q = \frac{2\omega_n^2}{b}, \qquad K_\omega = \frac{2\zeta\omega_n + a}{b}
```

Only two numbers are chosen by hand: the settling time Tₛ = 0.5 s and the damping ratio ζ = 0.7. Everything else comes from the learned a and b. If the model changes, the gains recompute. The tilt settles in 0.56 to 0.86 s, and the learned-model simulation and the full MuJoCo simulation agree to about 0.3 percent.

The outer loop is ordinary physics, deliberately not learned. A drone moves by tilting, which points part of its thrust sideways. Translation under a thrust vector is Newton's law, a plain double integrator with nothing for a data-driven method to discover. So the outer loop is a PD law with a gated integral term.

```math
a_{cmd} = K_p(p_{ref}-p) + K_d(v_{ref}-v) + K_i\!\int (p_{ref}-p)\,dt, \qquad K_p=\omega_n^2,\; K_d=2\zeta\omega_n \;\;(\omega_n=2,\ \zeta=1)
```

The thrust vector is m(a\_cmd + gẑ): its length is the total thrust and its direction is the tilt asked of the inner loop. The integral acts only when both the planned path and the drone are nearly at rest, which removes steady offsets (wind, wrong mass) without adding lag during a move.

Mixer: the four propeller thrusts are recovered from total thrust T and the three torques by inverting the known X-configuration geometry G. If a request would push a propeller below 0 N or above 6 N, yaw torque is sacrificed first, because it is the weakest axis (about five times more costly than roll or pitch), so total thrust is always kept.

```math
\begin{bmatrix}T\\ \tau_x\\ \tau_y\\ \tau_z\end{bmatrix} = G\,u \;\;\Rightarrow\;\; u = G^{-1}\begin{bmatrix}T\\ \tau_x\\ \tau_y\\ \tau_z\end{bmatrix}, \qquad 0 \le u_i \le 6\ \text{N}
```

#### 3.5 From speech to command

This is the part a person sees. A speech recogniser turns the spoken audio into text, and the text is matched to what the drone can do. Recognisers mishear, so the match is fuzzy: every known phrase is scored against what was heard, and the best one is accepted only if it is close enough.

```math
k^{*} = \arg\max_k\, s(\text{heard},\,c_k), \qquad \text{accept only if } s(\text{heard},\,c_{k^{*}}) \ge \tau
```

Here cₖ is the k-th command phrase (eight movement and control commands, plus land) and s is a similarity between 0 and 1. A common choice is twice the number of matching characters divided by the total length of both phrases; τ is the acceptance threshold, and anything below it is ignored rather than guessed.

```math
s(a,b) = \frac{2\,M(a,b)}{|a|+|b|}
```

Places use the same idea over the 31 named buildings, so "go to the central library" becomes a target position. The target is then clipped to the map edge and kept out of building cells, so a mis-heard or repeated command can never send the drone into a building.

#### 3.6 Smooth paths and braking

- A\* finds a route over a grid of the campus that avoids building cells. It always expands the cell with the smallest f, where g is the distance travelled and h the straight-line distance still to go.
- A cubic smoothing spline (a B-spline fit) rounds the route's corners, trading closeness to the points against curvature.
- A quintic Bezier curve (degree 5) is used for each leg so that position, velocity and acceleration all match at the joins. Chained commands therefore flow into each other instead of stopping.

```math
f(n) = g(n) + h(n), \qquad \min_s \sum_i \big(y_i - s(t_i)\big)^2 + \lambda\!\int (s'')^2\,dt
```

Braking follows the kinematic law below. From 1.0 m/s at a deceleration limit of 1.0 m/s² this is 0.50 m, and that is exactly what the simulation measured (0.92 s to rest).

```math
d = \frac{v^2}{2a}
```

### 4. What It Can Do

| Capability | How it is done | Measured result |
| --- | --- | --- |
| Move in 6 directions, hover, land | Fuzzy matching of speech to 8 commands plus land | 24 of 24 scripted voice trials reached the target |
| Fly to a named building | Place matcher, A\*, spline and Bezier | 31 buildings addressable by voice |
| Chain commands | Bezier legs with matched velocity and acceleration | 5 of 5 chained legs, max error 0.022 m |
| Change its mind mid-flight | New command overrides the old one | Reversal in 0.42 s, 2.04 m clear of the tower |
| Handle a tough list of maneuvers | Maneuver gauntlet | 67 of 67 passed |
| Survive disturbances | Yaw-priority mixer and gated integral | Within 0.15 m to 2 rad/s gyro noise, 2 N wind, ±20 percent wrong mass |
| Never fly into a building | Target clipping, map-edge limit | Also works when "forward" is repeated |

### 

### 5. References

- [Project repository, drone\_gazebo branch](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/drone_gazebo): full report, results and how to run it.
- E. Fresk and G. Nikolakopoulos, Full Quaternion Based Attitude Control for a Quadrotor, ECC 2013 (main reference for the controller).
- Brunton, Proctor and Kutz, PNAS 2016 (SINDy).

---

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
