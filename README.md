<p align="center">
  <img src="amrita_logo.png" alt="Amrita Vishwa Vidyapeetham" width="80%">
</p>

<h1 align="center">Voice-Controlled Quadcopter Simulation with Obstacle Handling</h1>

<p align="center">
  <b>Speak a command, watch the drone fly: a PyBullet city simulation with a speech-driven command parser</b><br>
  Amrita Vishwa Vidyapeetham
</p>

---

## Faculty Supervisor

| Role | Name | Email |
|---|---|---|
| Faculty Supervisor | Prof. S. Sunil Kumar | s_sunilkumar@cb.amrita.edu |

## Team Members

| S. No. | Name | Roll Number | Email |
|---:|---|---|---|
| 1 | Mamidala Som Praneeth Babu | CB.SC.U4AIE24029 | cb.sc.u4aie24029@cb.students.amrita.edu |
| 2 | C. Karthik | CB.SC.U4AIE24107 | cb.sc.u4aie24107@cb.students.amrita.edu |
| 3 | Ch. Virinchi | CB.SC.U4AIE24158 | cb.sc.u4aie24158@cb.students.amrita.edu |
| 4 | K. Jatin Chandra Gupta | CB.SC.U4AIE24162 | cb.sc.u4aie24162@cb.students.amrita.edu |

This README describes the `Drone_pybullet` branch. All of its code is in [`pybullet_voice_drone/`](pybullet_voice_drone/); the short usage guide is [`pybullet_voice_drone/README.md`](pybullet_voice_drone/README.md). Other attempts at the same drone are listed in [Section 23](#23-other-work-in-this-repository).

---

## Abstract

This project is a voice-controlled quadcopter simulator built with **PyBullet** and the **SpeechRecognition** library. A speech phrase such as *"forward three meters"* or *"go home"* is transcribed by Google's speech service, turned into a command by a small natural-language parser, and executed by a flight model that moves a drone through a generated city plaza. The drone brakes smoothly and stops in front of obstacles, can fly exactly the distance or angle it is asked for, can return to its take-off pad on its own, and records every flight to a CSV file. A live dashboard opens in the browser with the flight's graphs (altitude, speed, distance to the nearest obstacle, position, a top-down map and a command timeline). The keyboard works at any moment as a manual override. The simulation is kinematic (commanded velocities with a limited acceleration, not simulated rotor thrust), which keeps the focus on the voice interface, command logic and visualisation. The whole system, including the city, textures and traffic, is generated in Python with no downloaded assets, and is covered by 145 headless unit tests.

---

## Contents

- [Abstract](#abstract)
1. [Introduction](#1-introduction)
2. [Problem Statement](#2-problem-statement)
3. [Tools and Background](#3-tools-and-background)
4. [Methodology](#4-methodology)
5. [Results](#5-results)
6. [Parameters](#6-parameters)
7. [System Architecture](#7-system-architecture)
8. [Project Structure](#8-project-structure)
9. [Prerequisites & Installation](#9-prerequisites--installation)
10. [How to Run](#10-how-to-run)
11. [Commands](#11-commands)
12. [Telemetry Format](#12-telemetry-format)
13. [Running Tests](#13-running-tests)
14. [How the Project Evolved](#14-how-the-project-evolved)
15. [Key Observations](#15-key-observations)
16. [Limitations](#16-limitations)
17. [Known Issues and Troubleshooting](#17-known-issues-and-troubleshooting)
18. [Left Out, Abandoned or Not Finished](#18-left-out-abandoned-or-not-finished)
19. [Future Work](#19-future-work)
20. [What You Can Do With It](#20-what-you-can-do-with-it)
21. [Conclusion](#21-conclusion)
22. [References](#22-references)
23. [Other Work in This Repository](#23-other-work-in-this-repository)

---

## 1. Introduction

A voice-controlled drone has to solve three problems at once: understand an imperfect, noisy transcription of speech; turn that into safe, predictable motion; and show the operator clearly what the drone is doing and why.

**Why speech is hard to use directly.** Speech-to-text engines return the most likely transcript, not the intended command. Accents, background noise and short words produce transcripts such as *"for this"* instead of *"forward"*. A usable voice drone therefore cannot match exact strings; it needs ranked guesses, tolerance for common mishearings and a way to show the operator what was heard.

**Why a kinematic simulator.** The purpose here is the command and safety layer that sits in front of any flight controller. Simulating rotor thrust and attitude control is the job of the dynamics-based simulators on the other branches of this repository. A kinematic model gives deterministic, testable behaviour (a 3 m move really ends 3 m away), which is what makes the voice, obstacle and return-home logic verifiable.

**Why a city.** Height, distance and speed are hard to judge in an empty scene. A paved plaza with a helipad, shadows, traffic and buildings gives the operator real depth cues, and gives the obstacle logic something realistic to react to.

---

## 2. Problem Statement

Build and simulate a voice-controlled quadcopter that:

- Understands spoken commands in natural wording, including distances and angles (*"rotate left ninety degrees"*), despite imperfect transcription.
- Accepts keyboard control at any time without conflicting with a running voice command.
- Flies with smooth, bounded acceleration and has well-defined flight states (on the ground, taking off, flying, landing).
- Detects obstacles ahead and stops safely in front of them, instead of relying on the operator.
- Can fly a measured distance or turn, and can return to the take-off point and land.
- Logs every command and the flight state to a CSV file.
- Draws live graphs of the flight (altitude, speed, distance to obstacles, position, map, commands) while it runs.
- Shows its state clearly (camera modes, labels, sensor ray, trail) while keeping its main loop fast on ordinary laptop graphics.
- Is covered by automated tests that need no window, microphone or internet.

---

## 3. Tools and Background

| Tool | Role |
|---|---|
| **PyBullet** (Coumans & Bai) | 3D world, rendering, camera, debug drawing |
| **SpeechRecognition** (A. Zhang) + Google Web Speech API | Microphone capture and speech-to-text with ranked alternative transcripts |
| **PyAudio** | Microphone access |
| Python standard library (`difflib`, `threading`, `queue`, `csv`, `unittest`) | Spelling repair, concurrency, logging, tests |
| Python standard library (`http.server`, `webbrowser`) + a page of plain HTML/JavaScript | The live graphs: a local web server and the page that draws the charts (no plotting library, no internet) |

The drone model (`drone.urdf`), the textures and the meshes are written for this project. Textures and meshes are generated by `scene_assets.py`; nothing is downloaded.

---

## 4. Methodology

The system is organised as five layers, each testable on its own without a window.

```
speech / typed text → command parser → flight controller → drone + scene (PyBullet) → camera, labels, overlays
```

The same pipeline as a diagram, with the two side outputs (the CSV log and the live graphs):

```mermaid
flowchart LR
    MIC["Microphone"] --> SP["Speech service<br/>up to 3 ranked guesses"]
    SP --> PA["Command parser<br/>whole words, joins, spelling repair,<br/>distance and angle"]
    TXT["Typed text<br/>(--console)"] --> PA
    KEY["Keyboard"] --> FC
    PA --> FC["Flight controller<br/>states, measured moves, return home,<br/>obstacle check and braking"]
    FC --> SIM["Drone and city scene<br/>in PyBullet"]
    SIM --> VIEW["Cameras, labels,<br/>trail, sensor ray"]
    FC --> CSV["drone_data.csv"]
    FC --> GR["Live graphs<br/>in the browser"]
```

---

### 4.1 From Speech to a Command

```
microphone → speech service → up to 3 ranked guesses → first guess that is a command
          → words → join split words → repair slips → (intent, distance/angle) → controller
```

1. **Ranked guesses.** Google returns several alternative transcripts. The first of the top three that parses to a command is used, so a transcript whose best guess is *"for this"* but whose second guess is *"forward"* still works.
2. **Language.** The service is queried in Indian English (`en-IN`) by default; `--language` changes it.
3. **Split words are joined.** Recognisers often split one word in two, so pairs such as *"for word"*, *"four ward"* and *"back word"* become *forward* and *backward*.
4. **One-letter slips are repaired.** A word of five or more letters is replaced by the closest command word if their similarity ratio is at least 0.85:

```math
\text{ratio}(a,b) = \frac{2\,M}{|a| + |b|}
```

where $M$ is the number of matching characters. *"forwad"* (ratio 0.92) becomes *forward*, while *"island"* (ratio 0.62 to *landing*) is left alone.

> **📘 What this means:**
> The parser matches whole words, never substrings, so *"island"* cannot trigger *land*. The strict 0.85 cutoff means ordinary conversation is not turned into commands: unrelated speech is simply reported as "not recognised".

5. **Distances and angles.** The first number in the phrase, written as digits or words (`3`, `three`, `twenty five`, `2.5`), becomes the amount: metres by default or feet, degrees for turns. Distances are capped at 50 m and turns at 720°.
6. **Listening does not stop while transcribing.** One thread only records; a second thread sends each recording to the speech service. A command spoken right after another is therefore not lost while the first is being transcribed.

---

### 4.2 Kinematic Flight Model

A command sets a *target* body-frame velocity. The drone's actual world-frame velocity follows that target with a limited acceleration $a = 3\ \mathrm{m/s^2}$:

```math
\begin{bmatrix} v_x \\ v_y \end{bmatrix}_{cmd} = \begin{bmatrix} \cos\psi & -\sin\psi \\ \sin\psi & \cos\psi \end{bmatrix} \begin{bmatrix} v_{fwd} \\ v_{left} \end{bmatrix}
```

```math
v \leftarrow v + \mathrm{clip}\big(v_{cmd} - v,\ -a\,\Delta t,\ a\,\Delta t\big), \qquad x \leftarrow x + v\,\Delta t
```

The heading $\psi$ follows the commanded yaw rate with a yaw-acceleration limit of $180^\circ/\mathrm{s}^2$. Position is clamped to a $\pm 12$ m geofence and 0.5 – 8 m altitude.

> **📘 What this means:**
> "Forward" always means the way the red nose points, whatever the heading. The drone never jumps to a new speed; it speeds up and slows down smoothly, which also makes stopping distances predictable.

---

### 4.3 Flight States

| State | Entered by | Behaviour |
|---|---|---|
| `LANDED` | start, landing complete | Only *take off* and *up* are accepted; rotors still |
| `TAKEOFF` | *take off* | Climbs to 2 m, slowing near the target, then hovers. Never snaps a higher drone back down |
| `FLYING` | take-off complete, or any motion command | Obeys commands; the obstacle logic is active |
| `LANDING` | *land*, or the end of *go home* | Descends to the pad (0.31 m body height: the skids rest on the pad) |

A manual command during take-off or landing cancels the automatic manoeuvre and returns to `FLYING`.

---

### 4.4 Measured Manoeuvres and Return Home

*"Forward 3 meters"*, *"rotate left 90 degrees"* and *"go home"* are *manoeuvres*. The speed (or turn rate) shrinks as the goal gets close, so the drone arrives smoothly:

```math
v = \mathrm{clip}\big(2\,d_{remaining},\ 0.15,\ v_{max}\big), \qquad \omega = \mathrm{clip}\big(2\,\theta_{remaining},\ 0.15,\ \omega_{max}\big)
```

Return home uses $v = \mathrm{clip}(1.5 d,\ 0.15,\ 1.5)$ towards the pad and then lands. A manoeuvre ends when it completes, when it is cancelled (*stop*, or any new command), when the drone is pinned (speed below 0.05 m/s for 1.5 s, for example against the geofence), when an obstacle stops it, or after a timeout of $3d/v + 10$ s.

---

### 4.5 Obstacle Handling

**The safety zone.** Each obstacle box is grown by the drone's footprint (propeller-tip radius $R = 1.32$ m, vertical half-height 0.30 m) plus a safety distance $s = 0.25$ m. A ray is cast every frame along the commanded direction of travel, up to 2 m ahead, and tested against these zones with the slab method; the distance to the first zone entered is $t_{hit}$.

**Stopping (default).** The drone keeps its speed until it is close enough that a gentle deceleration $a_b = 2\ \mathrm{m/s^2}$ can still stop it, and then slows to halt a fixed distance $d_{stop} = 0.4$ m before the zone:

```math
v_{allowed} = \sqrt{2\,a_b\,\max(t_{hit} - d_{stop},\ 0)}
```

Once held, its command is dropped so it cannot set off again by itself (for example when it is turned to face open space). Any direction that is clear works immediately, and moving away from the obstacle always works.

> **📘 What this means:**
> The drone starts braking early enough to stop smoothly, never later. Holding still and waiting for a new instruction is the safe behaviour: the operator decides how to continue.

**Steering (optional, `--on-obstacle steer`).** The drone computes how far it would have to shift left and right to clear every obstacle within a 4 m planning horizon and takes the cheaper free side; if both sides are blocked it climbs over, and if it cannot, it stops. It resumes the original command as soon as the ray is clear.

**Safety net.** Independently of the above, any motion that would put the drone inside an obstacle is rejected one axis at a time, so it can never pass through one.

---

### 4.6 The Scene

| Part | How it is made |
|---|---|
| Ground layers | Grass, ring road, sidewalks and plaza are flat textured quads, each 1 cm above the one below to avoid flicker |
| Textures | Generated in pure Python and tiled every 2 m; building facades repeat a $2.5\ \mathrm{m} \times 3.2\ \mathrm{m}$ window bay |
| Buildings | Textured walls plus roofs and rooftop details; **taller on the side away from the sun**, so no building casts a shadow on the plaza |
| Traffic | 9 cars on two rounded-square lanes (opposite directions) and 10 pedestrians on the sidewalk, moved along the same path function |
| Obstacles | Kiosk towers whose collision boxes are exactly the boxes the obstacle logic uses, with a red halo that grows as the drone nears |
| Lighting | One sun at about 52° elevation with a 2048-pixel shadow map |

The drone is one PyBullet multibody loaded from `drone.urdf` (landing skids, camera gimbal, GPS puck, navigation lights, four counter-rotating propellers and translucent rotor discs), so a single call moves all of it at the same instant.

---

### 4.7 Cameras and On-screen Information

Four cameras, cycled with `C` (or the word *"camera"*): **follow** (keeps the angle and zoom set with the mouse), **chase** (behind the drone, turning with it), **top** (straight down, like a mission map) and **fixed**. For the chase camera, PyBullet's camera yaw equals the drone's heading minus 90°, a relation measured directly from PyBullet's own view matrices.

Four labels attached to the drone show its state (colour-coded), position, speed and heading, the distance to the nearest obstacle, and the last phrase heard. A floor marker with a thin line shows height, a sensor ray shows what is ahead (green to red), and a flight trail is coloured by speed.

---

### 4.8 Keeping It Smooth: Measuring the GUI

An early version managed only about 11 frames per second. The cost of each PyBullet call was measured in the real window with the full city loaded:

| Call | Cost |
|---|---|
| Move a body / set a rotor angle | 0.1 / 0.04 ms |
| Change a colour (`changeVisualShape`) | 0.1 – 0.9 ms |
| Rewrite a text label | about 3 ms when idle, **4 ms or more while the scene moves** |
| One physics step | 2.2 ms (nothing here is dynamic) |

Seven labels re-pinned to the screen every frame would cost about 30 ms, so they are instead **attached to the drone** (the window moves them for free) and rewritten only when their text changes, at most twice a second, one at a time. Rotor discs use three fixed opacity levels, the glow uses coarse steps, traffic moves at 60 Hz, and the physics step runs every third frame.

---

### 4.9 Telemetry

Every command, and one sample per second while airborne, is appended to a CSV file (`Time, Voice_Input, Command`, world-frame velocity, position and yaw), so a flight can be replayed or analysed afterwards.

---

### 4.10 Live Graphs

The CSV file is for afterwards. To see the flight **while it happens**, the simulation also serves a dashboard on the same computer and opens it in the browser (`dashboard.py` + `dashboard.html`). Every frame the main loop hands the drone's state to a `Dashboard` object, which stores one sample ten times a second (position, speed, heading, flight state and the gap to the nearest obstacle's safety zone) plus an event for every command: accepted, refused by the flight state, or not recognised. A small `http.server` thread on `127.0.0.1` answers the page's requests; the page asks five times a second for the samples it has not seen yet and draws them on canvases. Nothing leaves the computer.

| Graph | Shows |
|---|---|
| Status tiles | State, altitude, speed, heading, position, nearest obstacle, commands given |
| Altitude | Height with the take-off height and minimum flight height marked |
| Speed | Horizontal and vertical speed against the limit |
| Distance to the nearest obstacle | The gap to the safety zone, with the 0.4 m stopping band and the 2 m detection range, so braking and holding are visible |
| Position | x and y against the geofence |
| Map | Top view of the geofence, towers, their safety zones, the helipad and the trail coloured by speed |
| Timeline and command log | The flight state over time with a marker for each command; a red cross for a phrase that was not recognised or a command that was refused |

The window can be set to 30 s, 1 min, 5 min or the whole flight, the page can be paused, and the samples can be downloaded as a CSV. After the simulation window is closed the page keeps the last data. Storing a sample takes 0.03 - 0.05 ms and a frame that only checks the clock under 0.001 ms; in total recording costs 0.4 - 0.5 ms per second of flight (two headless runs with the real obstacle-gap function), so it does not change the loop speed of section 5.4. Options: `--no-graphs`, `--no-browser`, `--graphs-port PORT`.

![Live graphs after a scripted flight: take off, forward (held at the first tower), an unrecognised phrase, back 3 m, turn, left 2 m, up 1 m, go home](pybullet_voice_drone/screenshots/dashboard.png)

*The dashboard after a scripted flight (commands typed in the console): the drone takes off, is held 0.4 m in front of the first tower, ignores a phrase it does not know ("banana"), flies back 3 m, turns, moves left 2 m, climbs 1 m and returns home to land.*

---

## 5. Results

### 5.1 The Simulation

| City plaza overview | Held in front of an obstacle (the tower glows) |
| :---: | :---: |
| ![City overview](pybullet_voice_drone/screenshots/city_overview.png) | ![Obstacle stop](pybullet_voice_drone/screenshots/city_obstacle_stop.png) |
| **Chase camera** | **Top camera** |
| ![Chase camera](pybullet_voice_drone/screenshots/city_chase.png) | ![Top camera](pybullet_voice_drone/screenshots/city_top_view.png) |

These are frames from the window's own renderer. The labels, sensor ray, floor line and trail are drawn by the window and do not appear in such captures.

---

### 5.2 Measured Behaviour of the Flight Model

Measured by running the controller headless at 60 steps per second.

| Behaviour | Command | Result |
|---|---|---|
| Take-off | *take off* | Reaches 2.000 m in **2.78 s** |
| Measured move | *forward 3 meters* | Done in 3.25 s; **1.6 cm** along-track error, 0 lateral error |
| Measured turn | *rotate left 90 degrees* | Done in 2.9 s; **0.13°** heading error |
| Stop before an obstacle | *forward* towards the first tower | Halts at $x = 0.83$ m, **0.40 m** from the safety zone (the designed stop distance); peak deceleration $3.0\ \mathrm{m/s^2}$, equal to the acceleration limit; the hard safety net was never needed |
| Steer around obstacles | *forward* through all three towers (`steer`) | Reaches the far wall in 9.8 s; closest approach to any safety zone **0.70 m**; safety net never needed |
| Return home, open space | *go home* from 9.4 m away | Lands in 10.1 s, **9.6 cm** from the pad centre |
| Return home, through the towers (`steer`) | *go home* from the far wall, 13.0 m away | Lands in 13.1 s, **9.5 cm** from the pad centre |

### 5.3 Randomised Safety Check

190 random runs: random start position in the plaza, random heading, and a random one of *forward / backward / left / right*, each flown for 20 s with the default stopping behaviour (30 further random starts were discarded because they began inside a safety zone).

| Result | Count |
|---|---:|
| Runs | 190 |
| Times the drone entered a safety zone | **0** |
| Times the hard safety net was needed | **0** |
| Runs that ended held at an obstacle | 27 |

### 5.4 Loop Performance

Measured in the real window with the full city, traffic, labels, chase camera and overlays running, on integrated Intel UHD graphics:

| | Frames in 6 s | Loop rate | Work per frame |
|---|---:|---:|---|
| First version (labels re-pinned every frame) | 68 – 86 | about 11 – 14 Hz | about 70 – 88 ms |
| Current | **670** | **about 112 Hz** | **2.6 ms** of an 8.3 ms budget |

This measures the Python loop, not the GPU's own frame rate, which depends on the machine.

---

## 6. Parameters

| Parameter | Value | Description |
|---|---:|---|
| Horizontal / vertical speed | 1.5 / 1.0 m/s | Cruise speeds |
| Turn rate | 45 °/s | Yaw rate |
| Acceleration | $3\ \mathrm{m/s^2}$ | How fast velocity follows the command |
| Take-off height | 2.0 m | Cruise altitude after *take off* |
| Minimum flight height | 0.5 m | *down* stops here; *land* goes lower |
| Maximum height / geofence | 8 m / $\pm 12$ m | World limits |
| Drone footprint radius | 1.32 m | Propeller tip to centre |
| Safety distance | 0.25 m | Added around every obstacle |
| Detection distance | 2.0 m | How far ahead the ray looks |
| Stop distance / braking | 0.4 m / $2\ \mathrm{m/s^2}$ | Where the drone halts, and how gently |
| Speech guesses considered | 3 | Top Google alternatives |
| Spelling-repair similarity | 0.85 | Minimum ratio, words of 5+ letters |

---

## 7. System Architecture

```
Windows host (one Python process)
│
├── Main thread: the loop (~112 Hz)
│     ├── KeyboardInput        ← held keys steer, one-shot keys (T, L, H, C ...)
│     ├── DroneController      ← commands → smooth motion, flight states, obstacle logic
│     ├── DroneModel           ← pose, lean, rotors, lights → PyBullet
│     ├── World / CityScene    ← kiosk halos, traffic, pedestrians
│     ├── CameraRig, Hud, Overlays
│     ├── TelemetryLogger      ← CSV
│     └── Dashboard            ← samples + command events (in memory)
│
├── VoiceListener thread       ← microphone → recordings
│     └── transcription worker ← speech service → best of 3 guesses → command queue
│
├── ConsoleListener thread     ← typed commands (--console) → the same queue
│
└── Dashboard server thread    ← 127.0.0.1 only: serves dashboard.html and /api/data to the browser

PyBullet GUI window            ← draws everything (shadows, textures, labels)
```

---

## 8. Project Structure

```
Drones_S5_AB8_ESP32VoiceDrone/
│
├── README.md                        # this file
├── README_ESP32_SIMULATOR.md        # documentation of the original ESP32 simulator (kept on branch drone_esp32_sim)
├── drone_sim/                       # code of the original simulator (a copy; the original is on branch drone_esp32_sim)
│
└── pybullet_voice_drone/            # this branch's project
    ├── drone_core.py                # parser, flight model, obstacle logic, drone model, cameras, HUD, voice, telemetry, main loop
    ├── scene_city.py                # the city: plaza, roads, buildings, trees, kiosks, helipad, traffic, pedestrians
    ├── scene_assets.py              # generates textures and meshes
    ├── dashboard.py                 # live graphs: records the flight, serves it to the browser (127.0.0.1 only)
    ├── dashboard.html               # the graphs page (canvas charts, no libraries)
    ├── main.py                      # launcher: keyboard mode
    ├── Voice_Drone_PyBullet.py      # launcher: voice mode
    ├── obstacles.py                 # launcher: voice + obstacles (the full demo)
    ├── drone.urdf                   # the drone model
    ├── test_drone_core.py           # 145 headless unit tests
    ├── assets/                      # generated textures
    ├── screenshots/                 # pictures used in the READMEs
    ├── output/                      # results table, screenshots, logs and the scripts that produced them
    ├── legacy/                      # the first versions of the scripts and screenshots
    └── requirements.txt
```

---

## 9. Prerequisites & Installation

| Component | Requirement |
|---|---|
| Python | 3.9 – 3.12 |
| Packages | `pybullet`, `SpeechRecognition`, `PyAudio` (see `requirements.txt`) |
| For voice | A microphone and an internet connection (the speech service is online) |
| Without voice | Keyboard mode and `--console` work offline |

```powershell
cd pybullet_voice_drone
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

On Windows, `pip install PyAudio` normally works because wheels exist for Python 3.9 – 3.12. On Linux install PortAudio first (`sudo apt install portaudio19-dev`).

---

## 10. How to Run

| Command | Voice | Obstacles |
|---|:---:|:---:|
| `python obstacles.py` | on | on (the full demo) |
| `python Voice_Drone_PyBullet.py` | on | off |
| `python main.py` | off | off |

The drone starts on the helipad: say *"take off"* (or press `T`) first. Useful options:

| Option | Meaning |
|---|---|
| `--on-obstacle stop\|steer` | Stop in front of obstacles (default) or fly around them |
| `--language CODE` | Speech language: `en-IN` (default), `en-US`, `en-GB`, ... |
| `--scene city\|simple` | Full city (default) or a light plain scene for slow machines |
| `--camera follow\|chase\|top\|fixed` | Starting camera |
| `--console` | Also accept commands typed into the terminal |
| `--no-voice`, `--no-log`, `--no-hud`, `--hud-size S`, `--zones`, `--no-trail` | Switch features off or tune them |
| `--no-graphs`, `--no-browser`, `--graphs-port PORT` | Live graphs: switch them off, serve them without opening the browser, choose the port (default 8765) |

### First flight

A complete first run. Typing the commands works the same as speaking them, so start with typing and switch to the microphone once it works:

```powershell
cd pybullet_voice_drone
venv\Scripts\activate
python obstacles.py --console
```

Two windows open: the PyBullet window with the city, and the live graphs in the browser (the address, `http://127.0.0.1:8765/`, is also printed in the console). Click the PyBullet window once if you want to use the keyboard. Then say or type, one at a time:

| Step | Say or type | What you should see |
|---|---|---|
| 1 | `take off` | The rotors spin up and the drone climbs to 2 m. The console prints `>> TAKE OFF`, then `>> TAKEOFF COMPLETE - HOVERING`. |
| 2 | `forward` | The drone flies toward the first tower and holds 0.40 m before its safety zone. The tower glows red and the label turns red. The console prints `OBSTACLE DETECTED: Obstacle_1` and `STOPPED: Obstacle_1 is in the way. Give another direction to move on.` |
| 3 | `back 3 meters` | It flies exactly 3 m back and hovers (`>> MOVE COMPLETE`). |
| 4 | `rotate left 90 degrees` | It turns 90 degrees and hovers (`>> TURN COMPLETE`). |
| 5 | `left 2 meters` | It moves 2 m to its left. |
| 6 | `go home` | It flies back over the pad and lands (`>> RETURN HOME COMPLETE`, then `>> LANDING COMPLETE`). |
| 7 | `exit` | The window closes. The flight is in `drone_data.csv`, and the graphs page keeps the last data. |

This is the flight recorded in [`pybullet_voice_drone/output/logs/sample_flight_console.txt`](pybullet_voice_drone/output/logs/sample_flight_console.txt). If something does not work, see [Section 17](#17-known-issues-and-troubleshooting).

---

## 11. Commands

### Voice (or typed with `--console`)

| Say | Action |
|---|---|
| *take off*, *land*, *stop*, *exit* | Basic flight control |
| *forward*, *backward*, *left*, *right*, *up*, *down* | Move until *stop* |
| *forward 3 meters*, *up ten feet*, *back 4m* | Move exactly that far, then hover |
| *rotate left*, *turn right 90 degrees* | Turn (continuously, or by an angle) |
| *go home* | Fly back over the take-off point and land |
| *camera*, *trail*, *zones* | Cycle the camera, toggle the trail, toggle the safety zones |

### Keyboard (click the window first)

| Key | Action |
|---|---|
| `W` `S` `A` `D` | Forward, backward, left, right (hold) |
| `Q` `E` | Up, down (hold) |
| `←` `→` | Rotate (hold) |
| `T` `L` `H` | Take off, land, go home |
| `Space` | Stop and hover |
| `C` `P` `Z` | Camera, trail, safety zones |
| `[` `]` | Smaller / bigger labels |
| `X` | Exit |

---

## 12. Telemetry Format

```csv
Time,Voice_Input,Command,Velocity_X,Velocity_Y,Velocity_Z,Drone_X,Drone_Y,Drone_Z,Yaw
2026-10-21 20:54:44,take off,TAKE OFF,0.000,0.000,0.000,0.000,0.000,0.310,0.0
2026-10-21 20:54:45,,TAKE OFF,0.000,0.000,1.000,0.000,0.000,1.152,0.0
2026-10-21 20:54:47,,HOVER,0.000,0.000,0.000,0.000,0.000,2.000,0.0
```

Velocities are in m/s (world frame), positions in metres and yaw in degrees. `Voice_Input` is empty on the periodic rows.

---

## 13. Running Tests

```powershell
cd pybullet_voice_drone
python -m unittest -v
```

The 145 tests need no window, microphone or internet.

| Area | Tests |
|---|---:|
| Command parser, distances and angles, mishearings | 16 |
| Flight states | 12 |
| Measured moves and return home | 12 |
| Stopping before obstacles | 14 |
| Steering around obstacles | 7 |
| Obstacle sensor | 3 |
| Voice thread (with a stand-in recogniser) | 9 |
| Keyboard | 5 |
| Labels | 13 |
| Cameras | 8 |
| City, lane paths, texture and mesh generators | 16 |
| Drone model, visual effects, scene | 16 |
| Telemetry | 2 |
| Live graphs (recording, what is sent, limits, local-only server) | 12 |

Examples of what is checked: the drone never enters a safety zone and never exceeds its braking limit; cars stay on the road and pedestrians never walk through a lamp post; no building's shadow reaches the plaza; the obstacle towers keep exactly the collision boxes the obstacle logic uses; and a slow speech service cannot stop the microphone loop from hearing the next phrase.

---

## 14. How the Project Evolved

The code on this branch is the result of several rewrites. The earlier scripts are kept in [`pybullet_voice_drone/legacy/`](pybullet_voice_drone/legacy/).

| Stage | Where | What it was |
|---|---|---|
| 1. First scripts | `legacy/Voice_Drone_PyBullet_original.py` (208 lines), `legacy/main_original.py` (356 lines) | A drone built from boxes and cylinders, with voice and keyboard control as two separate scripts. Each command moved the drone one fixed step (0.8 m, 0.5 m up, 20 degrees) by setting its position directly. A phrase was matched by looking for a command word anywhere in the text (`"land" in command`), so *island* would have landed it. |
| 2. Bug-fixed voice script | `legacy/Voice_Drone_PyBullet_FIXED.py` (300 lines) | The same idea after a round of fixes: smaller steps (0.6 m, 0.4 m up, 15 degrees) and rewritten drone-building and command functions. |
| 3. Obstacles | `legacy/obstacles_original.py` (1,250 lines) | Obstacle boxes, detection ahead and an automatic way around, together with the voice thread in one script. |
| 4. Obstacles, fixed | `legacy/Voice_Drone_PyBullet_FIXED_obstacles_v1.py` (1,983 lines) | The obstacle script after fixes, with a check that a position is safe, a safe-velocity calculation and an explicit stop. |
| 5. One core module | commit `1b56094` | Everything merged into `drone_core.py` (1,591 lines) with three small launchers and 86 headless tests: flight states, smooth acceleration, measured moves, return home, stopping before obstacles (with an optional steer-around mode), whole-word parsing and telemetry. |
| 6. City and smarter voice | commit `8a56906` | The generated city plaza (`scene_city.py`, `scene_assets.py`), four cameras, labels attached to the drone, Indian-English recognition with the top three guesses, joining of split words, spelling repair and a listener that keeps hearing while a phrase is being transcribed. 133 tests. |
| 7. Live graphs and results | commits `2b238aa` to `751a5d4` | The browser dashboard (`dashboard.py`, `dashboard.html`), 12 more tests (145 in all) and the [`output/`](pybullet_voice_drone/output/) folder with the measured results, screenshots, logs and the scripts that produced them. |

---

## 15. Key Observations

1. Ranking matters more than accuracy: taking the first of Google's top three guesses that is a command recovers phrases whose best guess is wrong.
2. A keyboard override must not fight a voice command: held keys steer, and with no key activity a running voice command carries on.
3. Dropping the command when the drone is held at an obstacle prevents a "ghost" restart when it is later turned to face open space.
4. In PyBullet's GUI, anything that rewrites text or colours is slow while the scene moves, but moving bodies is cheap. Designing around those measured costs raised the loop from about 11 Hz to about 112 Hz.
5. PyBullet's OpenGL renderer draws a whole link in one colour, so every differently coloured part of the drone is its own link.
6. Wide, very thin discs a few millimetres above the floor flicker; thin boxes do not.
7. Large buildings must be kept low on the sunny side, or their shadows cover the area the drone flies in.

---

## 16. Limitations

- The flight model is kinematic: it does not simulate rotor thrust, attitude control, drag or wind.
- Speech recognition needs an internet connection, and everything the microphone hears, background talk included, is sent to Google's service.
- The obstacle set is fixed in code (three kiosk towers); there is no map building.
- The speed of the PyBullet window itself depends on the graphics hardware; `--scene simple` is provided for slow machines.
- The size of the labels depends on how PyBullet scales text on a given machine, so it is adjustable (`[` and `]`, or `--hud-size`).

---

## 17. Known Issues and Troubleshooting

Problems met while building the project, and what to do about them on another computer. The first table lists what was fixed and how. The second lists what you may still run into.

### Found and fixed

| Problem | Cause | What fixed it |
|---|---|---|
| A phrase containing a command word inside another word (*island*) triggered the command | The first scripts looked for the command word anywhere in the text (`"land" in command`) | Whole-word matching ([Section 4.1](#41-from-speech-to-a-command)) |
| A command spoken right after another was lost | Recording and transcribing ran in one thread, so the microphone was deaf while the speech service answered | One thread only records and a second one transcribes ([Section 4.1](#41-from-speech-to-a-command)) |
| The window ran at about 11 to 14 frames per second | Every label was re-pinned and rewritten each frame, and a text or colour change costs several milliseconds in PyBullet's window | Labels attached to the drone and rewritten rarely: about 112 frames per second ([Section 4.8](#48-keeping-it-smooth-measuring-the-gui), [Section 5.4](#54-loop-performance)) |
| The drone restarted by itself after being held at an obstacle | The held command stayed active, so turning the drone toward open space resumed it | The command is dropped when the drone is held ([Section 15](#15-key-observations)) |
| Flicker on the floor | Wide, very thin discs a few millimetres above the floor | Thin boxes instead ([Section 15](#15-key-observations)) |
| Building shadows covered the area the drone flies in | Tall buildings on the sunny side | Buildings are taller on the side away from the sun ([Section 4.6](#46-the-scene)) |

### What you may still run into

| What you see | Cause | What to do |
|---|---|---|
| `ERROR: could not connect to the PyBullet GUI. Close any old PyBullet windows and retry.` | Usually an old PyBullet window is still open | Close old PyBullet windows (or end the leftover Python process) and start again |
| `Voice control unavailable: SpeechRecognition is not installed` | The package is missing | `pip install -r requirements.txt` |
| `Voice control unavailable: ... Keyboard control still works` | No microphone, or PyAudio is not installed | Install PyAudio ([Section 9](#9-prerequisites--installation)), or fly with the keyboard or `--console` |
| `Speech service unreachable (needs internet)` | Google's speech service needs an internet connection | Reconnect, or type commands with `--console`. The message repeats at most every 10 seconds |
| `COMMAND NOT RECOGNIZED` for a phrase you meant | Background noise or an accent the service misheard | Check the `heard` label above the drone, say the command word on its own, try `--language en-US` or `en-GB`, or type with `--console` |
| `Still transcribing earlier speech - dropped a recording.` | Six recordings were already waiting for the speech service | Pause between phrases and check the connection |
| The keys do nothing | The PyBullet window does not have the keyboard focus | Click the window first |
| `Live graphs unavailable (...)` | None of the ports 8765 to 8774 could be opened | `--graphs-port PORT` with a free port, or `--no-graphs` |
| `Telemetry logging disabled (...)` | `drone_data.csv` cannot be written, for example because another program has it open | Close that program, or use `--telemetry-file PATH` or `--no-log` |
| `No obstacles in this mode (try obstacles.py or --obstacles).` | Z was pressed in a mode that has no obstacles | Start with `python obstacles.py`, or add `--obstacles` |
| The window is slow | Weak graphics hardware, and the full city has many objects | `--scene simple`, or `--no-hud` |
| The labels are too large or too small | PyBullet scales text differently on different machines | `[` and `]` while flying, or `--hud-size` |
| The first start takes longer | Textures and meshes are generated on the first run (`assets/mesh/` is not tracked) | Wait once; later starts reuse them |
| `python -m unittest` cannot find `drone_core` | It was started from the wrong folder | `cd pybullet_voice_drone` first |

---

## 18. Left Out, Abandoned or Not Finished

Listed so that nobody assumes these exist.

- **Rotor thrust, attitude control, drag and wind.** The flight model is kinematic on purpose. The dynamics-based versions are on the `drone_mujoco` and `drone_gazebo` branches ([Section 23](#23-other-work-in-this-repository)).
- **A screen-fixed on-screen display.** PyBullet can only draw text at a 3D position, and re-pinning text to the screen every frame costs several milliseconds while the scene moves ([Section 4.8](#48-keeping-it-smooth-measuring-the-gui)). The labels are attached to the drone instead.
- **A smooth fade of the rotor discs.** Every colour change is a call into the window, so the discs use three opacity levels.
- **Sensing and mapping.** The drone knows the exact positions of the obstacles. There is no sensor model, no map building and there are no moving obstacles.
- **Offline speech recognition.** Speech goes to Google's service, so it needs internet and sends whatever the microphone hears. The `drone_esp32_sim` branch (the original ESP32 simulator) uses an offline recogniser (Vosk).
- **Commands from the graphs page.** The dashboard only shows data, on purpose. It accepts no commands.
- **Named waypoints, stored missions and failsafes.** Planned ([Section 19](#19-future-work)), not built.
- **Hardware.** Everything is simulation. No real drone flew.

---

## 19. Future Work

- Waypoint navigation by name (*"go to point A"*) and a stored mission (HOME → A → B → C → HOME), as in the team's plan for the ESP32 drone.
- Failsafes: return home on geofence breach, low battery or lost connection.
- Replace the kinematic model with a thrust-and-attitude model so the same voice and obstacle layers drive a PID-controlled drone, and compare it with the MuJoCo and Gazebo branches.
- Random obstacle layouts and moving obstacles.
- Offline speech recognition for use without internet.

---

## 20. What You Can Do With It

- **Move or add obstacles.** Edit `DEFAULT_OBSTACLES` in `drone_core.py`. Each entry is a name, a centre `(x, y, z)` and half-sizes `(hx, hy, hz)` in metres, for example `Obstacle("Obstacle_4", (-4.0, 3.0, 1.5), (0.7, 0.7, 1.5))`. The city scene builds a kiosk tower for each entry. Keep them inside the geofence (`WORLD_LIMIT`, 12 m). Every safety zone grows by 1.57 m around its box (the drone's 1.32 m radius plus 0.25 m of clearance), so two obstacles closer than about 3.1 m face to face leave no gap the drone may use.
- **Add or change command words.** `_WORD_INTENTS` maps each spoken word to an intent, so a new synonym is one line (`"ascend"` already means *up*). A new kind of action needs a new `Intent` and a branch in `apply_intent`. Run the tests afterwards.
- **Change how the drone flies.** The constants at the top of `drone_core.py` set the speeds, acceleration, take-off and ceiling heights, the braking distance and the detection range ([Section 6](#6-parameters)). Keep `BRAKE_DECEL` below `ACCELERATION`, so that the planned stop is always achievable.
- **Go around instead of stopping.** Start with `--on-obstacle steer`.
- **Use another accent or language.** Start with `--language en-US`, `en-GB` and so on.
- **Analyse a flight.** Open `drone_data.csv` ([Section 12](#12-telemetry-format)) or press Download CSV on the graphs page. The scripts in [`pybullet_voice_drone/output/results/scripts/`](pybullet_voice_drone/output/results/scripts/) repeat the measurements.
- **Run it on a slow computer.** Use `--scene simple` and `--no-hud`.
- **Extend the tests.** Add cases to `test_drone_core.py`. They run headless, so they need no window, microphone or internet.

---

## 21. Conclusion

The project shows that a voice interface for a drone depends as much on forgiving command handling as on the recogniser: ranked guesses, split-word joining, spelling repair and strict whole-word matching turn imperfect transcripts into reliable commands without turning conversation into commands. Behind the interface, a kinematic flight model with smooth acceleration, measured manoeuvres, return-home and a braking-based obstacle stop gives deterministic behaviour that is verified by measurement (a 3 m move ends 1.6 cm from its target, the drone holds exactly 0.40 m from an obstacle's safety zone, and 190 randomised runs never entered one) and by 145 automated tests. A generated city, four cameras and on-screen labels make the behaviour easy to see, and measuring the real cost of each GUI call kept the whole system smooth.

---

## 22. References

Every link was checked on 2026-10-24.

1. E. Coumans and Y. Bai, "PyBullet, a Python module for physics simulation for games, robotics and machine learning," 2016–2021. [Online]. Available: <https://pybullet.org>
2. A. Zhang, "SpeechRecognition: Library for performing speech recognition, with support for several engines and APIs" (Python package). [Online]. Available: <https://github.com/Uberi/speech_recognition>
3. Google, Web Speech API speech-to-text service, accessed through `SpeechRecognition.Recognizer.recognize_google` (language `en-IN`, up to three ranked alternatives).
4. J. W. Ratcliff and D. E. Metzener, "Pattern matching: The gestalt approach," *Dr. Dobb's Journal*, vol. 13, no. 7, p. 46, Jul. 1988. (The similarity ratio used by Python's `difflib`.)
5. Python Software Foundation, "difflib: Helpers for computing deltas," Python 3 standard library documentation. [Online]. Available: <https://docs.python.org/3/library/difflib.html>
6. T. L. Kay and J. T. Kajiya, "Ray tracing complex scenes," *ACM SIGGRAPH Comput. Graph.*, vol. 20, no. 4, pp. 269–278, 1986. doi: [10.1145/15886.15916](https://doi.org/10.1145/15886.15916)
7. R. Mahony, V. Kumar, and P. Corke, "Multirotor aerial vehicles: Modeling, estimation, and control of quadrotor," *IEEE Robot. Autom. Mag.*, vol. 19, no. 3, pp. 20–32, 2012. doi: [10.1109/MRA.2012.2206474](https://doi.org/10.1109/MRA.2012.2206474)
8. Open Robotics, "URDF XML specification," ROS Wiki. [Online]. Available: <https://wiki.ros.org/urdf/XML>
9. Python Software Foundation, "http.server: HTTP servers," Python 3 standard library documentation. [Online]. Available: <https://docs.python.org/3/library/http.server.html>

---

## 23. Other Work in This Repository

The repository holds several attempts at the same voice-controlled drone, each on its own branch:

- **[`drone_esp32_sim`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/drone_esp32_sim)** is the original ESP32-based attempt, moved there from `main`. Its documentation (Vosk voice parsing, cascaded PID control, GPS waypoints, SINDy/DMDc analysis) is kept in this branch as [`README_ESP32_SIMULATOR.md`](README_ESP32_SIMULATOR.md), and its code is in [`drone_sim/`](drone_sim/). [`main`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/main) now holds only the overview README of all the models.
- **[`drone_mujoco`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/drone_mujoco)** is a MuJoCo-based voice-controlled quadrotor with a SINDy-identified data-driven attitude controller (full pipeline: excitation → identification → validation → cascaded control → live voice demo).
- **[`drone_gazebo`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/drone_gazebo)** is a PX4 + Gazebo voice-controlled quadrotor, also using SINDy/DMDc identification.
- **[`Drones_Matlab`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/Drones_Matlab)** is a MATLAB Drone Flight Lab: a command interface with voice input, an obstacle stop and telemetry analysis in a 3D city.
