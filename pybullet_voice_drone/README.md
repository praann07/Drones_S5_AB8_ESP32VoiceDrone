# 🎙️ Voice-Controlled Quadcopter Drone in PyBullet 🚁

An interactive 3D quadcopter simulation built with **PyBullet** and **SpeechRecognition**. Fly it by voice, fall back to the keyboard at any moment, let it stop itself before it hits anything, and get a CSV flight log of everything it did.

---

## 📌 Features

- **🎙️ Voice control** – a background thread listens to the microphone (Google Web Speech API) and understands natural phrases: *"take off"*, *"go left"*, *"turn to the left"*, *"move forward"*, *"land"*. Whole words are matched, so *"island"* never lands the drone.
- **⌨️ Keyboard override** – hold keys to steer; release them and the drone hovers. A voice command keeps running while you are not touching the keyboard.
- **🛑 Stops before obstacles** – the drone looks ahead along its direction of travel, brakes smoothly and holds still at a safe distance in front of any obstacle. The pillar blocking it pulses red, the rear lights flash and the status line turns red. Give it another direction (or *"back"*) to move on. `--on-obstacle steer` makes it fly around obstacles instead. A hard collision stop is the last line of defence.
- **📏 Measured commands** – say *"forward 3 meters"*, *"up ten feet"* or *"rotate left 90 degrees"* and the drone moves exactly that far (or until an obstacle stops it), then hovers. Plain *"forward"* keeps going until you say *"stop"*.
- **🏠 Return home** – *"go home"* (or `H`) flies back over the take-off point and lands on the pad (it stops short if an obstacle is in the way).
- **🚁 Flight modes** – the drone starts on the ground and has to take off; it lands on the ground (not hovering half a metre above it); acceleration is smooth, not teleporting; the drone leans into its direction of travel and the rotors only spin while it is flying.
- **🎨 Look** – sky-blue backdrop, a gridded floor with sharp shadows (so height is easy to judge), hazard-striped pillars with beacons that glow as the drone approaches, and a home pad with an H. The drone has landing skids, a camera gimbal, a GPS puck, red/green/white navigation lights (the white one strobes), and translucent rotor discs that fade in as the rotors spin up.
- **🎥 Cameras & overlays** – a follow camera that keeps whatever angle/zoom you set with the mouse, or a fixed observer view (`C` key / `--camera fixed`). A colour-coded status line (red = held at an obstacle, cyan = taking off/landing, green = hovering, yellow = moving) floats above the drone with command, height, speed, heading and the last phrase heard. A marker on the floor under the drone joined to it by a thin line shows its height, a sensor ray shows what is ahead (green clear → red close), a flight trail coloured by speed fades behind it (`P`), and `Z` draws each obstacle's safety zone.
- **🧩 One-piece drone model** – `drone.urdf` loads as a single multibody with four counter-rotating propellers, so the whole drone moves in the same instant (no glitching parts).
- **📊 Telemetry** – every command and one sample per second while airborne are appended to `drone_data.csv`.

> The motion model is kinematic (commanded velocities, not simulated rotor thrust). PyBullet provides the 3D world, rendering and camera.

---

## 📸 Screenshots

| Simulation Overview | Obstacle Avoidance Environment |
| :---: | :---: |
| ![PyBullet Drone](Physics%20Server0.png) | ![Obstacle Simulation](Physics%20Server1.png) |
| ![Flight View](Physics%20Server2.png) | ![Perspective Camera](Physics%20Server3.png) |

---

## 📂 Project Structure

```text
Voice_Drone_PyBullet/
├── drone_core.py                 # Everything: command parser, flight model, obstacle handling, scene, voice, telemetry, main loop
├── main.py                       # Launcher: keyboard mode (voice/obstacles optional)
├── Voice_Drone_PyBullet.py       # Launcher: voice mode
├── obstacles.py                  # Launcher: voice mode + obstacles (the drone stops before them)
├── drone.urdf                    # Drone model (one multibody, 4 spinning propeller joints, lights, rotor discs)
├── assets/ground.png             # Floor texture (regenerated automatically if you delete it)
├── test_drone_core.py            # Headless unit tests
├── drone_data.csv                # Telemetry log (appended on every run)
├── drone_command_data.csv        # Old command log from an earlier version (no longer written)
├── legacy/                       # The original scripts, kept for reference (not maintained)
├── requirements.txt
└── Physics Server*.png           # Screenshots
```

The three launchers only differ in their defaults – all of them accept the same options (see below).

---

## ⚙️ Prerequisites

- **Python 3.9 – 3.12**
- **Microphone + internet** for voice control (not needed for keyboard mode or `--console`)
- **PyAudio** (installed by `requirements.txt`)

---

## 🚀 Installation

```bash
git clone https://github.com/your-username/Voice_Drone_PyBullet.git
cd Voice_Drone_PyBullet
```

**Windows (PowerShell):**
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```

**Linux / macOS:**
```bash
python3 -m venv venv
source venv/bin/activate
```

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

> [!TIP]
> **PyAudio won't install?** Windows has prebuilt wheels for Python 3.9–3.12, so `pip install PyAudio` normally just works. On Linux install PortAudio first (`sudo apt install portaudio19-dev`), on macOS `brew install portaudio`. Without a microphone you can still fly with the keyboard or type commands with `--console`.

---

## 🎮 How to Run

| Command | Voice | Obstacles |
| :--- | :---: | :---: |
| `python Voice_Drone_PyBullet.py` | on | off |
| `python obstacles.py` | on | on |
| `python main.py` | off | off |

Options (all launchers):

| Option | Meaning |
| :--- | :--- |
| `--voice` / `--no-voice` | Turn the microphone on or off |
| `--obstacles` / `--no-obstacles` | Spawn obstacles and make the drone detect them |
| `--on-obstacle stop\|steer` | What the drone does when an obstacle is in its way: `stop` before it (default) or `steer` around it |
| `--console` | Also accept commands typed into the terminal (handy without a microphone) |
| `--camera follow\|fixed` | Starting camera mode |
| `--no-trail` / `--zones` | Start with the flight trail hidden / the obstacle safety zones drawn |
| `--telemetry-file PATH` / `--no-log` | Where to log flight data, or don't |
| `--duration SECONDS` | Exit automatically (useful for smoke tests) |

For example, keyboard flying with obstacles: `python main.py --obstacles`.

The drone **starts on the ground**: say *"take off"* (or press `T`) first.

---

## 🗣️ Voice Commands

Speak (or type with `--console`) one command at a time. Several wordings work for each:

| Say | Action |
| :--- | :--- |
| **take off**, takeoff, lift off, launch | Climb to 2 m and hover (never pulls a higher drone back down) |
| **forward**, go forward, move forward | Fly in the direction the red nose points |
| **backward**, back, go back, reverse | Fly backwards |
| **left**, go left, move to the left | Slide left |
| **right**, go right, move to the right | Slide right |
| **up**, rise, climb, ascend | Climb (also lifts off from the ground) |
| **down**, descend, lower | Descend (stops at 0.5 m; say *land* to go lower) |
| **rotate left**, turn left, turn to the left | Yaw counter-clockwise |
| **rotate right**, turn right, spin right | Yaw clockwise |
| **stop**, halt, freeze, hover, hold | Brake and hover (also cancels a measured move or return home) |
| **go home**, return home, go back home | Fly back over the take-off point and land |
| **land** | Descend to the ground and stop the rotors |
| **camera** | Toggle follow / fixed camera |
| **trail** | Show / hide the flight trail |
| **zones**, safety zones | Show / hide the obstacle safety zones |
| **exit**, quit | Close the simulation |

**Distances and angles:** add a number to any move or rotation. Digits or words both work (`3`, `three`, `twenty five`, `2.5`), metres by default or `feet`; rotations are in degrees.

| Say | Action |
| :--- | :--- |
| *forward 3 meters*, *go left two meters*, *back 4m* | Move exactly that far along the drone's own heading, then hover |
| *up 2 meters*, *down ten feet* | Climb / descend by that amount (*up* also works on the ground as a lift-off) |
| *rotate left 90 degrees*, *turn right one hundred eighty* | Turn on the spot by that angle |

Distances are capped at 50 m and turns at 720°. A measured move gives up (and hovers) if the drone is pinned against the world limit or ceiling, or if it takes far too long to get there.

While the drone is on the ground only *take off* and *up* are accepted.

---

## ⌨️ Keyboard Controls

Click the PyBullet window first. PyBullet's own debug shortcuts (W = wireframe, A, C, G …) are switched off so they don't interfere.

| Key | Action |
| :---: | :--- |
| **`W` / `S`** | Forward / backward (hold) |
| **`A` / `D`** | Left / right (hold) |
| **`Q` / `E`** | Up / down (hold) |
| **`←` / `→`** | Rotate left / right (hold) |
| **`T`** | Take off |
| **`L`** | Land |
| **`H`** | Return home and land |
| **`Space`** | Stop and hover |
| **`C`** | Toggle camera |
| **`P`** | Show / hide flight trail |
| **`Z`** | Show / hide obstacle safety zones |
| **`X`** | Exit |
| *(release all)* | Hover in place |

Keys can be combined (e.g. `W`+`D` flies diagonally).

---

## 🛑 Obstacle Handling

Every frame the drone casts a ray along its commanded direction of travel (so it works for *backward*, *left/right* and *down* too) and tests it against each obstacle grown by the drone's footprint (propeller-tip radius 1.32 m, plus the skids' height) and a 0.25 m **safety distance** – the *safety zone*, drawn with `Z`. It looks up to `DETECTION_DISTANCE = 2 m` ahead.

**`--on-obstacle stop` (default)**
1. **Brake** – the drone keeps its speed until it is close enough that a gentle 2 m/s² deceleration can still stop it, then slows smoothly.
2. **Hold** – it halts `STOP_BUFFER = 0.4 m` in front of the zone and stays there. Its command is dropped, so it cannot take off again on its own (for example when you turn it to face clear space).
3. **Say what happened** – the status line turns red, the nearest pillar pulses, the rear lights flash, and a measured move or *"go home"* ends with *STOPPED - OBSTACLE AHEAD*.
4. **Move on** – any direction that is clear works at once; moving away from the obstacle always works, even from inside the zone.

**`--on-obstacle steer`**
1. **Choose a route** (once per obstacle) – it works out how far it would have to shift left and right to clear *everything* within a 4 m planning horizon and takes the cheaper side, provided that side is free and inside the world. If both sides are blocked it **climbs over** (when the obstacle is low enough); otherwise it stops.
2. **Go around** – forward speed fades as the obstacle gets closer while the drone strafes (or climbs).
3. **Resume** – as soon as the ray is clear (with a small margin) the drone continues with the command you gave.

**Safety net (both modes)** – motion that would put the drone inside an obstacle is cancelled one axis at a time, so it can never pass through one.

The three demo obstacles are defined by `DEFAULT_OBSTACLES` in `drone_core.py`; tuning constants (speeds, distances, colours, light position) sit at the top of the same file.

---

## 📊 Telemetry

`drone_data.csv` is appended to on every run (header written only once): one row per command, plus one per second while airborne. Velocities are world-frame m/s, positions metres, yaw degrees.

```csv
Time,Voice_Input,Command,Velocity_X,Velocity_Y,Velocity_Z,Drone_X,Drone_Y,Drone_Z,Yaw
2026-10-21 20:54:44,take off,TAKE OFF,0.000,0.000,0.000,0.000,0.000,0.310,0.0
2026-10-21 20:54:45,,TAKE OFF,0.000,0.000,1.000,0.000,0.000,1.152,0.0
2026-10-21 20:54:46,,TAKE OFF,0.000,0.000,0.275,0.000,0.000,1.867,0.0
2026-10-21 20:54:47,,HOVER,0.000,0.000,0.000,0.000,0.000,2.000,0.0
```

`Voice_Input` is the recognised (or typed) phrase and is empty on the periodic rows. Rows written by older versions of this project use the same header with other time formats.

---

## 🧪 Tests

The tests run headless – no window, microphone or internet:

```bash
python -m unittest -v
```

They cover the command parser (including distances and angles), flight modes, measured moves and return-home, stopping before obstacles (braking distance, no creeping, no restart after turning, return-home and measured moves being stopped), the optional steer-around mode (including walls it must climb over and dead ends), the visuals (lean, pillar glow, rotor discs, navigation lights, floor marker, status colours, floor texture), the URDF model, keyboard handling, the voice thread (with a stubbed recogniser) and the CSV logger.

---

## 📤 Pushing This Project to GitHub

This folder is already a git repository (nothing is committed yet):

```bash
git add .
git commit -m "Voice-controlled PyBullet drone with obstacle avoidance"
git branch -M main
git remote add origin https://github.com/<your-username>/<your-repo-name>.git
git push -u origin main
```

---

## 📜 License

No license file is included yet. Add a `LICENSE` file (for example MIT) before publishing if you want others to be able to reuse the code.
