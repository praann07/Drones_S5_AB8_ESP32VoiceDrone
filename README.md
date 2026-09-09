> **Branch status:** this branch currently holds the project write-up
> only — the actual code (PX4/Gazebo, `voice_control.py`, `data_logger.py`,
> `fit_sindy.py`, etc.) will be added here separately. Written by Jathin.

---

# Voice-Controlled Autonomous Drone with Data-Driven System Identification

A PX4 + Gazebo quadrotor simulation, flown by voice, with its flight dynamics independently rediscovered from telemetry using SINDy and DMDc -- data-driven system identification methods that don't rely on the drone's own first-principles equations of motion.

---

## 1. What This Project Actually Is

At its core, this project answers a research question, not just an engineering one:

> **If you didn't already know the equations governing a quadrotor's flight, could you recover them purely from flight data?**

The pipeline is split into two halves that mirror each other on purpose:

- **A real flight-control stack** (PX4 SITL + Gazebo) that flies a simulated quadrotor, controllable by voice.
- **A data-driven identification stack** (SINDy, DMDc) that watches the logged flight data and tries to reconstruct the dynamics -- without ever being told the underlying physics.

This mirrors exactly what would need to happen with a **real** drone, where nobody hands you the true equations of motion -- you only have sensor logs. Using PX4's own Gazebo physics engine (rather than a hand-written toy simulator) as the "ground truth" makes this a legitimate identification exercise rather than a circular one: the identification methods are not simply rediscovering equations that were written down by the same person running the experiment.

---

## 2. Data-Driven Point of View

### Why data-driven modeling, and why it matters here

A first-principles model (Newton's laws, motor mixing, rotation matrices) is only as good as the assumptions baked into it. Real hardware has unmodeled effects: motor lag, asymmetric drag, manufacturing tolerances, battery sag. Data-driven system identification exists to **recover a model's structure directly from what a vehicle actually does**, which is closer to how a real engineering team would characterize a new airframe.

Two different methods are used here, deliberately chosen to contrast a linear and a nonlinear approach:

**DMDc (Dynamic Mode Decomposition with control)**
Fits the simplest possible model: `x_{k+1} = A*x_k + B*u_k`, a linear state-space system. Solved via truncated SVD-based least-squares regression over logged `(state, control input, next state)` triples. Its main payoff isn't just prediction -- it's **interpretability**: the eigenvalues of `A` directly reveal the natural modes of the system (damping, oscillation frequency, stability), the same language used to describe real aircraft handling qualities. A quadrotor's attitude dynamics near hover are close to linear, so DMDc is expected to work reasonably well locally and degrade as the vehicle tilts further from level flight.

**SINDy (Sparse Identification of Nonlinear Dynamics)**
Doesn't assume linearity. Instead, it's handed a large *library* of candidate nonlinear terms (`sin(roll)`, `q*r`, `p*|p|`, motor-mixing combinations, etc. -- deliberately chosen to include the exact terms that appear in real rigid-body quadrotor equations) and uses sparse regression (STLSQ: fit, threshold out small coefficients, refit) to discover which handful of terms actually matter. A successful SINDy fit that lands on approximately the right terms and coefficients is a strong, independent check that the simulated dynamics behave the way theory says a quadrotor should.

### The one methodological detail that matters most

DMDc and SINDy both need a clean separation between **state** (`x`, what the vehicle is doing) and **control input** (`u`, what was commanded). Early versions of this project's logger used IMU-derived accelerations as `u` -- that's actually wrong, because accelerations are a *measured response*, not a *commanded input*, which blurs the exact distinction these methods depend on. The corrected approach uses **actual actuator/motor outputs** as the control input, and PX4's own internal `.ulg` flight log (converted via `ulog2csv`) is a better source for this than a live MAVLink telemetry stream, since it's higher-rate, doesn't drop packets, and includes the real actuator commands PX4 sent to the motors.

### Getting good data in the first place

Neither method can discover dynamics the flight never excites. A flight that only hovers and lands teaches nothing about pitch/roll coupling or drag. This is why the voice command set includes explicit **excitation maneuvers** -- `pitch sweep` and `roll sweep` command a sinusoidal oscillation via PX4's Offboard attitude mode specifically to generate the kind of varied, information-rich trajectory that system identification actually needs.

---

## 3. Simulation & Infrastructure Point of View

### Why PX4 + Gazebo instead of a custom physics engine

An early version of this project used a hand-written Python 6-DOF simulator. That's excellent for *learning* the underlying math (it's fully transparent, every line is inspectable), but it's a poor choice for a genuine system-identification exercise -- rediscovering your own hand-written equations proves the method works, not that it generalizes. Switching to **PX4 SITL + Gazebo** replaces the "ground truth" with a real, independent flight-control stack and physics engine, closing that gap.

### The stack

```
Voice (microphone)
   |
   v
Vosk (offline speech recognition)
   |
   v
voice_control.py  --MAVSDK-->  PX4 SITL  <--Gazebo physics-->  x500 quadrotor model
   |                              |
   |                              +--MAVLink-->  QGroundControl (Windows)
   |                              +--MAVLink-->  data_logger.py --> flight_data.csv
   |
   +-- offboard velocity/attitude setpoints for maneuvers
```

### Why this runs on WSL2, and the infrastructure lessons that came out of it

PX4's Gazebo integration doesn't support Windows natively -- **WSL2** (a real Ubuntu Linux environment inside Windows) is the standard, documented path. Getting there cleanly surfaced several non-obvious fixes worth recording, since they're easy to rediscover the hard way:

- **WSL2's default (NAT) networking assigns a new internal IP every restart**, which breaks any hardcoded IP in MAVLink/QGroundControl connection strings. Fixed permanently via **mirrored networking mode** (`.wslconfig`: `networkingMode=mirrored`), available on Windows 11 22H2+, which makes WSL and Windows share the same network stack -- `127.0.0.1` then works identically on both sides, with no IP-chasing required.
- **Microphone access** does *not* need a manually installed Windows-side PulseAudio server bridged over TCP. Modern WSL ships **WSLg**, which already provides a working PulseAudio server (`/mnt/wslg/PulseServer`) with built-in mic passthrough (`PulseAudioRDPSource`). A leftover `PULSE_SERVER` environment variable pointing elsewhere will silently override this working default and break audio -- `unset PULSE_SERVER` fixes it outright.
- **The actual root cause of a long chain of "Address already in use" and hanging-connection errors** wasn't gRPC ports, orphaned `mavsdk_server` processes, or anything Python-side -- it was a single PX4 command using **the same port number for both local and remote** (`mavlink start -u 14530 -o 14530`). PX4's own default streams (14550, 14540) always use *different* local/remote port numbers; matching that asymmetric pattern (`-u 14535 -o 14530`) resolved every downstream symptom at once. A MAVLink instance must bind a port that no other consumer also needs to bind, and the local/remote pair should generally differ.
- **Headless mode** for Gazebo is `HEADLESS=1`, not any `PX4_GZ_SIM_...`-prefixed variable -- a wrong environment variable name that looks plausible can silently do nothing.

### Why QGroundControl is kept in the loop

Even with voice as the primary interface, QGC provides an independent, natively-rendered (not WSL-graphics-dependent) visualization of the same flight -- map, artificial horizon, telemetry -- and serves as a manual override path if voice control ever misbehaves, which is standard practice for any autonomous/voice-driven vehicle project.

---

## 4. Coding Point of View -- The Scripts

### `voice_control.py`
Connects to PX4 over its own dedicated MAVLink port (`14530`, asymmetric-paired as above) via MAVSDK. Runs Vosk for fully offline speech-to-text, then dispatches recognized phrases to a command table. Two categories of command:

- **Direct Action commands** (arm, disarm, takeoff, land, hold, return-to-launch, goto_location) -- MAVSDK's high-level API, each independently wrapped in `try/except` so a failed command never kills the listening loop.
- **Offboard maneuvers** (directional velocity pulses, pitch/roll sweeps, one-revolution circle) -- these require continuously streaming setpoints (PX4 will fail safe if setpoints stop arriving), so each is implemented as its own `try/finally` block that guarantees a fallback to `hold()` even if the maneuver is interrupted or fails partway.

Dynamic named waypoints (`save point <name>` / `go to <name>` / `fly to <name>`) let you mark the drone's current live GPS position under an arbitrary spoken name and return to it later, rather than relying on hardcoded coordinates.

### `data_logger.py`
Connects on a separate dedicated port (`14540`) and subscribes to several MAVSDK telemetry streams concurrently (attitude, velocity, IMU, position), writing a synchronized row to `flight_data.csv` on a fixed timer regardless of which stream last updated. Actuator-output logging is attempted but wrapped defensively, since not every SITL build exposes that topic over MAVSDK -- PX4's own internal `.ulg` log (see below) is the more complete fallback for that specific data.

### `fit_sindy.py` (and the DMDc equivalent)
Loads `flight_data.csv`, builds either a linear (DMDc) or nonlinear-candidate-library (SINDy) representation, fits it via least-squares/sparse regression, and reports either the identified `A`/`B` matrices and eigenvalue-based stability analysis (DMDc) or a human-readable set of discovered differential equations (SINDy) -- then simulates the fitted model forward and compares it against the real logged trajectory as a validation check.

### A better data source worth adopting: `ulog2csv`
PX4 already logs a complete, high-rate internal record of every flight to a `.ulg` file automatically, no custom logger required. `pyulog`'s `ulog2csv` command converts this into per-topic CSVs -- including real actuator outputs -- which is a cleaner and more complete input to the identification scripts than anything sampled live over MAVLink.

---

## 5. Known Limitations & Honest Caveats

- The pitch/roll sweep maneuvers use a fixed, roughly-guessed hover-thrust value for the x500 model in Offboard attitude mode -- expect some altitude drift during a sweep; this is why sweeps are kept short.
- `do_orbit` support depends on the exact PX4 build; the circle maneuver is wrapped to fail gracefully (printing an error and returning to hold) rather than crashing if unsupported.
- DMDc is expected to degrade in accuracy for large-angle maneuvers, since it can only ever represent linear dynamics -- this is a known, expected result, not a bug, and is exactly why SINDy is run alongside it.
- Open-loop simulation of a discovered model (feeding it real recorded control inputs with no feedback correction) will diverge from the true trajectory over time even for a good model, since small per-step errors compound with nothing to correct them -- this is a property of open-loop testing, not necessarily evidence the identified model is bad.

---

## 6. Quick Reference -- Running It

```bash
# Terminal 1
cd ~/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500
# at pxh>:
mavlink start -u 14535 -o 14530 -r 4000 -t 127.0.0.1

# Windows: open QGroundControl (auto-connects on 14550)

# Terminal 2
python3 data_logger.py

# Terminal 3
python3 voice_control.py

# After landing, stop the logger (Ctrl+C), then:
python3 fit_sindy.py
```

### Voice command reference

| Category | Phrases | Action |
|---|---|---|
| System | "arm", "disarm" | Toggle motor arming |
| Basic flight | "take off", "land", "hold" / "stop" | Takeoff, land, or loiter |
| Waypoints | "save point *name*" | Remember current position |
| Navigation | "go to *name*" / "fly to *name*" | Fly to a saved point |
| Vector control | "move forward/backward/left/right" | 2-second velocity pulse |
| Altitude | "ascend" / "descend" | Altitude step |
| System ID | "pitch sweep" / "roll sweep" | Sinusoidal excitation for SINDy/DMDc |
| Trajectory | "circle maneuver" | One 360-degree orbit |
| Safety | "return home" | Return to launch |
| Session | "exit" | Stop listening (does not touch the drone) |
