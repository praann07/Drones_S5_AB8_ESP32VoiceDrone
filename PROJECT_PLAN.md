# ESP32 Voice Controlled GPS Waypoint Drone (LiteWing) — Project Plan

## 1. The Real Project (Goal)

Build an **ESP32 Voice-Controlled Drone** based on the CircuitDigest LiteWing project,
extended with **GPS waypoint navigation**.

### Hardware (what we will build)
- **LiteWing** — all-in-one ESP32-S3 drone board with onboard MPU6050 IMU (no separate
  ESP32 dev board / ESCs / flight controller needed).
- **GPS/GNSS module** (u-blox M10 class) mounted on a mast above the frame.
- **BMP280 barometer** (optional but recommended) for altitude hold.
- **LiPo battery**, propellers, GPS mast, buzzer, prop guards.
- **Laptop** = Ground Control Station (GCS) + voice recognition.

### Architecture
```
                LAPTOP (Windows)
        Python: Vosk voice + GCS map/commands
                    │  Wi-Fi / Bluetooth / ESP-NOW
                    ▼
              ESP32-S3 (LiteWing)
         GPS  IMU  Barometer  →  Navigation Logic
                    │
              Flight Controller / PID / Motor Mixer
                    │
              4 × ESC → 4 × Motors → Props
```

- The **laptop only sends commands** ("go to point A", "start mission").
- The **drone** reads GPS + IMU, does navigation and flight control, runs failsafes.

### Flight Modes
1. **Manual** — voice: forward / back / left / right / up / down
2. **GPS Point** — "go to point A"
3. **GPS Mission** — HOME → A → B → C → HOME
4. **Emergency / Failsafe** — "return home", geofence breach, comms loss, low battery

### Progressive Build Stages (from notes)
1. Get it flying manually
2. GPS coordinates display
3. Distance + bearing to target
4. GPS position hold
5. One waypoint
6. Multiple waypoints
7. Voice commands
8. Geofence + return-home + failsafe

---

## 2. Simulation Strategy

### Phase 1 — Pure Python simulator (DONE)
A self-contained Python sim was built in `drone_sim/`:

- Physics quadcopter (mass, motor mixer, drag, GPS/baro/mag noise)
- Cascaded flight controller (altitude PID, attitude-rate, velocity, position)
- GPS waypoint missions A→B→C→HOME, geofence, return-home, failsafe
- Pygame GCS dashboard: map, telemetry, battery, motor bars, mission progress, status banner
- Voice control (Vosk + sounddevice)
- Auto-demo: takes off and flies the mission on launch
- Headless auto-tests: all 6 phases PASS

```
drone_sim/
  config.py       constants (physics, control, mission)
  physics.py      quadcopter dynamics + sensor noise
  controller.py   PID flight controller
  navigation.py   waypoints, distance/bearing, mission
  commands.py     voice/GCS command parser
  voice.py        Vosk speech recognition
  dashboard.py    pygame GCS UI
  main.py         entry point (--headless, --smoke, --manual)
  run_sim.bat     double-click launcher
```

### Phase 2 — Gazebo + PX4 SITL via WSL (IN PROGRESS)
Industry-standard simulation on Windows via WSL2:

```
Windows (Python GCS + Voice)
      │  MAVLink (network port, localhost)
      ▼
WSL2 Ubuntu 24.04 terminal
      ├── PX4 Autopilot (SITL)  ← plays the role of the onboard flight controller
      └── Gazebo                 ← 3D world + drone body physics
```

- **PX4 SITL** runs a real autopilot: GPS waypoint logic, arming, takeoff/land, failsafe.
- **Gazebo** renders the drone in a 3D world and provides flight physics.
- Our existing **Python GCS + voice** stays on Windows and commands the sim drone over
  MAVLink — same mental model as the real drone (laptop = GCS, drone = flight controller).

---

## 3. Environment Status

| Item | Status |
|---|---|
| Windows 11 (build 26200) | OK |
| WSL2 installed | OK |
| Ubuntu 24.04 LTS in WSL (user: `tittu`, 12 cores, ~7.6 GB RAM) | OK |
| Disk space in WSL (~955 GB free) | OK |
| `sudo` passwordless setup | **BLOCKED** — user must run the sudoers command in own terminal |
| PX4 clone + dependencies | Pending |
| Gazebo + PX4 SITL build | Pending |
| Windows ↔ PX4 MAVLink link | Pending |

### One-time sudo setup (user runs in WSL terminal, types own password)
```bash
echo "tittu ALL=(ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/tittu-nopass
```

---

## 4. Roadmap (What We Will Do)

1. **Finish WSL prep** — passwordless sudo → `apt update` → install base tools.
2. **Install PX4 dependencies** — clone `PX4-Autopilot`, run official `ubuntu.sh` setup
   (installs Gazebo + toolchain). Takes a while (~30–60 min download/build).
3. **Build SITL target** — `make px4_sitl gz_x500` (Gazebo + PX4). First build compiles
   the full autopilot (~10–30 min).
4. **Verify flight** — launch the sim, confirm the drone arms, takes off, flies a
   waypoint mission in the Gazebo 3D view, and lands.
5. **Connect Windows Python GCS** — our voice + GCS (`drone_sim` or a new `mavlink_gcs`)
   sends MAVLink commands (takeoff, go to point, mission, return home) to PX4 over
   localhost, mirroring the real drone workflow.
6. **Demo mission** — speak or click: takeoff → A → B → C → return home → land, all
   visible in Gazebo from the WSL terminal.
7. **(Later) Map sim behavior to real LiteWing firmware + hardware** using
   `EXPENDITURE.txt` parts list and the build stages above.

---

## 5. Key Files in This Folder
- `notes.txt` — full project research (physical setup, GPS math, BOM reasoning)
- `EXPENDITURE.txt` — India component price list
- `drone_sim/` — Phase 1 pure-Python simulator (working)
- `PROJECT_PLAN.md` — this file
- `run_sim.bat` — launches the Phase 1 sim
