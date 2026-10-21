<p align="center">
  <img src="amrita_logo.png" alt="Amrita Vishwa Vidyapeetham" width="420"/>
</p>

| Name | Roll No. | Email |
|---|---|---|
| Mamidala Som Praneeth Babu | CB.SC.U4AIE24029 | cb.sc.u4aie24029@cb.students.amrita.edu |
| C. Karthik | CB.SC.U4AIE24107 | cb.sc.u4aie24107@cb.students.amrita.edu |
| Ch. Virinchi | CB.SC.U4AIE24158 | cb.sc.u4aie24158@cb.students.amrita.edu |
| K. Jatin Chandra Gupta | CB.SC.U4AIE24162 | cb.sc.u4aie24162@cb.students.amrita.edu |

---

# 🎙️ Voice-Controlled Quadcopter Drone in PyBullet 🚁

A voice-controlled quadcopter simulation built with **PyBullet** and **SpeechRecognition**. Speak a command, or press a key, and the drone flies over a generated city plaza. It stops by itself before it hits anything, and it logs every flight to CSV.

This is the `tittu-09-voice-drone` branch. All of its code is in [`pybullet_voice_drone/`](pybullet_voice_drone/); the full documentation is in [`pybullet_voice_drone/README.md`](pybullet_voice_drone/README.md).

| City plaza overview | Held in front of an obstacle (the tower glows) |
| :---: | :---: |
| ![City overview](pybullet_voice_drone/screenshots/city_overview.png) | ![Obstacle stop](pybullet_voice_drone/screenshots/city_obstacle_stop.png) |
| **Chase camera** | **Top camera** |
| ![Chase camera](pybullet_voice_drone/screenshots/city_chase.png) | ![Top camera](pybullet_voice_drone/screenshots/city_top_view.png) |

## What it does

- **Voice control** with natural phrases (*"take off"*, *"go left"*, *"turn to the left"*, *"land"*), tuned for Indian English and tolerant of common mishearings. The keyboard works at any time as an override.
- **Measured commands**: *"forward 3 meters"*, *"up ten feet"*, *"rotate left 90 degrees"*, and *"go home"* to fly back over the take-off point and land.
- **Stops before obstacles**: it brakes smoothly and holds still at a safe distance. `--on-obstacle steer` flies around them instead.
- **A living city**: a plaza with a helipad, a ring road with moving traffic, pedestrians, trees, benches and a skyline, all generated in code with no downloaded assets.
- **Four cameras** (follow, chase, top, fixed), labels above the drone showing its state, and a sensor ray, flight trail and optional safety zones.
- **Telemetry**: every command and one sample per second while airborne go to a CSV file.
- **133 headless unit tests** (no window, microphone or internet needed).

It is a kinematic visualisation (commanded velocities, not simulated rotor thrust), so it complements the dynamics-based simulators on the other branches rather than replacing them.

## Quick start

Python 3.9 – 3.12. A microphone and an internet connection are only needed for voice control.

```bash
cd pybullet_voice_drone
python -m venv venv
venv\Scripts\activate            # Windows (Linux/macOS: source venv/bin/activate)
pip install -r requirements.txt

python obstacles.py              # voice + obstacles (the default demo)
python Voice_Drone_PyBullet.py   # voice, no obstacles
python main.py                   # keyboard only
```

The drone starts on the ground: say *"take off"* (or press `T`) first. Useful keys: `T` take off, `L` land, `H` go home, `W/A/S/D` move, `Q/E` up/down, arrows rotate, `Space` stop, `C` camera, `P` trail, `Z` safety zones, `X` exit. Run the tests with `python -m unittest -v`.

## Other work in this repository

The repository holds several attempts at the same voice-controlled drone, each on its own branch:

- **[`main`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/main)** is our original ESP32-based attempt. Its documentation (Vosk voice parsing, cascaded PID control, GPS waypoints, SINDy/DMDc analysis) is kept in this branch as [`README_ESP32_SIMULATOR.md`](README_ESP32_SIMULATOR.md), and its code is in [`drone_sim/`](drone_sim/).
- **[`drone_mujoco`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/drone_mujoco)** is a MuJoCo-based voice-controlled quadrotor with a SINDy-identified data-driven attitude controller (full pipeline: excitation → identification → validation → cascaded control → live voice demo).
- **[`drone_gazebo`](https://github.com/praann07/Drones_S5_AB8_ESP32VoiceDrone/tree/drone_gazebo)** is a PX4 + Gazebo voice-controlled quadrotor, also using SINDy/DMDc identification.
