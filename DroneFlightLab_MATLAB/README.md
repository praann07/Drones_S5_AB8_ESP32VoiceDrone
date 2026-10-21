# MATLAB Drone Flight Simulation

A MATLAB-based drone flight simulation with seven flight commands, a separate voice-listening trigger, obstacle-stop behavior, and flight-analysis plots.

## Contents

- `DroneFlightLab_2026.m` — main MATLAB simulation.
- `listen_voice.ps1` — Windows voice-listener helper.
- `README_SETUP.txt` — setup instructions supplied with the project.
- `OBSTACLE_STOP_FIX.txt` — obstacle-stop notes supplied with the project.
- `outputs/` — six supplied output-graph screenshots.

## Flight commands

| Command | Purpose |
|---|---|
| TAKE OFF | Begin simulated takeoff |
| TURN LEFT | Turn left |
| TURN RIGHT | Turn right |
| BACKWARD | Move backward |
| HOVER | Hold position |
| LAND | Land the simulated drone |
| RETURN TO HOME | Return toward the home position |
| LISTEN TO VOICE | Separate trigger for voice-command listening; not a flight command |

## Running the project

In MATLAB, set the working directory to your extracted project folder. If you extracted the project under Downloads using the folder name below, run:

```matlab
cd(fullfile(getenv('USERPROFILE'), 'Downloads', 'DroneFlightLab_COMMANDS_7_OBSTACLE_STOP_FIXED'))
```

Then follow the instructions in `README_SETUP.txt` and run the main simulation file `DroneFlightLab_2026.m`.

If the folder has a different name or location after extraction, update the path accordingly.

## Output graphs

### East position: actual vs target

![East position actual versus target](outputs/east_position_actual_vs_target.png)

### North position: actual vs target

![North position actual versus target](outputs/north_position_actual_vs_target.png)

### Altitude: actual vs target

![Altitude actual versus target](outputs/altitude_actual_vs_target.png)

### Axis-wise and 3D tracking error

![Axis-wise and 3D tracking error](outputs/axis_wise_and_3d_tracking_error.png)

### Velocity components and total speed

![Velocity components and total speed](outputs/velocity_components_and_total_speed.png)

### Drone attitude: roll, pitch, and yaw

![Drone attitude](outputs/drone_attitude_roll_pitch_yaw.png)

> **Note:** These images are the output screenshots supplied with the project. They are not freshly regenerated MATLAB figures or raw telemetry data.

## Obstacle-stop behavior and limitations

The intended safety behavior is to stop the simulated drone before it enters a protected obstacle region rather than automatically detouring around it. The city obstacles and safety regions are simplified approximations. This is a software simulation, not a certified real-world collision-avoidance system.

## Voice listener

`listen_voice.ps1` is a Windows PowerShell helper. Review the supplied setup notes before using it, and ensure the required permissions and dependencies are available.

## Troubleshooting

- If MATLAB reports that telemetry variables are missing, follow `README_SETUP.txt` and run the simulation/export workflow before running a plotting-only script.
- If MATLAB selects the wrong copy of the main file, run:

```matlab
which DroneFlightLab_2026 -all
```

Ensure the returned path points to the extracted project folder.

- If a graph image does not render on GitHub, check that the `outputs` folder and image filenames match the relative paths above.

## Safety

This project is for simulation and educational use. Do not interpret the obstacle-stop behavior as proof of safe operation on physical drone hardware.
