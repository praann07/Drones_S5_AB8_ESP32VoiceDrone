# Output folder

Results, screenshots and logs of the PyBullet voice-drone simulation, produced on 2026-10-21 from the code on the
`Drone_pybullet` branch (commit `2af6627`) of `praann07/Drones_S5_AB8_ESP32VoiceDrone`.

Everything here was produced by running the real code. Nothing was typed in by hand.

## Contents

| Path | What it is |
|---|---|
| `results/results_table.md` | **The results table**: every measured number, grouped by behaviour (read this first) |
| `results/results_table.csv` | The same numbers, one measurement per row (opens in Excel) |
| `results/results.json` | The same numbers for further processing |
| `results/unit_tests.txt` | Output of `python -m unittest -v`: all 145 tests, `OK` |
| `results/loop_performance.txt` | How long each part of the main loop takes in the live window |
| `results/gui_call_costs.txt` | How long each kind of PyBullet call takes in the live window |
| `results/dashboard_cost.txt` | What recording the live graphs costs the simulation loop (headless) |
| `results/scripts/` | The scripts that produced the above, so everything can be repeated |
| `screenshots/` | Seven frames from the simulation window's own renderer (01 - 07) and two screenshots of the live graphs (08 - 09) |
| `logs/sample_flight_log.csv` | A real telemetry log written by the simulation during a scripted flight |
| `logs/sample_flight_console.txt` | What the console showed during that same flight |
| `logs/dashboard_flight_telemetry.csv`, `logs/dashboard_flight_console.txt` | The telemetry and console of the flight behind the live-graph screenshots |

## Screenshots

| File | Shows |
|---|---|
| `01_city_overview.png` | The whole plaza from above: helipad, kiosk towers, trees, ring road, traffic, skyline |
| `02_helipad_parked.png` | The drone parked on the helipad |
| `03_low_view_toward_buildings.png` | A low view toward the buildings |
| `04_stopped_at_obstacle.png` | The drone held in front of the first tower, which glows red |
| `05_traffic_on_ring_road.png` | Cars and crosswalks on the ring road |
| `06_chase_camera.png` | The chase camera, behind the drone |
| `07_top_camera.png` | The top camera, like a mission map |
| `08_live_graphs_dark.png` | The live graphs in the browser (dark theme) right after a scripted flight: take off, forward (held in front of the first tower), an unrecognised phrase, back 3 m, turn left 90 degrees, left 2 m, up 1 m, go home |
| `09_live_graphs_light.png` | The same page and flight in the light theme |

The labels, sensor ray, floor line and trail are drawn by the window itself and do not appear in captures 01 - 07.
Captures 08 and 09 are of the browser page (`http://127.0.0.1:8765/?theme=dark` and `?theme=light`) taken with headless Microsoft Edge while the simulation was still running.

## The sample flight

`logs/sample_flight_log.csv` was recorded by running `obstacles.py` with these typed commands:
*take off, forward, back 3 meters, rotate left 90 degrees, left 2 meters, go home, exit*.
It shows the drone taking off to 2 m, being held at x = 0.83 m in front of the first tower, flying 3 m back
(ending 1.6 cm from the target), turning to 89.9 degrees, moving 2 m left, returning home and landing about 10 cm
from the pad centre. Columns: `Time, Voice_Input, Command, Velocity_X/Y/Z, Drone_X/Y/Z, Yaw` (metres, m/s, degrees).

## Two things to know about the numbers

- **Flight-model results are exact and repeatable.** They come from running the controller headless with fixed
  random seeds, and a second run gave identical numbers.
- **The cost of window-touching PyBullet calls varies between runs.** An earlier measurement found colour changes
  at 0.1 - 0.9 ms and text rewrites at about 3 ms; the run saved here found about 6 ms for all of them. What stays
  the same in every run is that these calls cost far more than moving a body (under 0.1 ms), which is why the
  simulation moves bodies every frame but rewrites text and colours only rarely. The main-loop result (about 110 Hz,
  2.6 ms of work per frame) reproduced closely both times.

## Repeating the measurements

The scripts in `results/scripts/` find the project folder themselves (it is three levels up from the script), so
they can be run from anywhere on any computer. Run them with a Python that has `pybullet` installed:

| Script | Needs a window? | Produces |
|---|:---:|---|
| `measure_results.py` | no | `results_table.md/.csv`, `results.json` |
| `measure_loop_performance.py` | yes | the numbers in `loop_performance.txt` |
| `benchmark_gui_calls.py` | yes | the numbers in `gui_call_costs.txt` |
| `capture_screenshots.py`, `capture_camera_views.py` | yes | the screenshots 01 - 07 (saved to `%TEMP%\city_shots`) |
| `benchmark_dashboard.py` | no | the numbers in `dashboard_cost.txt` |
| `capture_dashboard.py` | yes | flies the scripted flight with typed commands and saves the two live-graph screenshots, the telemetry and the console text next to the script (needs Microsoft Edge) |
