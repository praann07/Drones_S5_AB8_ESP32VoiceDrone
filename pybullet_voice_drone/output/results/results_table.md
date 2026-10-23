# Measured results

Generated 2026-10-21 23:07 by `scripts/measure_results.py` (Python 3.12.10, pybullet 3.2.8, Windows-11-10.0.26300-SP0).  
The controller was run headless at 60 steps per second with fixed random seeds, so the numbers repeat.

## Flight-model behaviour

### Take-off

| Command | Metric | Value |
|---|---|---:|
| take off | time to reach 2 m | 2.783 s |
| take off | final altitude error | 0 m |

### Measured move

| Command | Metric | Value |
|---|---|---:|
| forward 3 meters | time to complete | 3.25 s |
| forward 3 meters | along-track error | 0.016 m |
| forward 3 meters | sideways error | 0 m |

### Measured turn

| Command | Metric | Value |
|---|---|---:|
| rotate left 90 degrees | time to complete | 2.9 s |
| rotate left 90 degrees | heading error | 0.126 deg |

### Stop before an obstacle

| Command | Metric | Value |
|---|---|---:|
| forward (towards the first tower) | stopping position x | 0.83 m |
| forward (towards the first tower) | distance held from the safety zone | 0.4 m |
| forward (towards the first tower) | closest approach to the zone | 0.4 m |
| forward (towards the first tower) | peak deceleration | 3 m/s^2 |
| forward (towards the first tower) | hard safety-net interventions | 0 count |

### Steer around obstacles

| Command | Metric | Value |
|---|---|---:|
| forward through all three towers (--on-obstacle steer) | time to the far wall | 9.833 s |
| forward through all three towers (--on-obstacle steer) | closest approach to any zone | 0.699 m |
| forward through all three towers (--on-obstacle steer) | hard safety-net interventions | 0 count |

### Return home (open space)

| Command | Metric | Value |
|---|---|---:|
| go home | starting distance | 9.434 m |
| go home | time to land | 10.05 s |
| go home | landing error from the pad centre | 0.096 m |

### Return home (through the towers)

| Command | Metric | Value |
|---|---|---:|
| go home (--on-obstacle steer) | starting distance | 13.013 m |
| go home (--on-obstacle steer) | time to land | 13.117 s |
| go home (--on-obstacle steer) | landing error from the pad centre | 0.095 m |

## Randomised safety check

Random start position in the plaza, random heading, and a random one of forward / backward / left / right, each flown for 20 s with the default stopping behaviour. Starts that began inside a safety zone were discarded.

| Command | Metric | Value |
|---|---|---:|
| random start, heading and direction, 20 s each | runs | 190 count |
| random start, heading and direction, 20 s each | starts discarded (inside a zone) | 30 count |
| random start, heading and direction, 20 s each | runs that entered a safety zone | 0 count |
| random start, heading and direction, 20 s each | runs that needed the hard safety net | 0 count |
| random start, heading and direction, 20 s each | runs that ended held at an obstacle | 27 count |

## Main-loop performance (live window)

Measured by `scripts/measure_loop_performance.py` with the full city, traffic, labels, chase camera and overlays running (raw output in `loop_performance.txt`).

| Frames in 6 s | Loop rate | Work per frame |
|---:|---:|---:|
| 658 | about 110 Hz | 2.61 ms of a 8.3 ms budget |

This measures the Python loop, not the graphics card's own frame rate.

## Cost of each PyBullet call (live window)

Raw output of `scripts/benchmark_gui_calls.py`. The window-touching calls (text, lines, colours) cost far more than moving a body, and how much more varies from run to run with how busy the window is; moving bodies stays under 0.1 ms in every run.

```text
cost per call (full city scene loaded):
  resetBasePositionAndOrientation (drone)           0.08 ms/call
  resetBasePositionAndOrientation (car)             0.05 ms/call
  resetJointState (rotor)                           0.02 ms/call
  stepSimulation                                    1.17 ms/call
  getDebugVisualizerCamera                          0.04 ms/call
  resetDebugVisualizerCamera                        0.02 ms/call
  addUserDebugText REPLACE (same position)          6.27 ms/call
  addUserDebugText REPLACE (new position)           6.29 ms/call
  addUserDebugLine REPLACE                          6.35 ms/call
  changeVisualShape colour (halo box)               6.45 ms/call
  changeVisualShape colour (drone link: light)      6.39 ms/call
  changeVisualShape colour (drone link: disc)       6.40 ms/call
```
