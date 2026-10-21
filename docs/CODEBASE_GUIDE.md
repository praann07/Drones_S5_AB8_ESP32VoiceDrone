# Codebase Guide

**Audience:** course grader / senior engineer who wants to read the code, not just the results.
**Companion docs:** [`ONBOARDING.md`](../ONBOARDING.md) (what is claimed and how to check it), [`CONTROLLER_AND_IMU_EXPLAINED.md`](../CONTROLLER_AND_IMU_EXPLAINED.md) (plain-English controller/IMU walkthrough), [`CLAUDE.md`](../CLAUDE.md) (decision and bug history). This file is the code map: where things live, how data flows, how to change things safely, and what breaks.

Facts below were checked against the working tree on 2026-10-21 (Python 3.13, 157 tests collected).

---

## 1. Architecture in one picture

```
 mic / typed text
      │
      ▼
 voice_input.py ──► commands.py: classify() ─► command word (8) or "goto x y z"
      │                         target_offset()  world-frame offset
      ▼
 flight.py: FlightController.dispatch()
      ├─ city.py: clamp_target_to_safe_zone()   never target inside a building
      └─ trajectory.py: QuinticBezierTrajectory (carries v0, a0 → no dead stop)
      │
      ▼   every 2 ms (500 Hz)  FlightController.step()
 measured_state()  ◄── IMU sensors in quad.xml (framepos, framequat, velocimeter, gyro)
      │
      ▼
 position_controller.py   OUTER loop, standard physics (NOT identified)
      │  q_desired + thrust
      ▼
 controller.py            INNER loop, quaternion P² law,
      │                   gains from the frozen SINDy model
      ▼
 mixer.py: inverse_mix(_yaw_priority)   (thrust, torque) → 4 rotor thrusts
      ▼
 MuJoCo plant (scene.xml → quad.xml), drag applied as qfrc_applied
```

Autonomy mode swaps one box: `FlightController.reference = AvoidancePlanner(...)` replaces the dispatched trajectory as the outer loop's reference. Everything below it is unchanged.

**The design rule that explains most of the layout:** only the inner attitude loop is identified from data (SINDy). The outer position loop is textbook physics on purpose (a double integrator has nothing to discover). See `docs/TDD.md` §7b.

---

## 2. Reading order (about 45 minutes)

| # | File | Why |
|---|---|---|
| 1 | `docs/TDD.md` §1–2, §7, §7b | Frames, quaternion convention, rotor geometry, the cascade argument |
| 2 | `01_simulation/models/quad.xml` | The plant and its IMU `<sensor>` block |
| 3 | `01_simulation/sim_driver.py` | Where drag (`C_ROT`, `C_TRANS`) lives. It is **not** in the XML |
| 4 | `02_identification/sindy_fit.py`, `preprocessing.py` | The graded core: library, thresholding, segment-boundary rule |
| 5 | `03_validation/run_validation.py` | Gates, ablations, model selection, `freeze_selected_model()` |
| 6 | `04_control/controller.py` | `gains_from_identified_model()` and the P² law |
| 7 | `04_control/position_controller.py`, `mixer.py`, `trajectory.py` | Outer loop, torque-to-rotor map, planner |
| 8 | `05_voice_interface/flight.py` | Where everything is wired together |
| 9 | `tests/test_identification_physics.py`, `test_scene_physics_inert.py` | The two tests that guard the project's core claims |

---

## 3. Module map

| Folder | Role | Notable files |
|---|---|---|
| `01_simulation/` | Plant + open-loop excitation (no controller, by design) | `excitation.py` (PRBS/chirp), `run_excitation.py`, `sim_driver.py` (`MODEL_PATH`, `SCENE_PATH`) |
| `01_simulation/models/` | MJCF and the city | `quad.xml` (physics), `scene.xml` / `city.xml` / `scene_auto.xml` (visual-only, generated), `city.py` (single source of truth for buildings + `clamp_target_to_safe_zone`), `obstacles.py` (moving obstacles), `make_city.py` (deterministic generator) |
| `02_identification/` | SINDy + DMDc fits | `sindy_fit.py`, `dmdc_fit.py`, `preprocessing.py`, `run_identification.py` |
| `03_validation/` | One-step → rollout → eigenvalues → ablations | `rollout.py`, `run_validation.py` (writes `data/processed/sindy_fitted_model.npz`) |
| `04_control/` | The cascade | `controller.py`, `position_controller.py`, `mixer.py`, `trajectory.py`, `avoidance.py`, `closed_loop_sim.py`, `metrics.py` |
| `05_voice_interface/` | Voice, flight wrapper, demos, studies | `flight.py`, `commands.py`, `voice_input.py`, `demo.py`, `run_trials.py`, `run_preemption.py`, `run_robustness.py`, `run_autonomous.py` |
| `tests/` | 157 tests | see §6 |
| `results/` | Mirrored evidence gallery | rebuilt by `save_plots.py` |

**Import convention (surprising):** the numbered folders are not Python packages. Scripts do `sys.path.insert(0, ROOT / "<folder>")` and import by bare module name (`from controller import ...`). Do not add `__init__.py` and do not rename a module without grepping for its bare name. Module names must stay unique across folders.

---

## 4. Conventions that bite if ignored

- **Frames:** ENU world, FLU body. Identity quaternion = level hover. Hamilton, scalar-first `[w, x, y, z]`, body-to-world.
- **Units:** SI. **Rate:** 500 Hz, `DT = 0.002 s`.
- **Yaw is never commanded.** The outer loop uses the minimal-rotation tilt quaternion. `heading_mode="forward_facing"` exists and is unit-tested but was measured to destabilise the cascade, so `flight.py` pins `"fixed"`.
- **Held-out means separate trials**, never a slice of one trajectory (`data/processed/trial_split.parquet`).
- **Seeds:** use `seed_for_trial`, never `hash()` (Python randomises `hash()` per process). `tests/test_reproducibility.py` enforces cross-process determinism.
- **Index math off `DT`:** use `round(t / DT)`, not `int(t / DT)` (`int(0.7/0.002)` gives 349).
- **Windows PowerShell**, `pathlib` only, no hardcoded `/` paths.
- **Defaults reproduce the gated behaviour.** New behaviour is opt-in (`robust=False` is the original Stage D cascade; `reference=None` is voice/waypoint mode; `sensor_noise=None` is ideal sensors).

---

## 5. Data flow and artifacts

```
run_excitation.py    ─► data/raw/*.parquet, data/processed/trial_split.parquet
run_identification.py ─► identification_results.csv
run_validation.py    ─► validation_plots/, rollout_results.csv, sindy_fitted_model.npz   ◄── frozen model
run_inner_loop.py / run_cascade.py ─► control_plots/
05_voice_interface/* ─► voice_sessions/*.parquet (gitignored), voice_demo/, robustness_plots/, autonomy/
save_plots.py        ─► results/{plots_2d,plots_3d,videos,data}/
```

`sindy_fitted_model.npz` is the contract between identification and flight. `FlightController` loads it and **refuses to refit at runtime**. A stale or missing file is a loud error, not a silent refit. The file is loaded with `allow_pickle=True` (feature names are an object array), which is acceptable only because this project writes it.

---

## 6. Tests: what each file protects

| File | Protects |
|---|---|
| `test_identification_physics.py` | Recovered SINDy coefficients match ground-truth physics (about 2–6%) |
| `test_mixer.py`, `test_inverse_mixer.py` | Mixer matrix equals the XML's actual rotor sites and spin signs; inverse round-trips |
| `test_reproducibility.py` | Same seed gives same data across processes |
| `test_scene_physics_inert.py` | Scene/city/autonomy XML add no mass, joints, contacts, actuators (bit-for-bit open-loop comparison) |
| `test_voice_interface.py` | Classifier, waypoints, headless pipeline, safety clamp, telemetry, live logger |
| `test_natural_motion.py` | Planner/banking behaviour |
| `test_robustness.py` | Noise, wind, payload cases, yaw-priority mixer |
| `test_autonomy.py` | Avoidance planner and scenario-fairness rules (movers never pinch against buildings) |

Run: `.\.venv\Scripts\python.exe -m pytest` (about 17 s just to collect; full run is minutes).

---

## 7. Runbooks

**Change the controller's target response.** Edit `SETTLING_TIME_S` / `DAMPING_RATIO` in `controller.py`. These are the only hand-picked inner-loop numbers. Then rerun `run_inner_loop.py` and `run_cascade.py` and confirm the outer/inner bandwidth separation still holds (outer must be slower).

**Re-identify the model.** Edit the SINDy library or threshold, run `run_identification.py`, then `run_validation.py`. Check Stage B's one-step gate **before** reading any Stage C plot (project rule). `run_validation.py` re-freezes `sindy_fitted_model.npz`; commit that file with the change.

**Add a voice command.** Add the word to `COMMANDS` and an offset to `_COMMAND_OFFSET` in `commands.py`, add synonyms to `_SYNONYMS`, and add a test. `command_vocabulary()` is derived from `_SYNONYMS`, so the printed guide updates itself. If the offset moves the drone vertically, keep it small: the floor is visual-only and non-collidable.

**Change the city or obstacles.** Edit `city.py` / `obstacles.py` (never the generated XML), run `make_city.py`, then `test_scene_physics_inert.py`. Moving-obstacle ranges are constrained by `test_movers_never_pinch_against_buildings`.

**Add a disturbance study.** Use the opt-in hooks on `FlightController` (`sensor_noise`, `wind_force`, `wind_fn`, `set_payload_scale`). Noise is applied only inside `measured_state()`; the public accessors stay noiseless truth, so logged truth is still truth.

**Rebuild all evidence.** Double-click `REBUILD_RESULTS.bat` (logs to `results\logs\`), or run the stage scripts in order (listed in `ONBOARDING.md` §3).

---

## 8. Debugging guide

| Symptom / message | Cause | Fix |
|---|---|---|
| `RuntimeError: Frozen model artifact not found at ...sindy_fitted_model.npz` | Stage C never run (or file deleted) | `python 03_validation\run_validation.py` |
| `RuntimeError: quad.xml has no sensor named 'position'` (or `attitude`, `linear_velocity`, `angular_velocity`) | IMU `<sensor>` block edited | Restore the four named sensors; `flight.py` looks them up by name |
| pytest: `import file mismatch` on every module | The untracked nested copy `Drones_S5_AB8_ESP32VoiceDrone/` inside the repo root | `pytest.ini` excludes it; delete the nested copy before submission |
| Controller state lags by one step (small errors, 3 tests fail) | Sensors read right after `mj_step()` are pre-integration | `step()` ends with `mj_forward()`; keep it |
| Black or empty viewer / GIF | Loaded `quad.xml` instead of `scene.xml` | Use `SCENE_PATH`; open the output image to check, do not trust console output |
| Drone stops short of a commanded waypoint, log shows `[safety] ... clamped` | Target would enter a building | Intended. See `city.clamp_target_to_safe_zone` |
| Demo buttons or plots do nothing | Matplotlib backend silently resolved to `Agg`; or the window lacks OS keyboard focus | Launch via `RUN.bat` (real console); use the dashboard buttons, not hotkeys |
| Dashboard plots show old flights | Plots read the newest `voice_sessions/chained_*.parquet` by mtime | Fly first, then `RUN.bat` option 2 |
| Rotors saturate at 0 / 6 N, thrust lost under noise | Yaw is about 5x costlier in rotor headroom (`K_M` 0.02 vs `ARM` 0.106) | `robust=True` uses `inverse_mix_yaw_priority` |
| `.bat` aborts with `. was unexpected at this time.` | Unescaped `(` or `)` in text inside an `if/else` block | Remove or escape the parentheses |
| Voice mode silently not used | No `vosk-model-small-en-us-0.15/` folder (not in repo) | Download from alphacephei.com/vosk/models; otherwise typed-command mode is used |

**Where logs are:** console of the demo window (`[heard]`, `[dispatch]`, `[safety]` lines); `data/processed/voice_sessions/*.parquet`; `results/logs/` for `REBUILD_RESULTS.bat`; `MUJOCO_LOG.TXT` in the repo root.

---

## 9. Risky files (coordinate or re-validate before editing)

| Path | Risk |
|---|---|
| `01_simulation/models/quad.xml`, `sim_driver.py` (`C_ROT`, `C_TRANS`) | Changes the plant, which invalidates the frozen model and every gate |
| `data/processed/sindy_fitted_model.npz` | Regenerate only via `run_validation.py` |
| `04_control/controller.py` gain derivation | Gains and the outer/inner bandwidth separation were gated together |
| `04_control/mixer.py` | Pinned to the XML by `test_mixer.py` |
| `FlightController.step()` ordering | Sensor freshness, drag application and the trailing `mj_forward()` are order-sensitive |
| `.gitignore` | Parquet logs and large GIFs are ignored on purpose (one GIF exceeded GitHub's 100 MB limit) |

---

## 10. Known limits

Stated in full in `ONBOARDING.md` §7. The ones that matter when reading code: simulation only; DMDc valid near hover only; only the diagonal of the SINDy Jacobian feeds gain design (each axis designed independently, cross-coupling is treated as a disturbance); yaw tracking is not validated; the live microphone path has not been run end to end on this machine.

---

## 11. Glossary

- **SINDy:** Sparse Identification of Nonlinear Dynamics. Fits a sparse equation `x_dot = Theta(x, u) * Xi` from data.
- **DMDc:** Dynamic Mode Decomposition with control. A linear model fit, valid near one operating point.
- **P² law:** quaternion-error vector feedback plus angular-rate feedback, from Fresk and Nikolakopoulos.
- **Cascade:** outer position loop produces the attitude setpoint for the inner attitude loop. The inner loop must be faster.
- **IMU:** inertial measurement unit; here the `<sensor>` block in `quad.xml` (gyro, orientation, velocimeter, position).
- **Marginal pole:** eigenvalue on the stability boundary. Expected for attitude, since θ̇ = ω has no restoring force.
- **Gate:** a pass/fail check that must be confirmed before the next stage starts.
