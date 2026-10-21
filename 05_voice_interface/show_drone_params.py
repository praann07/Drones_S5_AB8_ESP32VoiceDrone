"""Print the drone's key numbers in plain language, for anyone.

Every value is read from the files the simulator itself uses (quad.xml,
mixer.py, sim_driver.py, trajectory.py, position_controller.py and the
frozen SINDy model), so this sheet can never disagree with the sim.
Run: SHOW_DRONE_PARAMS.bat  (or python 05_voice_interface/show_drone_params.py)
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
for _p in ("05_voice_interface", "04_control", "01_simulation", "01_simulation/models",
           "02_identification"):
    sys.path.insert(0, str(ROOT / _p))

import mujoco  # noqa: E402

from sim_driver import MODEL_PATH, C_ROT, C_TRANS, SAMPLE_RATE_HZ  # noqa: E402
from mixer import ARM, K_M, U_MAX  # noqa: E402
from trajectory import MAX_VELOCITY_MPS, MAX_ACCEL_MPS2, WAYPOINT_OFFSET_M  # noqa: E402
from position_controller import KP_POS, KD_POS, KI_POS_DEFAULT, MAX_BANK_DEG  # noqa: E402
from controller import gains_from_identified_model, SETTLING_TIME_S  # noqa: E402
from flight import load_frozen_sindy, MIN_CLEARANCE_M, LAND_DESCENT_MPS  # noqa: E402
from city import SAFETY_MARGIN_M  # noqa: E402

G = 9.81
W = 80


def section(title: str):
    print()
    print("=" * W)
    print(f"  {title}")
    print("=" * W)


def row(name: str, value: str, meaning: str):
    print(f"  {name:<26}{value:<18}{meaning}")


def main():
    m = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "quad")
    mass = float(m.body_mass[bid])
    ixx, iyy, izz = m.body_inertia[bid]
    weight = mass * G
    max_thrust = 4 * U_MAX
    hover_each = weight / 4
    sindy, A = load_frozen_sindy()
    Kq, Kw = gains_from_identified_model(A, sindy.coef_[3:6, 7:11])
    n_terms = int(np.count_nonzero(sindy.coef_))

    print("\n  YOUR DRONE, IN NUMBERS  -  every value read live from the simulator files")

    section("1. BODY  (what it is)")
    row("Type", "quadcopter", "4 propellers in an X shape")
    row("Mass", f"{mass:.2f} kg", "about a 1-litre water bottle")
    row("Weight", f"{weight:.2f} N", "the force gravity pulls it down with")
    row("Arm length", f"{np.hypot(ARM, ARM) * 100:.1f} cm", "centre to each propeller")
    row("Spin inertia roll/pitch", f"{ixx:.5f} kg m2", "how hard it is to tip it over")
    row("Spin inertia yaw", f"{izz:.5f} kg m2", "how hard it is to turn it on the spot")

    section("2. MOTORS  (what lifts it)")
    row("Max thrust per motor", f"{U_MAX:.1f} N", f"each motor can lift ~{U_MAX / G:.2f} kg")
    row("Max total thrust", f"{max_thrust:.1f} N", "all 4 motors flat out")
    row("Thrust-to-weight", f"{max_thrust / weight:.2f} : 1", "above 2 = agile; it can lift 2x its weight")
    row("Hover thrust per motor", f"{hover_each:.2f} N", f"just floating uses {100 * hover_each / U_MAX:.0f}% of each motor")
    row("Yaw torque factor", f"{K_M:.3f} m", "turning uses propeller drag, which is weak")
    row("Spin directions", "2 CW + 2 CCW", "opposite spins cancel, so it does not twist")

    section("3. AIR  (what slows it)")
    row("Air drag", f"{C_TRANS:.2f} N per m/s", f"at full speed drag is {C_TRANS * MAX_VELOCITY_MPS:.2f} N")
    row("Spin drag", f"{C_ROT:.3f} N m s", "air resisting rotation")
    row("Gravity", f"{G:.2f} m/s2", "Earth")

    section("4. HOW IT FLIES  (limits we set)")
    row("Top speed", f"{MAX_VELOCITY_MPS:.1f} m/s", f"= {MAX_VELOCITY_MPS * 3.6:.1f} km/h, brisk walking pace")
    row("Max acceleration", f"{MAX_ACCEL_MPS2:.1f} m/s2", "smooth starts and stops, no jerks")
    row("Max tilt", f"{MAX_BANK_DEG:.0f} deg", "it leans to move; never more than this")
    row("Step size (step mode)", f"{WAYPOINT_OFFSET_M:.1f} m", "one 'left' = this far, in step mode")
    row("Braking distance", f"{MAX_VELOCITY_MPS ** 2 / (2 * MAX_ACCEL_MPS2):.2f} m", "after 'stop' at top speed")
    row("Gap kept from walls", f"{SAFETY_MARGIN_M:.2f} m", "stops this far before a building")
    row("Lowest hover", f"{MIN_CLEARANCE_M:.2f} m", "'down' never goes lower; only 'land' does")
    row("Landing speed", f"{LAND_DESCENT_MPS:.2f} m/s", "gentle final descent")

    section("5. BRAIN  (the controllers)")
    row("Control rate", f"{SAMPLE_RATE_HZ:.0f} times/s", "it re-decides motor power every 2 ms")
    row("Outer loop (position)", "PID", "'where should I be?' -> 'which way to lean?'")
    row("  stiffness Kp", f"{KP_POS:.2f}", "pull back toward the target per metre off")
    row("  damping Kd", f"{KD_POS:.2f}", "brakes so it does not overshoot")
    row("  integral Ki", f"{KI_POS_DEFAULT:.2f}", "slowly cancels wind or extra weight")
    row("Inner loop (attitude)", "quaternion P2", "'which way to lean?' -> motor power")
    row("  attitude gain Kq", f"{Kq[0]:.2f} / {Kq[2]:.2f}", "roll-pitch / yaw: how hard it fixes a wrong lean")
    row("  rate gain Kw", f"{Kw[0]:.3f} / {Kw[2]:.3f}", "roll-pitch / yaw: damps wobble")
    row("  settles in", f"{SETTLING_TIME_S:.1f} s", "time to reach a new lean angle")

    section("6. LEARNED MODEL  (the data-driven part)")
    row("Method", "SINDy", "learns the equations of motion from flight data")
    row("Active terms", f"{n_terms} of {sindy.coef_.size}", "it keeps only the terms that matter")
    row("Used for", "inner-loop gains", "the attitude controller is tuned on this model")

    section("7. SENSORS  (what it feels)")
    row("Gyroscope", "spin rate", "how fast it is rotating")
    row("Attitude", "quaternion", "which way it is tilted")
    row("Position", "x, y, z", "where it is (like GPS)")
    row("Velocimeter", "speed", "how fast it is moving")
    row("Feet contact", "4 feet", "knows when it has touched down")
    print()


if __name__ == "__main__":
    main()
