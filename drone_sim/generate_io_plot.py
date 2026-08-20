"""
Generate Input vs. Output Data Visual Demonstration for Course Presentation / Professor Review.
Plots Input Data U(t) alongside Output State Data X(t) and Derivative Targets dX/dt.
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

current_dir = os.path.dirname(os.path.abspath(__file__))
log_file = os.path.join(current_dir, "..", "telemetry_logs", "perimeter_flight_250hz.csv")
out_dir = os.path.join(current_dir, "..", "data_driven_results")
os.makedirs(out_dir, exist_ok=True)

if not os.path.isfile(log_file):
    print("Telemetry log not found. Run analysis first.")
    sys.exit(1)

df = pd.read_csv(log_file)
# Slice first 30 seconds for ultra-crisp presentation display
df_sub = df[df["timestamp"] <= 30.0]
t = df_sub["timestamp"].values

fig, axes = plt.subplots(3, 1, figsize=(13, 10), dpi=160, sharex=True)
fig.suptitle("DATA-DRIVEN MODEL SYSTEM IDENTIFICATION: INPUT vs. OUTPUT DATASET\n(Amrita Coimbatore 250 Hz Flight Telemetry)", fontsize=13, fontweight="bold")

# Panel 1: INPUT DATA U(t)
ax1 = axes[0]
ax1.plot(t, df_sub["u1_pwm"], label="Motor 1 (u1 PWM)", color="#e63946", linewidth=1.5)
ax1.plot(t, df_sub["u2_pwm"], label="Motor 2 (u2 PWM)", color="#457b9d", linewidth=1.5)
ax1.plot(t, df_sub["u3_pwm"], label="Motor 3 (u3 PWM)", color="#2a9d8f", linewidth=1.5)
ax1.plot(t, df_sub["u4_pwm"], label="Motor 4 (u4 PWM)", color="#e76f51", linewidth=1.5)
ax1.set_ylabel("INPUT DATA U(t)\n[Motor PWM (0 to 1)]", fontsize=10, fontweight="bold")
ax1.set_title("1. CONTROL & EXOGENOUS INPUTS (Fed into SINDy & DMDc Matrix U)", fontsize=11, fontweight="bold", color="#1d3557")
ax1.legend(loc="upper right", ncol=4, fontsize=8)
ax1.grid(True, alpha=0.3)

# Panel 2: OUTPUT / STATE DATA X(t)
ax2 = axes[1]
ax2.plot(t, df_sub["p_rads"], label="Roll Rate p (rad/s)", color="#9b5de5", linewidth=1.4)
ax2.plot(t, df_sub["q_rads"], label="Pitch Rate q (rad/s)", color="#f15bb5", linewidth=1.4)
ax2.plot(t, np.radians(df_sub["roll_deg"]), label="Roll Angle φ (rad)", color="#00bbf9", linestyle="--", linewidth=1.4)
ax2.plot(t, np.radians(df_sub["pitch_deg"]), label="Pitch Angle θ (rad)", color="#00f5d4", linestyle="--", linewidth=1.4)
ax2.set_ylabel("OUTPUT DATA X(t)\n[Gyros & Orientation]", fontsize=10, fontweight="bold")
ax2.set_title("2. SYSTEM STATE RESPONSES (Observed Sensor Telemetry Vector X)", fontsize=11, fontweight="bold", color="#1d3557")
ax2.legend(loc="upper right", ncol=4, fontsize=8)
ax2.grid(True, alpha=0.3)

# Panel 3: DERIVATIVE TARGETS dX/dt & Accelerations
ax3 = axes[2]
ax3.plot(t, df_sub["ax_body"], label="Longitudinal Accel ax (m/s²)", color="#d90429", linewidth=1.3)
ax3.plot(t, df_sub["ay_body"], label="Lateral Accel ay (m/s²)", color="#3a86ff", linewidth=1.3)
ax3.plot(t, df_sub["az_body"], label="Normal Accel az (m/s²)", color="#8338ec", linewidth=1.3)
ax3.plot(t, df_sub["vup"], label="Climb Velocity v_up (m/s)", color="#38b000", linestyle=":", linewidth=1.5)
ax3.set_ylabel("DERIVATIVE TARGETS\n[Accelerations & Speeds]", fontsize=10, fontweight="bold")
ax3.set_xlabel("Flight Time t (seconds) — Sampled at 250 Hz (Δt = 0.004s)", fontsize=11, fontweight="bold")
ax3.set_title("3. SYSTEM DYNAMICS & DERIVATIVE TARGETS (Learned by SINDy Theta(X,U)*Xi)", fontsize=11, fontweight="bold", color="#1d3557")
ax3.legend(loc="upper right", ncol=4, fontsize=8)
ax3.grid(True, alpha=0.3)

plt.tight_layout()
out_fig = os.path.join(out_dir, "input_vs_output_data_definition.png")
fig.savefig(out_fig, bbox_inches="tight")
plt.close(fig)
print(f"[DDMS] Presentation plot saved: {out_fig}")
