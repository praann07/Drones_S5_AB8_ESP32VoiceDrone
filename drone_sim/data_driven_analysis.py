"""
Comprehensive Data-Driven Modeling & Simulation (DDMS) Analysis Engine.
Executes SINDy (Sparse Nonlinear System Identification) and DMDc (Dynamic Mode Decomposition)
on 250 Hz flight telemetry logs and exports comparative publication-ready charts and reports.
"""

import os
import sys
import math
import argparse
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

# Local imports
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

import config
from sindy_model import SINDyModel
from dmdc_model import DMDcModel
from data_logger import TelemetryLogger


def generate_benchmark_perimeter_telemetry(csv_path):
    """
    Simulates an automated 250 Hz perimeter flight over the Indian Campus
    to generate high-fidelity training data if no live log is available.
    """
    from physics import Drone
    from main import GCS
    import controller as ctrl

    print("[DDMS] >> Simulating 250 Hz perimeter survey over Indian Campus...")
    drone = Drone()
    logger = TelemetryLogger(filename=os.path.basename(csv_path), output_dir=os.path.dirname(csv_path))
    logger.reset()

    gcs = GCS(drone)
    gcs.failsafe_enabled = False
    gcs.tts = None  # Silent for automated data generation
    gcs.logger = logger

    # Enable moderate wind & gusts to capture disturbance dynamics
    drone.set_wind("LIGHT", speed=3.5, heading_deg=45.0)
    drone.trigger_gust(strength=4.5, duration=4.0)

    drone.arm()
    gcs.home_pos = (0.0, 0.0)
    gcs.handle("takeoff", {"alt": config.WAYPOINT_ALT})
    gcs.handle("mission", {})

    dt = config.DT
    t_max = 75.0
    steps = int(t_max / dt)

    for step in range(steps):
        drone.step(dt)
        gcs.update(dt)
        gcs.controller.update(drone, dt)

        # Disarm when mission finishes and landed
        if step > 500 and not drone.armed:
            break

    saved_file = logger.save_to_csv(csv_path)
    print(f"[DDMS] >> Generated {logger.sample_count} samples saved to: {saved_file}")
    return saved_file


def run_data_driven_pipeline(csv_file=None, output_dir=None):
    """
    Main Data-Driven Analysis Runner.
    """
    base_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(base_dir, "..", config.TELEMETRY_LOG_DIR)
    results_dir = output_dir or os.path.join(base_dir, "..", config.DATA_ANALYSIS_OUTPUT_DIR)
    os.makedirs(results_dir, exist_ok=True)

    if csv_file is None:
        csv_file = os.path.join(log_dir, config.TELEMETRY_CSV_FILE)

    if not os.path.isfile(csv_file):
        print(f"[INFO] Log file '{csv_file}' not found. Generating benchmark flight...")
        csv_file = generate_benchmark_perimeter_telemetry(csv_file)

    print(f"[DDMS] 📊 Loading flight telemetry from: {csv_file}")
    df = pd.read_csv(csv_file)
    print(f"[DDMS] Loaded {len(df)} telemetry snapshots ({df['timestamp'].iloc[-1]:.2f} seconds of 250 Hz flight).")

    # Extract State & Input Matrices
    # States: [p, q, r, roll_rad, pitch_rad, yaw_rad, vn, ve, vup]
    p = df["p_rads"].values
    q = df["q_rads"].values
    r = df["r_rads"].values
    roll_rad = np.radians(df["roll_deg"].values)
    pitch_rad = np.radians(df["pitch_deg"].values)
    yaw_rad = np.radians(df["yaw_deg"].values)
    vn = df["vn"].values
    ve = df["ve"].values
    vup = df["vup"].values

    X = np.column_stack([p, q, r, roll_rad, pitch_rad, yaw_rad, vn, ve, vup])
    state_names = ["p (roll-rate)", "q (pitch-rate)", "r (yaw-rate)", "roll", "pitch", "yaw", "v_north", "v_east", "v_up"]

    # Inputs: [u1, u2, u3, u4]
    u1 = df["u1_pwm"].values
    u2 = df["u2_pwm"].values
    u3 = df["u3_pwm"].values
    u4 = df["u4_pwm"].values
    U = np.column_stack([u1, u2, u3, u4])
    input_names = ["u1 (PWM)", "u2 (PWM)", "u3 (PWM)", "u4 (PWM)"]

    t = df["timestamp"].values
    dt = config.DT

    print("\n" + "=" * 70)
    print("🚀 FITTING SINDy NONLINEAR SYSTEM IDENTIFICATION MODEL...")
    print("=" * 70)
    sindy = SINDyModel(threshold=0.012, max_iter=15)
    sindy.fit(X, U, dt, state_names=state_names)
    sindy_report = sindy.get_equations_report()
    print(sindy_report)

    # Save SINDy report
    sindy_report_path = os.path.join(results_dir, "sindy_discovered_equations.txt")
    with open(sindy_report_path, "w", encoding="utf-8") as f:
        f.write(sindy_report)

    print("\n" + "=" * 70)
    print("🌀 FITTING DMDc (DYNAMIC MODE DECOMPOSITION WITH CONTROL)...")
    print("=" * 70)
    dmdc = DMDcModel(energy_threshold=0.9995)
    dmdc.fit(X, U, dt, state_names=state_names, input_names=input_names)
    dmdc_report = dmdc.get_summary_report()
    print(dmdc_report)

    # Save DMDc report
    dmdc_report_path = os.path.join(results_dir, "dmdc_stability_report.txt")
    with open(dmdc_report_path, "w", encoding="utf-8") as f:
        f.write(dmdc_report)

    print("\n" + "=" * 70)
    print("📈 SIMULATING & VALIDATING DATA-DRIVEN PREDICTIONS...")
    print("=" * 70)
    x0 = X[0]
    X_sindy_sim = sindy.simulate(x0, U, dt)
    X_dmdc_sim = dmdc.simulate(x0, U)

    # Compute Trajectory Integration Positions (North, East, Alt)
    # Ground Truth Positions
    pos_gt_n = np.cumsum(vn * dt)
    pos_gt_e = np.cumsum(ve * dt)
    pos_gt_alt = df["alt"].values

    # SINDy Simulated Positions
    pos_sindy_n = np.cumsum(X_sindy_sim[:, 6] * dt)
    pos_sindy_e = np.cumsum(X_sindy_sim[:, 7] * dt)
    pos_sindy_alt = np.cumsum(X_sindy_sim[:, 8] * dt)

    # DMDc Simulated Positions
    pos_dmdc_n = np.cumsum(X_dmdc_sim[:, 6] * dt)
    pos_dmdc_e = np.cumsum(X_dmdc_sim[:, 7] * dt)
    pos_dmdc_alt = np.cumsum(X_dmdc_sim[:, 8] * dt)

    # Compute Summary Accuracy Metrics
    metrics = []
    for j, name in enumerate(state_names):
        y_true = X[:, j]
        y_sindy = X_sindy_sim[:, j]
        y_dmdc = X_dmdc_sim[:, j]

        r2_sindy = max(0.0, 1.0 - np.sum((y_true - y_sindy)**2) / (np.sum((y_true - np.mean(y_true))**2) + 1e-9))
        r2_dmdc = max(0.0, 1.0 - np.sum((y_true - y_dmdc)**2) / (np.sum((y_true - np.mean(y_true))**2) + 1e-9))
        rmse_sindy = np.sqrt(np.mean((y_true - y_sindy)**2))
        rmse_dmdc = np.sqrt(np.mean((y_true - y_dmdc)**2))

        r2_deriv_sindy = sindy.r2_scores.get(name, 0.0)
        metrics.append({
            "State": name,
            "SINDy Eq R²": f"{r2_deriv_sindy:.4f}",
            "SINDy RMSE": f"{rmse_sindy:.4f}",
            "DMDc Discrete R²": f"{r2_dmdc:.4f}",
            "DMDc RMSE": f"{rmse_dmdc:.4f}",
        })

    metrics_df = pd.DataFrame(metrics)
    print("\n--- QUANTITATIVE BENCHMARK ACCURACY TABLE ---")
    print(metrics_df.to_string(index=False))

    # ==========================================
    # PLOT 1: Spacious 3D & 2D Waypoint Flight Trajectory
    # ==========================================
    fig = plt.figure(figsize=(14, 6.5), dpi=160)

    # 1. Left Subplot: 3D Flight Trajectory
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.plot(pos_gt_e, pos_gt_n, pos_gt_alt, label="True Flight Telemetry", color="#00bcd4", linewidth=2.5)

    # Focus axis bounds tightly around actual flight domain to avoid squishing
    margin = 25.0
    min_e, max_e = min(pos_gt_e) - margin, max(pos_gt_e) + margin
    min_n, max_n = min(pos_gt_n) - margin, max(pos_gt_n) + margin
    min_alt, max_alt = 0.0, max(pos_gt_alt) + 6.0

    mask_s = (pos_sindy_e >= min_e) & (pos_sindy_e <= max_e) & (pos_sindy_n >= min_n) & (pos_sindy_n <= max_n)
    mask_d = (pos_dmdc_e >= min_e) & (pos_dmdc_e <= max_e) & (pos_dmdc_n >= min_n) & (pos_dmdc_n <= max_n)

    if np.any(mask_s):
        ax1.plot(pos_sindy_e[mask_s], pos_sindy_n[mask_s], pos_sindy_alt[mask_s], label="SINDy Discovered Model", color="#e91e63", linestyle="--", linewidth=1.8)
    if np.any(mask_d):
        ax1.plot(pos_dmdc_e[mask_d], pos_dmdc_n[mask_d], pos_dmdc_alt[mask_d], label="DMDc Dynamic Model", color="#ff9800", linestyle=":", linewidth=1.8)

    # Waypoint Markers & Non-overlapping Offset Labels
    from navigation import default_mission
    wps = default_mission()
    offsets = {
        "A": (4, 4),
        "B": (4, -5),
        "C": (-8, -5),
        "D": (-8, 4),
        "HOME": (3, 3)
    }
    for wp in wps:
        ax1.scatter([wp.e], [wp.n], [wp.alt], color="#d32f2f", s=60, marker="^")
        dx, dy = offsets.get(wp.name, (3, 3))
        ax1.text(wp.e + dx, wp.n + dy, wp.alt + 0.6, wp.name, color="#1a237e", fontsize=9, fontweight="bold")

    ax1.set_xlim(min_e, max_e)
    ax1.set_ylim(min_n, max_n)
    ax1.set_zlim(min_alt, max_alt)
    ax1.set_title("3D Flight Trajectory Benchmark", fontsize=11, fontweight="bold", pad=10)
    ax1.set_xlabel("East Position (m)")
    ax1.set_ylabel("North Position (m)")
    ax1.set_zlabel("Altitude (m)")
    ax1.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax1.grid(True, alpha=0.3)

    # 2. Right Subplot: 2D Overhead Mission Projection (North vs East)
    ax2 = fig.add_subplot(122)
    ax2.plot(pos_gt_e, pos_gt_n, label="True Flight Path", color="#00bcd4", linewidth=2.2)
    if np.any(mask_s):
        ax2.plot(pos_sindy_e[mask_s], pos_sindy_n[mask_s], label="SINDy Model", color="#e91e63", linestyle="--", linewidth=1.6)
    if np.any(mask_d):
        ax2.plot(pos_dmdc_e[mask_d], pos_dmdc_n[mask_d], label="DMDc Model", color="#ff9800", linestyle=":", linewidth=1.6)

    # Draw Waypoint Mission Line
    wp_e = [wp.e for wp in wps]
    wp_n = [wp.n for wp in wps]
    ax2.plot(wp_e, wp_n, color="#9c27b0", linestyle="-.", alpha=0.6, label="Waypoint Plan (A-B-C-D-HOME)")

    for wp in wps:
        ax2.scatter(wp.e, wp.n, color="#d32f2f", s=70, marker="^", zorder=5)
        dx, dy = offsets.get(wp.name, (3, 3))
        ax2.annotate(wp.name, (wp.e, wp.n), xytext=(wp.e + dx, wp.n + dy),
                     fontsize=9, fontweight="bold", color="#1a237e",
                     arrowprops=dict(arrowstyle="->", color="#888888", lw=0.8))

    ax2.set_xlim(min_e, max_e)
    ax2.set_ylim(min_n, max_n)
    ax2.set_title("Overhead 2D Mission Map Projection (North vs East)", fontsize=11, fontweight="bold", pad=10)
    ax2.set_xlabel("East Position (m)")
    ax2.set_ylabel("North Position (m)")
    ax2.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax2.grid(True, alpha=0.3)

    fig.suptitle("Quadrotor 6-DOF Flight Dynamics Benchmark: Waypoints A, B, C, D, HOME\n(Ground Truth Telemetry vs. SINDy vs. DMDc Discovered Models)", fontsize=13, fontweight="bold")
    plt.tight_layout()
    p1_path = os.path.join(results_dir, "trajectory_3d_comparison.png")
    fig.savefig(p1_path, bbox_inches="tight")
    plt.close(fig)

    # ==========================================
    # PLOT 2: 6-Panel Time Series Comparison
    # ==========================================
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), dpi=150)
    fig.suptitle("6-DOF Flight Dynamics State Tracking (Ground Truth vs. SINDy vs. DMDc)", fontsize=14, fontweight="bold")

    # 1. Angular Rates (p, q)
    ax = axes[0, 0]
    ax.plot(t, p, label="p (True)", color="black", alpha=0.7)
    ax.plot(t, X_sindy_sim[:, 0], label="p (SINDy)", color="crimson", linestyle="--")
    ax.plot(t, X_dmdc_sim[:, 0], label="p (DMDc)", color="darkorange", linestyle=":")
    ax.set_ylabel("Roll Rate p (rad/s)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 2. Pitch Rate (q)
    ax = axes[0, 1]
    ax.plot(t, q, label="q (True)", color="black", alpha=0.7)
    ax.plot(t, X_sindy_sim[:, 1], label="q (SINDy)", color="crimson", linestyle="--")
    ax.plot(t, X_dmdc_sim[:, 1], label="q (DMDc)", color="darkorange", linestyle=":")
    ax.set_ylabel("Pitch Rate q (rad/s)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 3. Roll & Pitch Angles
    ax = axes[1, 0]
    ax.plot(t, np.degrees(roll_rad), label="Roll (True)", color="blue", alpha=0.7)
    ax.plot(t, np.degrees(X_sindy_sim[:, 3]), label="Roll (SINDy)", color="magenta", linestyle="--")
    ax.plot(t, np.degrees(X_dmdc_sim[:, 3]), label="Roll (DMDc)", color="goldenrod", linestyle=":")
    ax.set_ylabel("Roll Angle (deg)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, np.degrees(pitch_rad), label="Pitch (True)", color="blue", alpha=0.7)
    ax.plot(t, np.degrees(X_sindy_sim[:, 4]), label="Pitch (SINDy)", color="magenta", linestyle="--")
    ax.plot(t, np.degrees(X_dmdc_sim[:, 4]), label="Pitch (DMDc)", color="goldenrod", linestyle=":")
    ax.set_ylabel("Pitch Angle (deg)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 4. Velocities
    ax = axes[2, 0]
    ax.plot(t, vn, label="v_North (True)", color="green", alpha=0.7)
    ax.plot(t, X_sindy_sim[:, 6], label="v_North (SINDy)", color="red", linestyle="--")
    ax.plot(t, X_dmdc_sim[:, 6], label="v_North (DMDc)", color="orange", linestyle=":")
    ax.set_ylabel("North Velocity (m/s)")
    ax.set_xlabel("Time (seconds)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    ax.plot(t, pos_gt_alt, label="Altitude (True)", color="purple", alpha=0.8)
    ax.plot(t, pos_sindy_alt, label="Altitude (SINDy)", color="red", linestyle="--")
    ax.plot(t, pos_dmdc_alt, label="Altitude (DMDc)", color="orange", linestyle=":")
    ax.set_ylabel("Altitude (m)")
    ax.set_xlabel("Time (seconds)")
    ax.legend(loc="lower right", fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    p2_path = os.path.join(results_dir, "state_time_series_comparison.png")
    fig.savefig(p2_path, bbox_inches="tight")
    plt.close(fig)

    # ==========================================
    # PLOT 3: DMDc Eigenvalues & Stability
    # ==========================================
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5), dpi=150)

    # Discrete Eigenvalues on Unit Circle
    theta_circ = np.linspace(0, 2 * np.pi, 200)
    ax1.plot(np.cos(theta_circ), np.sin(theta_circ), 'k--', alpha=0.6, label="Unit Circle (|z| = 1)")
    ax1.scatter(dmdc.eigenvalues.real, dmdc.eigenvalues.imag, color="crimson", s=70, zorder=5, label="DMDc Discrete Eigenvalues")
    ax1.axhline(0, color="gray", linewidth=0.8)
    ax1.axvline(0, color="gray", linewidth=0.8)
    ax1.set_title("Discrete-Time Eigenvalues (z-plane)\n(Stability Criteria: |z| ≤ 1)", fontsize=11, fontweight="bold")
    ax1.set_xlabel("Real(z)")
    ax1.set_ylabel("Imag(z)")
    ax1.axis("equal")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)

    # Continuous Poles
    ax2.scatter(dmdc.continuous_poles.real, dmdc.continuous_poles.imag, color="royalblue", s=70, marker="x", zorder=5, label="Continuous Poles s = ln(z)/dt")
    ax2.axvline(0, color="red", linestyle="--", linewidth=1.2, label="Stability Margin (Re(s) < 0)")
    ax2.axhline(0, color="gray", linewidth=0.8)
    ax2.set_title("Continuous-Time Pole Spectrum (s-plane)\n(Damping & Natural Frequencies)", fontsize=11, fontweight="bold")
    ax2.set_xlabel("Real(s) [Damping Rate]")
    ax2.set_ylabel("Imag(s) [Oscillation Freq rad/s]")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    p3_path = os.path.join(results_dir, "dmdc_eigenvalue_spectrum.png")
    fig.savefig(p3_path, bbox_inches="tight")
    plt.close(fig)

    # ==========================================
    # Format Markdown Table manually
    col_names = list(metrics_df.columns)
    md_table_rows = [
        "| " + " | ".join(col_names) + " |",
        "| " + " | ".join(["---"] * len(col_names)) + " |"
    ]
    for _, row in metrics_df.iterrows():
        md_table_rows.append("| " + " | ".join([str(v) for v in row.values]) + " |")
    metrics_md_table = "\n".join(md_table_rows)

    # Final Report File
    # ==========================================
    summary_md = f"""# Data-Driven Modeling & Simulation (DDMS) Final Report

## Indian Campus Perimeter Flight Analysis
- **Location**: Amrita Coimbatore Campus (10.9001°N, 76.9002°E)
- **Flight Duration**: {df['timestamp'].iloc[-1]:.2f} seconds
- **Sampling Frequency**: 250 Hz ({len(df):,} total samples)
- **Atmospheric Wind**: {df['wind_speed_mps'].mean():.1f} m/s with turbulence & gust bursts

---

## 📊 Summary Accuracy Metrics Table

{metrics_md_table}

---

## 🔬 Discovered SINDy Governing Equations
```
{sindy_report}
```

---

## 🌀 Discovered DMDc System Dynamics
```
{dmdc_report}
```

---

## 🖼️ Generated Visual Artifacts
1. **3D Flight Path Benchmark**: `trajectory_3d_comparison.png`
2. **6-DOF State Tracking Comparison**: `state_time_series_comparison.png`
3. **DMDc Eigenvalue Stability Circle**: `dmdc_eigenvalue_spectrum.png`
"""
    summary_md_path = os.path.join(results_dir, "ddms_final_summary.md")
    with open(summary_md_path, "w", encoding="utf-8") as f:
        f.write(summary_md)

    print(f"\n[DDMS] 🏆 Complete Data-Driven Analysis Pipeline Finished Successfully!")
    print(f"[DDMS] 📁 Artifacts saved in: {os.path.abspath(results_dir)}")
    return results_dir


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run SINDy & DMDc Data-Driven Analysis on 250 Hz Drone Flight Data.")
    parser.add_argument("--csv", type=str, default=None, help="Path to flight telemetry CSV.")
    parser.add_argument("--out", type=str, default=None, help="Output directory for charts and reports.")
    args = parser.parse_args()
    run_data_driven_pipeline(args.csv, args.out)
