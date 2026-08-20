"""
High-Rate (250 Hz) Telemetry Logger for Data-Driven Modeling and Simulation (DDMS).
Captures full 6-DOF states, IMU accelerations, gyros, motor commands, and atmospheric disturbances.
"""

import os
import csv
import math
import numpy as np

import config


class TelemetryLogger:
    def __init__(self, output_dir=None, filename=None):
        self.base_dir = output_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", config.TELEMETRY_LOG_DIR)
        self.filename = filename or config.TELEMETRY_CSV_FILE
        self.filepath = os.path.join(self.base_dir, self.filename)
        
        self.headers = [
            "timestamp",
            "lat",
            "lon",
            "alt",
            "climb_rate",
            "vn",
            "ve",
            "vup",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "p_rads",
            "q_rads",
            "r_rads",
            "ax_body",
            "ay_body",
            "az_body",
            "ax_world",
            "ay_world",
            "az_world",
            "u1_pwm",
            "u2_pwm",
            "u3_pwm",
            "u4_pwm",
            "thrust_total_N",
            "battery_pct",
            "wind_speed_mps",
            "wind_heading_deg",
        ]
        
        self.buffer = []
        self.is_logging = True
        self.sample_count = 0
        self.last_saved_path = None
        os.makedirs(self.base_dir, exist_ok=True)

    def reset(self):
        self.buffer = []
        self.sample_count = 0

    def record_step(self, drone):
        if not self.is_logging or not drone.armed:
            return

        row = [
            round(drone.t, 4),
            round(drone.gps_lat, 7),
            round(drone.gps_lon, 7),
            round(drone.alt, 4),
            round(drone.vel_up, 4),
            round(drone.vel_n, 4),
            round(drone.vel_e, 4),
            round(drone.vel_up, 4),
            round(math.degrees(drone.roll), 3),
            round(math.degrees(drone.pitch), 3),
            round(drone.heading_deg, 3),
            round(drone.p, 5),
            round(drone.q, 5),
            round(drone.r, 5),
            round(getattr(drone, "ax_body", 0.0), 4),
            round(getattr(drone, "ay_body", 0.0), 4),
            round(getattr(drone, "az_body", -config.G), 4),
            round(getattr(drone, "ax_w", 0.0), 4),
            round(getattr(drone, "ay_w", 0.0), 4),
            round(getattr(drone, "az_w", 0.0), 4),
            round(drone.motors[0], 4),
            round(drone.motors[1], 4),
            round(drone.motors[2], 4),
            round(drone.motors[3], 4),
            round(getattr(drone, "thrust_total", 0.0), 4),
            round(drone.battery, 2),
            round(drone.wind_speed, 2),
            round(drone.wind_heading_deg, 1),
        ]
        self.buffer.append(row)
        self.sample_count += 1

    def save_to_csv(self, custom_path=None):
        if not self.buffer:
            return None
        
        target_path = custom_path or self.filepath
        target_dir = os.path.dirname(os.path.abspath(target_path))
        os.makedirs(target_dir, exist_ok=True)

        with open(target_path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(self.headers)
            writer.writerows(self.buffer)

        self.last_saved_path = target_path
        return target_path

    def to_numpy_dict(self):
        if not self.buffer:
            return {}
        arr = np.array(self.buffer)
        return {h: arr[:, i] for i, h in enumerate(self.headers)}
