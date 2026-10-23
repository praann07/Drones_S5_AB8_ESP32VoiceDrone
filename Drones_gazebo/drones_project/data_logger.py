import asyncio
import csv
import os
import subprocess
import time

# Clean orphaned servers before initializing MAVSDK
subprocess.run(
    ["pkill", "-9", "-f", "mavsdk_server"],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
)

from mavsdk import System

LOG_PORT = 14540
INTERNAL_GRPC_PORT = 50051
OUTPUT_FILE = "flight_data.csv"


async def log_telemetry():
  # Isolated internal gRPC port prevents conflict with voice_control.py
  drone = System(port=INTERNAL_GRPC_PORT)
  await drone.connect(system_address=f"udpin://0.0.0.0:{LOG_PORT}")

  print(f"Waiting for telemetry connection on port {LOG_PORT}...")
  async for state in drone.core.connection_state():
    if state.is_connected:
      print(f"Data Logger Connected! Logging to '{OUTPUT_FILE}'...")
      break

  latest_att, latest_vel, latest_imu = None, None, None

  async def update_att():
    nonlocal latest_att
    async for a in drone.telemetry.attitude_euler():
      latest_att = a

  async def update_vel():
    nonlocal latest_vel
    async for v in drone.telemetry.velocity_ned():
      latest_vel = v

  async def update_imu():
    nonlocal latest_imu
    async for i in drone.telemetry.imu():
      latest_imu = i

  tasks = [
      asyncio.create_task(update_att()),
      asyncio.create_task(update_vel()),
      asyncio.create_task(update_imu()),
  ]

  await asyncio.sleep(1)

  with open(OUTPUT_FILE, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow([
        "timestamp",
        "roll",
        "pitch",
        "yaw",
        "vx",
        "vy",
        "vz",
        "ax",
        "ay",
        "az",
        "p",
        "q",
        "r",
    ])

    start_time = time.time()
    print("Logging active... Press Ctrl+C to stop.")

    try:
      while True:
        if latest_att and latest_vel and latest_imu:
          t = time.time() - start_time
          writer.writerow([
              t,
              latest_att.roll_deg,
              latest_att.pitch_deg,
              latest_att.yaw_deg,
              latest_vel.north_m_s,
              latest_vel.east_m_s,
              latest_vel.down_m_s,
              latest_imu.acceleration_frd.forward_m_s2,
              latest_imu.acceleration_frd.right_m_s2,
              latest_imu.acceleration_frd.down_m_s2,
              latest_imu.angular_velocity_frd.forward_rad_s,
              latest_imu.angular_velocity_frd.right_rad_s,
              latest_imu.angular_velocity_frd.down_rad_s,
          ])
        await asyncio.sleep(0.02)
    except asyncio.CancelledError:
      pass
    finally:
      for task in tasks:
        task.cancel()


if __name__ == "__main__":
  try:
    asyncio.run(log_telemetry())
  except KeyboardInterrupt:
    print(f"\nSaved dataset to '{OUTPUT_FILE}'.")
  finally:
    subprocess.run(
        ["pkill", "-9", "-f", "mavsdk_server"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
