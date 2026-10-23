"""Offline Alexa-style voice-controlled drone commander for PX4 SITL.

Full command set restored + B-spline dynamic obstacle avoidance routing.
"""

import asyncio
import csv
from datetime import datetime
import json
import math
import queue
import sys

from mavsdk import System
from mavsdk.action import OrbitYawBehavior
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.offboard import Attitude, OffboardError, VelocityBodyYawspeed
from path_planner import SplinePathPlanner
import sounddevice as sd
from vosk import KaldiRecognizer, Model

# Single port setup matching PX4 -o 14530
VOICE_PORT = 14530
INTERNAL_GRPC_PORT = 50052
SAMPLE_RATE = 16000

ALT_STEP_M = 2.0
MIN_REL_ALT_M = 2.0

MOVE_SPEED_MS = 2.0
MOVE_DURATION_S = 2.0

SWEEP_AMPLITUDE_DEG = 10.0
SWEEP_DURATION_S = 6.0
SWEEP_FREQ_HZ = 0.5
HOVER_THRUST_GUESS = 0.5

ORBIT_RADIUS_M = 10.0
ORBIT_SPEED_MS = 3.0

audio_queue = queue.Queue()


def audio_callback(indata, frames, time_info, status):
  if status:
    print(status, file=sys.stderr)
  audio_queue.put(bytes(indata))


def log_operation(action, status):
  with open("operation_log.csv", mode="a", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([datetime.now().isoformat(), action, status])


class VoiceState:

  def __init__(self):
    self.saved_points = {}  # name -> (lat, lon, absolute_altitude_m)
    self.active = False  # Alexa wake state (start/end)


# ---------- System / Basic Flight ----------


async def cmd_arm(drone):
  print(">> Executing: ARM")
  await drone.action.arm()


async def cmd_disarm(drone):
  print(">> Executing: DISARM")
  await drone.action.disarm()


async def cmd_takeoff(drone):
  print(">> Executing: TAKEOFF")
  await drone.action.arm()
  await drone.action.takeoff()


async def cmd_land(drone):
  print(">> Executing: LAND")
  await drone.action.land()


async def cmd_hold(drone):
  print(">> Executing: HOLD")
  await drone.action.hold()


async def cmd_return_home(drone):
  print(">> Executing: RETURN TO LAUNCH")
  await drone.action.return_to_launch()


# ---------- Dynamic Waypoints with B-Spline Obstacle Avoidance ----------


async def cmd_save_point(drone, state, name):
  pos = await anext(drone.telemetry.position())
  state.saved_points[name] = (
      pos.latitude_deg,
      pos.longitude_deg,
      pos.absolute_altitude_m,
  )
  print(
      f">> Executing: SAVE POINT '{name}' ({pos.latitude_deg:.6f},"
      f" {pos.longitude_deg:.6f}, {pos.absolute_altitude_m:.1f} m)"
  )


async def cmd_goto_point_bspline(drone, state, name):
  print(f">> Executing: GO TO '{name}' (via B-Spline Obstacle Avoidance)")
  if name not in state.saved_points:
    print(
        f"   [No saved point named '{name}' -- say 'save point {name}' first]"
    )
    return

  target_lat, target_lon, target_alt = state.saved_points[name]

  # Get current position
  pos = await anext(drone.telemetry.position())
  curr_lat, curr_lon = pos.latitude_deg, pos.longitude_deg
  curr_alt = pos.relative_altitude_m

  # Convert GPS delta to local relative meters (x: East, y: North)
  delta_y = (target_lat - curr_lat) * 111320.0
  delta_x = (
      (target_lon - curr_lon)
      * 40075000.0
      * math.cos(math.radians(curr_lat))
      / 360.0
  )

  start_pt = [0.0, 0.0, curr_alt]
  end_pt = [delta_x, delta_y, target_alt]

  # Generate B-spline path avoiding virtual buildings in obstacles.json
  planner = SplinePathPlanner()
  path = planner.generate_bspline_path(start_pt, end_pt)

  # Convert local spline points back to MAVSDK Mission Waypoints
  mission_items = []
  for pt in path:
    lat = curr_lat + (pt[1] / 111320.0)
    lon = curr_lon + (
        pt[0] / (40075000.0 * math.cos(math.radians(curr_lat)) / 360.0)
    )
    mission_items.append(
        MissionItem(
            lat,
            lon,
            pt[2],
            5.0,
            True,
            float("nan"),
            float("nan"),
            MissionItem.CameraAction.NONE,
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
            float("nan"),
        )
    )

  plan = MissionPlan(mission_items)
  await drone.mission.upload_mission(plan)
  await drone.action.arm()
  await drone.mission.start_mission()
  print(f">> B-Spline mission to '{name}' uploaded and active in QGC!")


# ---------- Altitude ----------


async def cmd_ascend(drone):
  pos = await anext(drone.telemetry.position())
  target_alt = pos.absolute_altitude_m + ALT_STEP_M
  print(f">> Executing: ASCEND (target abs alt {target_alt:.1f} m)")
  await drone.action.goto_location(
      pos.latitude_deg, pos.longitude_deg, target_alt, float("nan")
  )


async def cmd_descend(drone):
  pos = await anext(drone.telemetry.position())
  new_rel_alt = max(MIN_REL_ALT_M, pos.relative_altitude_m - ALT_STEP_M)
  target_alt = pos.absolute_altitude_m - pos.relative_altitude_m + new_rel_alt
  print(f">> Executing: DESCEND (target abs alt {target_alt:.1f} m)")
  await drone.action.goto_location(
      pos.latitude_deg, pos.longitude_deg, target_alt, float("nan")
  )


# ---------- Vector Control (Offboard Velocity Pulse) ----------


async def move_pulse(drone, forward_ms, right_ms):
  print(
      f">> Executing: MOVE (forward={forward_ms}, right={right_ms} m/s for"
      f" {MOVE_DURATION_S}s)"
  )
  try:
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(forward_ms, right_ms, 0.0, 0.0)
    )
    try:
      await drone.offboard.start()
    except OffboardError as e:
      print(f"   [Offboard start failed, aborting move: {e}]")
      return

    loop = asyncio.get_event_loop()
    end_time = loop.time() + MOVE_DURATION_S
    while loop.time() < end_time:
      await drone.offboard.set_velocity_body(
          VelocityBodyYawspeed(forward_ms, right_ms, 0.0, 0.0)
      )
      await asyncio.sleep(0.1)
  finally:
    try:
      await drone.offboard.set_velocity_body(
          VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
      )
      await asyncio.sleep(0.2)
      await drone.offboard.stop()
    except Exception as e:
      print(f"   [Offboard stop issue (may be harmless): {e}]")
    try:
      await drone.action.hold()
    except Exception as e:
      print(f"   [Hold after move failed: {e}]")


# ---------- System ID Excitation (Offboard Attitude Sweep) ----------


async def attitude_sweep(drone, axis):
  print(
      f">> Executing: {axis.upper()} SWEEP (system ID excitation,"
      f" {SWEEP_DURATION_S}s)"
  )
  try:
    att = await anext(drone.telemetry.attitude_euler())
    current_yaw = att.yaw_deg
  except Exception:
    current_yaw = 0.0

  try:
    await drone.offboard.set_attitude(
        Attitude(0.0, 0.0, current_yaw, HOVER_THRUST_GUESS)
    )
    try:
      await drone.offboard.start()
    except OffboardError as e:
      print(f"   [Offboard start failed, aborting sweep: {e}]")
      return

    loop = asyncio.get_event_loop()
    start_t = loop.time()
    while loop.time() - start_t < SWEEP_DURATION_S:
      t = loop.time() - start_t
      angle = SWEEP_AMPLITUDE_DEG * math.sin(2 * math.pi * SWEEP_FREQ_HZ * t)
      roll = angle if axis == "roll" else 0.0
      pitch = angle if axis == "pitch" else 0.0
      await drone.offboard.set_attitude(
          Attitude(roll, pitch, current_yaw, HOVER_THRUST_GUESS)
      )
      await asyncio.sleep(0.1)
  finally:
    try:
      await drone.offboard.set_attitude(
          Attitude(0.0, 0.0, current_yaw, HOVER_THRUST_GUESS)
      )
      await asyncio.sleep(0.2)
      await drone.offboard.stop()
    except Exception as e:
      print(f"   [Offboard stop issue (may be harmless): {e}]")
    try:
      await drone.action.hold()
    except Exception as e:
      print(f"   [Hold after sweep failed: {e}]")


# ---------- Complex Trajectory ----------


async def cmd_circle(drone):
  print(">> Executing: CIRCLE MANEUVER (one revolution)")
  try:
    pos = await anext(drone.telemetry.position())
    await drone.action.do_orbit(
        radius_m=ORBIT_RADIUS_M,
        velocity_ms=ORBIT_SPEED_MS,
        yaw_behavior=OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER,
        latitude_deg=pos.latitude_deg,
        longitude_deg=pos.longitude_deg,
        absolute_altitude_m=pos.absolute_altitude_m,
    )
  except Exception as e:
    print(f"   [Orbit command failed/unsupported: {e}]")
    return

  duration = (2 * math.pi * ORBIT_RADIUS_M) / ORBIT_SPEED_MS
  print(f"   Orbiting for {duration:.1f}s...")
  await asyncio.sleep(duration)
  try:
    await drone.action.hold()
  except Exception as e:
    print(f"   [Hold after circle failed: {e}]")


# ---------- Command Parsing / Dispatch ----------


def strip_leading_the(name):
  return name[4:] if name.startswith("the ") else name


async def dispatch_command(drone, state, text):
  try:
    # Alexa-style wake / sleep word control
    if "start" in text:
      state.active = True
      print("\n[ALEXA]: Wake word detected. LISTENING ACTIVE.")
      log_operation("WAKE_START", "SUCCESS")
      return
    if "end" in text:
      state.active = False
      print("\n[ALEXA]: Sleep word detected. LISTENING PAUSED.")
      log_operation("WAKE_END", "SUCCESS")
      return

    if not state.active:
      return

    print(f"[Heard Command]: '{text}'")

    if text.startswith("save point "):
      name = strip_leading_the(text[len("save point ") :].strip())
      await cmd_save_point(drone, state, name)
      log_operation(f"SAVE_POINT_{name}", "EXECUTED")

    elif text.startswith("go to ") or text.startswith("fly to "):
      prefix_len = len("go to ") if text.startswith("go to ") else len("fly to ")
      name = strip_leading_the(text[prefix_len:].strip())
      # Routes via SplinePathPlanner & obstacles.json
      await cmd_goto_point_bspline(drone, state, name)
      log_operation(f"GOTO_BSPLINE_{name}", "EXECUTED")

    elif "pitch sweep" in text:
      await attitude_sweep(drone, "pitch")
      log_operation("PITCH_SWEEP", "EXECUTED")

    elif "roll sweep" in text:
      await attitude_sweep(drone, "roll")
      log_operation("ROLL_SWEEP", "EXECUTED")

    elif "circle maneuver" in text or "circle" in text:
      await cmd_circle(drone)
      log_operation("CIRCLE", "EXECUTED")

    elif "move forward" in text or text.strip() == "forward":
      await move_pulse(drone, MOVE_SPEED_MS, 0.0)
      log_operation("MOVE_FORWARD", "EXECUTED")

    elif "move backward" in text or "move back" in text or "backward" in text:
      await move_pulse(drone, -MOVE_SPEED_MS, 0.0)
      log_operation("MOVE_BACKWARD", "EXECUTED")

    elif "move left" in text or text.strip() == "left":
      await move_pulse(drone, 0.0, -MOVE_SPEED_MS)
      log_operation("MOVE_LEFT", "EXECUTED")

    elif "move right" in text or text.strip() == "right":
      await move_pulse(drone, 0.0, MOVE_SPEED_MS)
      log_operation("MOVE_RIGHT", "EXECUTED")

    elif "ascend" in text:
      await cmd_ascend(drone)
      log_operation("ASCEND", "EXECUTED")

    elif "descend" in text:
      await cmd_descend(drone)
      log_operation("DESCEND", "EXECUTED")

    elif "disarm" in text:
      await cmd_disarm(drone)
      log_operation("DISARM", "EXECUTED")

    elif "arm" in text:
      await cmd_arm(drone)
      log_operation("ARM", "EXECUTED")

    elif "take off" in text or "takeoff" in text:
      await cmd_takeoff(drone)
      log_operation("TAKEOFF", "EXECUTED")

    elif "land" in text:
      await cmd_land(drone)
      log_operation("LAND", "EXECUTED")

    elif "return home" in text or "return to launch" in text or "rth" in text:
      await cmd_return_home(drone)
      log_operation("RETURN_HOME", "EXECUTED")

    elif "hold" in text or "stop" in text:
      await cmd_hold(drone)
      log_operation("HOLD", "EXECUTED")

    else:
      print(f"   [No matching command for: '{text}']")
  except Exception as e:
    print(f"   [Command failed, continuing to listen: {e}]")


async def run_voice_control():
  drone = System(port=INTERNAL_GRPC_PORT)
  print(f"Connecting to PX4 on port {VOICE_PORT}...")
  await drone.connect(system_address=f"udpin://0.0.0.0:{VOICE_PORT}")

  print("Waiting for drone connection...")
  async for conn_state in drone.core.connection_state():
    if conn_state.is_connected:
      print("Voice Control Connected to PX4!")
      break

  state = VoiceState()

  print("Loading Vosk model (en-us)...")
  model = Model(lang="en-us")
  recognizer = KaldiRecognizer(model, SAMPLE_RATE)

  loop = asyncio.get_event_loop()

  def start_audio_stream():
    stream = sd.RawInputStream(
        samplerate=SAMPLE_RATE,
        blocksize=8000,
        dtype="int16",
        channels=1,
        callback=audio_callback,
    )
    stream.start()
    return stream

  stream = await loop.run_in_executor(None, start_audio_stream)

  print("\n--- VOICE CONTROL ACTIVE ---")
  print("Say 'start' to activate, 'end' to pause listening.")
  print("arm | disarm | take off | land | hold/stop | return home")
  print("save point <name> | go to <name> | fly to <name> (uses B-spline)")
  print("move forward/backward/left/right | ascend | descend")
  print("pitch sweep | roll sweep | circle maneuver | exit\n")

  try:
    while True:
      data = await loop.run_in_executor(None, audio_queue.get)
      if recognizer.AcceptWaveform(data):
        result = json.loads(recognizer.Result())
        text = result.get("text", "").lower()

        if not text:
          continue

        if "exit" in text and state.active:
          print(
              "Exiting voice control (drone untouched -- say 'land' first if"
              " flying)."
          )
          break

        await dispatch_command(drone, state, text)

      await asyncio.sleep(0.01)
  except KeyboardInterrupt:
    print("\nVoice Control Shutting Down.")
  finally:
    stream.stop()
    stream.close()


if __name__ == "__main__":
  asyncio.run(run_voice_control())
