"""Patch snippets for voice_controller_v4_gemini.py  (NOT yet tested against a live PX4 -
test each in SITL).  Paste over the matching functions / add the new ones.
"""
import asyncio, json, math, os
from mavsdk.mission import MissionItem, MissionPlan

POINTS_FILE = "saved_points.json"
M_PER_DEG_LAT = 111320.0

# ---- FIX 1: saved points survive restarts (stored as lat, lon, RELATIVE altitude) ----
def load_points(state):
  if os.path.exists(POINTS_FILE):
    state.saved_points = {k: tuple(v) for k, v in json.load(open(POINTS_FILE)).items()}
    print(f">> Loaded {len(state.saved_points)} saved points: {list(state.saved_points)}")

def persist_points(state):
  json.dump(state.saved_points, open(POINTS_FILE, "w"), indent=2)

async def cmd_save_point(drone, state, name):
  pos = await anext(drone.telemetry.position())
  # relative altitude, because the mission uses relative altitude (old code mixed AMSL and relative)
  state.saved_points[name] = (pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m)
  persist_points(state)
  print(f">> SAVED POINT '{name}' {state.saved_points[name]}")
# call load_points(state) once right after `state = VoiceState()` in run_voice_control()

# ---- FIX 2: obstacles anchored to HOME, not to the drone's current position ----
async def get_home(drone):
  h = await anext(drone.telemetry.home())
  return h.latitude_deg, h.longitude_deg

def to_local(lat, lon, o_lat, o_lon):          # -> (east_m, north_m)
  return ((lon - o_lon) * M_PER_DEG_LAT * math.cos(math.radians(o_lat)),
          (lat - o_lat) * M_PER_DEG_LAT)

def to_gps(x, y, o_lat, o_lon):
  return (o_lat + y / M_PER_DEG_LAT,
          o_lon + x / (M_PER_DEG_LAT * math.cos(math.radians(o_lat))))
# In cmd_goto_point_bspline: origin = home; start = to_local(current pos, home);
# goal = to_local(saved point, home); convert path points back with to_gps(..., home).
# obstacles.json coordinates are then metres from HOME and never move.

# ---- FIX 3: 35 waypoints, not 70 items (no per-waypoint "Change speed") ----
def make_item(lat, lon, rel_alt):
  return MissionItem(
      latitude_deg=lat, longitude_deg=lon, relative_altitude_m=float(rel_alt),
      speed_m_s=float("nan"),            # NaN => MAVSDK does not insert DO_CHANGE_SPEED
      is_fly_through=True,
      gimbal_pitch_deg=float("nan"), gimbal_yaw_deg=float("nan"),
      camera_action=MissionItem.CameraAction.NONE,
      loiter_time_s=float("nan"), camera_photo_interval_s=float("nan"),
      acceptance_radius_m=3.0, yaw_deg=float("nan"),
      camera_photo_distance_m=float("nan"),
      vehicle_action=MissionItem.VehicleAction.NONE)
# set cruise speed once instead:  await drone.mission.set_return_to_launch_after_mission(False)
# and PX4 param MPC_XY_CRUISE (e.g. 5 m/s) in the pxh> shell:  param set MPC_XY_CRUISE 5

# ---- FIX 4: upload -> wait until airborne & healthy -> start, with retries ----
async def upload_and_start(drone, mission_items):
  try:
    await drone.mission.clear_mission()
  except Exception:
    pass
  await drone.mission.upload_mission(MissionPlan(mission_items))
  # make sure we are airborne (say 'take off' first, or auto-takeoff here)
  async for in_air in drone.telemetry.in_air():
    if in_air:
      break
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(6)
  async for h in drone.telemetry.health():
    if h.is_global_position_ok and h.is_home_position_ok:
      break
  await drone.mission.set_current_mission_item(0)
  for attempt in range(3):
    try:
      await drone.mission.start_mission()
      print(">> Mission started")
      return True
    except Exception as e:
      print(f"   [start_mission attempt {attempt+1} failed: {e}]")
      await asyncio.sleep(1.5)
  return False

# ---- FIX 5: do NOT upload buildings as INCLUSION fences ----
# Easiest/safest: delete the call `await upload_obstacles_to_qgc(...)` and instead draw the
# buildings in QGC (Plan > Fence > Polygon > set to Exclusion) or only call
# drone.geofence.clear_geofence() at start-up so old fences from earlier runs disappear.
# If you still upload via code, use FenceType.EXCLUSION and NO master inclusion polygon:
#     Polygon(corners, FenceType.EXCLUSION)
# For the demo you can also set in pxh>:   param set GF_ACTION 0   (0 = None -> warning only)
