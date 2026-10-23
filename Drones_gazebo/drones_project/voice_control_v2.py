"""
Offline voice-controlled drone commander for PX4 SITL -- full command set.

PX4-side setup (at the pxh> prompt), AFTER PX4 has already auto-started
its own 14550/14540 streams by default:

    mavlink start -u 14535 -o 14530 -r 4000 -t 127.0.0.1

(Same asymmetric-port setup that fixed the connection -- unchanged.)

Commands:
    "arm" / "disarm"                          -> toggle motor arming
    "take off" / "land" / "hold" / "stop"      -> basic flight
    "save point <name>"                        -> remember current position
    "go to <name>" / "fly to <name>"           -> fly to a saved position
    "move forward/backward/left/right"         -> 2-second velocity pulse
    "ascend" / "descend"                       -> altitude step
    "pitch sweep" / "roll sweep"               -> sinusoidal excitation for
                                                    system identification
    "circle maneuver"                          -> one 360 deg orbit, then hold
    "exit"                                      -> stop listening only
                                                    (does not touch the drone)

Design principle: every command handler runs inside dispatch_command()'s
try/except, and the two maneuvers that use Offboard mode (move pulses,
attitude sweeps) each wrap their own offboard.start()/stop() in a
try/finally that always falls back to action.hold(). A failed or
unsupported command prints an error and the script keeps listening --
it never crashes or stalls the whole session.
"""

import asyncio
import json
import math
import queue
import sys

from mavsdk import System
from mavsdk.action import OrbitYawBehavior
from mavsdk.offboard import Attitude, OffboardError, VelocityBodyYawspeed
import sounddevice as sd
from vosk import KaldiRecognizer, Model


VOICE_PORT = 14530  # must match PX4's -o value, NOT its -u value
SAMPLE_RATE = 16000

ALT_STEP_M = 2.0
MIN_REL_ALT_M = 2.0

MOVE_SPEED_MS = 2.0
MOVE_DURATION_S = 2.0

SWEEP_AMPLITUDE_DEG = 10.0
SWEEP_DURATION_S = 6.0
SWEEP_FREQ_HZ = 0.5
# Rough normalized-thrust guess to roughly hold altitude during a sweep on
# the default x500 SITL model. Not exact -- expect some altitude drift
# during the maneuver; sweeps are kept short specifically to limit this.
HOVER_THRUST_GUESS = 0.5

ORBIT_RADIUS_M = 10.0
ORBIT_SPEED_MS = 3.0

audio_queue = queue.Queue()


def audio_callback(indata, frames, time_info, status):
    if status:
        print(status, file=sys.stderr)
    audio_queue.put(bytes(indata))


class VoiceState:
    def __init__(self):
        self.saved_points = {}  # name -> (lat, lon, absolute_altitude_m)


# ---------- System / basic flight ----------

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


# ---------- Dynamic waypoints ----------

async def cmd_save_point(drone, state, name):
    pos = await anext(drone.telemetry.position())
    state.saved_points[name] = (
        pos.latitude_deg,
        pos.longitude_deg,
        pos.absolute_altitude_m,
    )
    print(f">> Executing: SAVE POINT '{name}' "
          f"({pos.latitude_deg:.6f}, {pos.longitude_deg:.6f}, {pos.absolute_altitude_m:.1f} m)")


async def cmd_goto_point(drone, state, name):
    print(f">> Executing: GO TO '{name}'")
    if name not in state.saved_points:
        print(f"   [No saved point named '{name}' -- say 'save point {name}' first]")
        return
    lat, lon, alt = state.saved_points[name]
    await drone.action.goto_location(lat, lon, alt, float("nan"))


# ---------- Altitude ----------

async def cmd_ascend(drone):
    pos = await anext(drone.telemetry.position())
    target_alt = pos.absolute_altitude_m + ALT_STEP_M
    print(f">> Executing: ASCEND (target abs alt {target_alt:.1f} m)")
    await drone.action.goto_location(pos.latitude_deg, pos.longitude_deg, target_alt, float("nan"))


async def cmd_descend(drone):
    pos = await anext(drone.telemetry.position())
    new_rel_alt = max(MIN_REL_ALT_M, pos.relative_altitude_m - ALT_STEP_M)
    target_alt = pos.absolute_altitude_m - pos.relative_altitude_m + new_rel_alt
    print(f">> Executing: DESCEND (target abs alt {target_alt:.1f} m)")
    await drone.action.goto_location(pos.latitude_deg, pos.longitude_deg, target_alt, float("nan"))


# ---------- Vector control (Offboard velocity pulse) ----------

async def move_pulse(drone, forward_ms, right_ms):
    print(f">> Executing: MOVE (forward={forward_ms}, right={right_ms} m/s for {MOVE_DURATION_S}s)")
    try:
        # PX4 requires an initial setpoint before offboard.start() succeeds.
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
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.2)
            await drone.offboard.stop()
        except Exception as e:
            print(f"   [Offboard stop issue (may be harmless): {e}]")
        try:
            await drone.action.hold()
        except Exception as e:
            print(f"   [Hold after move failed: {e}]")


# ---------- System ID excitation (Offboard attitude sweep) ----------

async def attitude_sweep(drone, axis):
    print(f">> Executing: {axis.upper()} SWEEP (system ID excitation, {SWEEP_DURATION_S}s)")
    try:
        att = await anext(drone.telemetry.attitude_euler())
        current_yaw = att.yaw_deg
    except Exception:
        current_yaw = 0.0

    try:
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, current_yaw, HOVER_THRUST_GUESS))
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
            await drone.offboard.set_attitude(Attitude(roll, pitch, current_yaw, HOVER_THRUST_GUESS))
            await asyncio.sleep(0.1)
    finally:
        try:
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, current_yaw, HOVER_THRUST_GUESS))
            await asyncio.sleep(0.2)
            await drone.offboard.stop()
        except Exception as e:
            print(f"   [Offboard stop issue (may be harmless): {e}]")
        try:
            await drone.action.hold()
        except Exception as e:
            print(f"   [Hold after sweep failed: {e}]")


# ---------- Complex trajectory ----------

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


# ---------- Command parsing/dispatch ----------

def strip_leading_the(name):
    return name[4:] if name.startswith("the ") else name


async def dispatch_command(drone, state, text):
    try:
        if text.startswith("save point "):
            name = strip_leading_the(text[len("save point "):].strip())
            await cmd_save_point(drone, state, name)

        elif text.startswith("go to ") or text.startswith("fly to "):
            prefix_len = len("go to ") if text.startswith("go to ") else len("fly to ")
            name = strip_leading_the(text[prefix_len:].strip())
            await cmd_goto_point(drone, state, name)

        elif "pitch sweep" in text:
            await attitude_sweep(drone, "pitch")

        elif "roll sweep" in text:
            await attitude_sweep(drone, "roll")

        elif "circle maneuver" in text or "circle" in text:
            await cmd_circle(drone)

        elif "move forward" in text or text.strip() == "forward":
            await move_pulse(drone, MOVE_SPEED_MS, 0.0)

        elif "move backward" in text or "move back" in text or "backward" in text:
            await move_pulse(drone, -MOVE_SPEED_MS, 0.0)

        elif "move left" in text or text.strip() == "left":
            await move_pulse(drone, 0.0, -MOVE_SPEED_MS)

        elif "move right" in text or text.strip() == "right":
            await move_pulse(drone, 0.0, MOVE_SPEED_MS)

        elif "ascend" in text:
            await cmd_ascend(drone)

        elif "descend" in text:
            await cmd_descend(drone)

        elif "disarm" in text:
            await cmd_disarm(drone)

        elif "arm" in text:
            await cmd_arm(drone)

        elif "take off" in text or "takeoff" in text:
            await cmd_takeoff(drone)

        elif "land" in text:
            await cmd_land(drone)

        elif "return home" in text or "return to launch" in text or "rth" in text:
            await cmd_return_home(drone)

        elif "hold" in text or "stop" in text:
            await cmd_hold(drone)

        else:
            print(f"   [No matching command for: '{text}']")
    except Exception as e:
        print(f"   [Command failed, continuing to listen: {e}]")


async def run_voice_control():
    drone = System()
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
    print("arm | disarm | take off | land | hold/stop | return home")
    print("save point <name> | go to <name> | fly to <name>")
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

                print(f"[Heard]: {text}")

                if "exit" in text:
                    print("Exiting voice control (drone untouched -- say 'land' first if flying).")
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
