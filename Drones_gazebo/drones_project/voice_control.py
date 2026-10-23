"""
Offline voice-controlled drone commander for PX4 SITL.

PX4-side setup (at the pxh> prompt), AFTER PX4 has already auto-started
its own 14550/14540 streams by default:

    mavlink start -u 14535 -o 14530 -r 4000 -t 127.0.0.1

Note the asymmetric ports: PX4 binds its own local port 14535 (arbitrary,
unused elsewhere) and sends its stream OUT to 14530 -- the port this
script listens on. This mirrors the exact pattern PX4's own default
14550/14540 streams already use (different local vs remote port), which
is what avoids the "Address already in use" collision entirely.

Do NOT change this script's VOICE_PORT to match PX4's -u value -- it
must match PX4's -o (remote) value instead.

Run alongside data_logger.py -- this script does not touch or kill any
other MAVSDK process, so it's safe to run at the same time.
"""

import asyncio
import json
import queue
import sys

from mavsdk import System
import sounddevice as sd
from vosk import KaldiRecognizer, Model


VOICE_PORT = 14530  # must match PX4's -o value, NOT its -u value
SAMPLE_RATE = 16000

audio_queue = queue.Queue()


def audio_callback(indata, frames, time_info, status):
    if status:
        print(status, file=sys.stderr)
    audio_queue.put(bytes(indata))


async def run_voice_control():
    drone = System()
    print(f"Connecting to PX4 on port {VOICE_PORT}...")
    await drone.connect(system_address=f"udpin://0.0.0.0:{VOICE_PORT}")

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Voice Control Connected to PX4!")
            break

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
    print("Commands: 'take off', 'land', 'hold'\n")

    try:
        while True:
            data = await loop.run_in_executor(None, audio_queue.get)
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                text = result.get("text", "").lower()

                if not text:
                    continue

                print(f"[Heard]: {text}")

                if "take off" in text or "takeoff" in text:
                    print(">> Executing: ARM & TAKEOFF")
                    await drone.action.arm()
                    await drone.action.takeoff()

                elif "land" in text:
                    print(">> Executing: LAND")
                    await drone.action.land()

                elif "hold" in text or "stop" in text:
                    print(">> Executing: HOLD POSITION")
                    await drone.action.hold()

            await asyncio.sleep(0.01)
    except KeyboardInterrupt:
        print("\nVoice Control Shutting Down.")
    finally:
        stream.stop()
        stream.close()


if __name__ == "__main__":
    asyncio.run(run_voice_control())
