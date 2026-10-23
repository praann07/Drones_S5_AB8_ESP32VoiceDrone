#!/usr/bin/env python3
"""
Voice Control for PX4 SITL using Vosk
"""

import asyncio
import os
import sys
import json
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
from mavsdk import System

# Force mavsdk_server to use a specific port
os.environ["MAVSDK_SERVER_ADDRESS"] = "127.0.0.1:50051"
# Configuration
VOICE_PORT = 14531
AUDIO_SAMPLE_RATE = 16000
AUDIO_CHANNELS = 1
AUDIO_BLOCKSIZE = 8000

# Audio queue for processing
audio_queue = queue.Queue()

def audio_callback(indata, frames, time, status):
    """Callback for sounddevice to capture microphone audio"""
    if status:
        print(f"Audio error: {status}", file=sys.stderr)
    audio_queue.put(bytes(indata))

async def run_voice_control():
    """Main voice control loop"""
    
    print(f"Connecting to PX4 on port {VOICE_PORT}...")
    
    # Initialize drone connection
    drone = System()
    
    # Use udpout to connect to PX4's listening port
    await drone.connect(
        system_address=f"udp://127.0.0.1:{VOICE_PORT}"
    )
    
    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Voice Control Connected to PX4!")
            break
    
    # Load Vosk model
    print("Loading Vosk model (en-us)...")
    model_path = os.path.expanduser("~/.cache/vosk/vosk-model-small-en-us-0.15")
    
    # Download model if not exists
    if not os.path.exists(model_path):
        import urllib.request
        import zipfile
        import shutil
        
        url = "https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip"
        zip_path = "/tmp/vosk-model.zip"
        
        print("Downloading Vosk model...")
        urllib.request.urlretrieve(url, zip_path)
        
        print("Extracting model...")
        with zipfile.ZipFile(zip_path, 'r') as zip_ref:
            zip_ref.extractall(os.path.dirname(model_path))
        
        os.remove(zip_path)
    
    # Initialize recognizer
    model = Model(model_path)
    recognizer = KaldiRecognizer(model, AUDIO_SAMPLE_RATE)
    
    # Start microphone stream
    print("\n--- VOICE CONTROL ACTIVE ---")
    print("Commands: 'take off', 'land', 'hold'")
    print("Say a command...\n")
    
    stream = sd.RawInputStream(
        samplerate=AUDIO_SAMPLE_RATE,
        channels=AUDIO_CHANNELS,
        blocksize=AUDIO_BLOCKSIZE,
        dtype='int16',
        callback=audio_callback
    )
    
    with stream:
        while True:
            # Get audio data from queue
            try:
                data = audio_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            
            # Process audio with Vosk
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                text = result.get("text", "").lower()
                
                if text:
                    print(f"[Heard]: {text}")
                    
                    # Command processing
                    if "take off" in text or "takeoff" in text:
                        print(">> Executing: ARM & TAKEOFF")
                        try:
                            await drone.action.arm()
                            await drone.action.takeoff()
                            print("✓ Takeoff command sent")
                        except Exception as e:
                            print(f"✗ Takeoff failed: {e}")
                    
                    elif "land" in text:
                        print(">> Executing: LAND")
                        try:
                            await drone.action.land()
                            print("✓ Land command sent")
                        except Exception as e:
                            print(f"✗ Land failed: {e}")
                    
                    elif "hold" in text or "loiter" in text:
                        print(">> Executing: HOLD")
                        try:
                            await drone.action.hold()
                            print("✓ Hold command sent")
                        except Exception as e:
                            print(f"✗ Hold failed: {e}")
                    
                    elif "stop" in text or "exit" in text or "quit" in text:
                        print(">> Exiting voice control")
                        break

if __name__ == "__main__":
    try:
        asyncio.run(run_voice_control())
    except KeyboardInterrupt:
        print("\nVoice control stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
