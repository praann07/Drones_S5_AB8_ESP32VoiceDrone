"""
Quick Standalone Microphone & Vosk Voice Test Script
Run this to test your microphone and see recognized words in real-time.
"""

import os
import sys
import queue
import time

try:
    import sounddevice as sd
    import vosk
    import numpy as np
except ImportError as e:
    print(f"Missing dependency: {e}")
    sys.exit(1)

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drone_sim", "models", "vosk-model-small-en-us-0.15")

if not os.path.isdir(MODEL_PATH):
    print(f"Error: Vosk model not found at {MODEL_PATH}")
    sys.exit(1)

print("=" * 60)
print("🎙️  VOICE RECOGNITION TEST — SPEAK INTO YOUR MICROPHONE")
print("=" * 60)
print("Try saying:")
print("  • 'take off'")
print("  • 'go to point a'")
print("  • 'start mission'")
print("  • 'turn left'")
print("  • 'return home'")
print("  • 'land'")
print("\n[INFO] Loading Vosk AI model...")

vosk.SetLogLevel(-1)
model = vosk.Model(MODEL_PATH)
rec = vosk.KaldiRecognizer(model, 16000)

print("[READY] Listening on default microphone! (Press Ctrl+C to stop)\n")

try:
    with sd.RawInputStream(samplerate=16000, channels=1, dtype="int16", blocksize=4000) as stream:
        while True:
            data, _ = stream.read(2000)
            audio = np.frombuffer(data, dtype=np.int16)
            if rec.AcceptWaveform(audio.tobytes()):
                result = rec.Result()
                if '"text" : "' in result:
                    text = result.split('"text" : "')[-1].split('"')[0]
                elif '"text": "' in result:
                    text = result.split('"text": "')[-1].split('"')[0]
                else:
                    text = ""
                text = text.strip()
                if text:
                    print(f"  👉 RECOGNIZED: \"{text}\"")
except KeyboardInterrupt:
    print("\n\nVoice test stopped.")
