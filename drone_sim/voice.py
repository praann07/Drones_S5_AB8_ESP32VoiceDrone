import os
import queue
import subprocess
import threading
import urllib.request
import zipfile

MODEL_NAME = "vosk-model-small-en-us-0.15"
MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
MODEL_PATH = os.path.join(MODEL_DIR, MODEL_NAME)
MODEL_URL = "https://alphacephei.com/vosk/models/{}.zip".format(MODEL_NAME)

try:
    import vosk  # noqa: F401
    import sounddevice as sd
    import numpy as np

    HAVE_VOSK = True
except Exception:
    HAVE_VOSK = False


def model_available():
    return os.path.isdir(MODEL_PATH)


def download_model(progress_cb=None):
    if model_available():
        return True
    os.makedirs(MODEL_DIR, exist_ok=True)
    zip_path = os.path.join(MODEL_DIR, MODEL_NAME + ".zip")
    try:
        req = urllib.request.Request(MODEL_URL, headers={"User-Agent": "Mozilla/5.0"})
        with urllib.request.urlopen(req, timeout=60) as resp:
            total = int(resp.headers.get("Content-Length", 0))
            got = 0
            with open(zip_path, "wb") as f:
                while True:
                    chunk = resp.read(65536)
                    if not chunk:
                        break
                    f.write(chunk)
                    got += len(chunk)
                    if progress_cb and total:
                        progress_cb(got / total)
        with zipfile.ZipFile(zip_path, "r") as z:
            z.extractall(MODEL_DIR)
        os.remove(zip_path)
        return True
    except Exception:
        return False


class TTSAnnouncer:
    """Zero-overhead Persistent Text-To-Speech announcer using background PowerShell pipe."""
    def __init__(self, enabled=True):
        self.enabled = enabled
        self._proc = None
        self._thread = None
        self._queue = queue.Queue(maxsize=10)
        self._stop_event = threading.Event()
        if self.enabled:
            self._start_worker()

    def _start_worker(self):
        def _runner():
            try:
                ps_script = (
                    "Add-Type -AssemblyName System.Speech; "
                    "$synth = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
                    "while ($true) { "
                    "  $line = [Console]::In.ReadLine(); "
                    "  if (!$line -or $line -eq 'QUIT') { break }; "
                    "  try { $synth.Speak($line) } catch {} "
                    "}"
                )
                self._proc = subprocess.Popen(
                    ["powershell", "-NoProfile", "-NonInteractive", "-Command", ps_script],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    bufsize=1
                )
                while not self._stop_event.is_set() and self._proc.poll() is None:
                    try:
                        msg = self._queue.get(timeout=0.2)
                        if msg == "QUIT":
                            break
                        if self._proc.stdin:
                            self._proc.stdin.write(msg + "\n")
                            self._proc.stdin.flush()
                    except queue.Empty:
                        continue
                    except Exception:
                        break
            except Exception:
                pass
            finally:
                if self._proc:
                    try:
                        self._proc.kill()
                    except Exception:
                        pass

        self._thread = threading.Thread(target=_runner, daemon=True)
        self._thread.start()

    def speak(self, text):
        if not self.enabled or not text:
            return
        clean_text = text.replace('\n', ' ').replace('"', '').replace("'", "").strip()
        if not clean_text:
            return
        try:
            # Non-blocking queue put, drop if queue full to avoid any lag
            if not self._queue.full():
                self._queue.put_nowait(clean_text)
        except Exception:
            pass

    def stop(self):
        self._stop_event.set()
        try:
            self._queue.put_nowait("QUIT")
        except Exception:
            pass



class VoiceListener:
    def __init__(self, out_queue, ready_cb=None):
        self.out_queue = out_queue
        self.ready_cb = ready_cb
        self.available = HAVE_VOSK
        self.ready = False
        self.thread = None
        self.stop_event = threading.Event()

    def start(self):
        if not self.available:
            return False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        self.stop_event.set()

    def _run(self):
        if not model_available():
            self.out_queue.put(("_voice_status", "Downloading speech model (~40 MB)..."))
            if not download_model():
                self.out_queue.put(("_voice_status", "Voice unavailable - using typed commands"))
                return
        try:
            vosk.SetLogLevel(-1)
            model = vosk.Model(MODEL_PATH)
            rec = vosk.KaldiRecognizer(model, 16000)
            stream = sd.RawInputStream(samplerate=16000, channels=1, dtype="int16", blocksize=4000)
            stream.start()
        except Exception as e:
            self.out_queue.put(("_voice_status", "Voice mic unavailable - using typed commands"))
            return
        self.ready = True
        if self.ready_cb:
            self.ready_cb()
        self.out_queue.put(("_voice_status", "Voice active - speak command (e.g. 'take off', 'go to A')"))
        try:
            while not self.stop_event.is_set():
                data, overflow = stream.read(2000)
                audio = np.frombuffer(data, dtype=np.int16)
                if rec.AcceptWaveform(audio.tobytes()):
                    result = rec.Result()
                    text = result.split('"text" : "')[-1].split('"')[0] if '"text" : "' in result else ""
                    if not text and '"text": "' in result:
                        text = result.split('"text": "')[-1].split('"')[0]
                    text = text.strip()
                    if text:
                        self.out_queue.put(("_voice", text))
        except Exception:
            pass
        finally:
            try:
                stream.stop()
                stream.close()
            except Exception:
                pass

