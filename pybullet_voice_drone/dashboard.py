"""Live graphs for the simulation, served on this computer only.

While the simulation runs it records a sample about ten times a second and every command it receives. A
tiny web server (Python standard library, no extra packages) on 127.0.0.1 hands those to dashboard.html,
which draws the graphs in your browser and asks for new data a few times a second. Nothing leaves the
machine: the server only accepts connections from this computer.

    simulation loop --record()/event()--> Dashboard (in memory) <--/api/data-- browser page

Endpoints (read-only):
    /              the page (dashboard.html, re-read on every request so it can be edited live)
    /api/config    fixed facts about the scene: obstacles, limits, home position
    /api/data      samples and events recorded since the indexes the page last saw
"""

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard.html")


class Dashboard:
    SAMPLE_HZ = 10.0                 # how often the loop's state is recorded for the graphs
    MAX_PORT_TRIES = 10              # if the port is taken, try the next ones

    def __init__(self, config, gap_fn=None, port=8765, history=6000):
        self.config = dict(config, sample_hz=self.SAMPLE_HZ)
        self.gap_fn = gap_fn         # controller -> distance to the nearest obstacle's safety zone (or None)
        self.port = port             # 0 = let the system choose (used by the tests)
        self.history = history       # samples kept (6000 = ten minutes at 10 Hz); older ones are dropped
        self.url = None
        self.start_time = time.monotonic()
        self._lock = threading.Lock()
        self._samples, self._events = [], []
        self._dropped_samples = self._dropped_events = 0     # how many were trimmed off the front
        self._last_sample = -1e9
        self._server = self._thread = None

    # ---------------------------------------------------------------- recording (main loop)

    def record(self, controller, now=None):
        """Store a sample of the drone's state (at most SAMPLE_HZ times a second). Returns True if stored."""
        t = (time.monotonic() if now is None else now) - self.start_time
        if t - self._last_sample < 1.0 / self.SAMPLE_HZ:
            return False
        self._last_sample = t
        x, y, z = controller.pos
        vx, vy, vz = controller.vel
        gap = self.gap_fn(controller) if self.gap_fn else None
        sample = {"t": round(t, 2), "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
                  "hs": round(math.hypot(vx, vy), 3), "vz": round(vz, 3),
                  "yaw": round(math.degrees(controller.yaw) % 360.0, 1),
                  "gap": None if gap is None else round(gap, 2),
                  "label": controller.label, "mode": controller.mode.value}
        with self._lock:
            self._samples.append(sample)
            self._trim("_samples", "_dropped_samples", self.history)
        return True

    def event(self, kind, text, label="", now=None):
        """Log something the operator did or said. kind: 'command', 'heard' or 'rejected'."""
        t = (time.monotonic() if now is None else now) - self.start_time
        with self._lock:
            self._events.append({"t": round(t, 2), "kind": kind, "text": text, "label": label})
            self._trim("_events", "_dropped_events", max(200, self.history // 10))

    def _trim(self, items, dropped, limit):
        if len(getattr(self, items)) > limit:
            excess = len(getattr(self, items)) - limit + limit // 10      # drop a tenth at once, not one by one
            del getattr(self, items)[:excess]
            setattr(self, dropped, getattr(self, dropped) + excess)

    def data(self, samples_from=0, events_from=0):
        """Everything recorded since the given indexes, plus the indexes to ask for next time."""
        with self._lock:
            s0 = max(samples_from - self._dropped_samples, 0)
            e0 = max(events_from - self._dropped_events, 0)
            return {"s": self._dropped_samples + len(self._samples), "e": self._dropped_events + len(self._events),
                    "samples": self._samples[s0:], "events": self._events[e0:], "uptime": round(
                        time.monotonic() - self.start_time, 1)}

    # ---------------------------------------------------------------- the server

    def start(self):
        """Start serving. Returns the URL. Raises OSError if no port could be opened."""
        handler = self._handler()
        last_error = None
        for offset in range(self.MAX_PORT_TRIES if self.port else 1):
            try:
                server = ThreadingHTTPServer(("127.0.0.1", self.port + offset if self.port else 0), handler)
                break
            except OSError as error:
                last_error = error
        else:
            raise OSError("no free port for the live graphs (%s)" % last_error)
        server.daemon_threads = True
        self._server = server
        self.url = "http://127.0.0.1:%d/" % server.server_address[1]
        self._thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.2}, daemon=True)
        self._thread.start()
        return self.url

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None

    def _handler(self):
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):                       # keep the console free for the drone
                pass

            def _send(self, status, body, content_type):
                payload = body if isinstance(body, bytes) else body.encode("utf-8")
                try:
                    self.send_response(status)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Content-Length", str(len(payload)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(payload)
                except ConnectionError:
                    pass                                         # the browser closed the tab mid-reply

            def do_GET(self):
                url = urlparse(self.path)
                if url.path in ("/", "/index.html"):
                    try:
                        with open(PAGE, "rb") as f:
                            self._send(200, f.read(), "text/html; charset=utf-8")
                    except OSError:
                        self._send(500, "dashboard.html is missing", "text/plain; charset=utf-8")
                elif url.path == "/api/config":
                    self._send(200, json.dumps(dashboard.config), "application/json")
                elif url.path == "/api/data":
                    query = parse_qs(url.query)
                    try:
                        s, e = int(query.get("s", ["0"])[0]), int(query.get("e", ["0"])[0])
                    except ValueError:
                        s = e = 0
                    self._send(200, json.dumps(dashboard.data(s, e)), "application/json")
                else:
                    self._send(404, "not found", "text/plain; charset=utf-8")

        return Handler
