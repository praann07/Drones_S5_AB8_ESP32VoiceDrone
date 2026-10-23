"""Run the real simulation with typed commands on a timer (graphs served, browser not opened) and capture the
dashboard with headless Edge right after the drone has landed.

Change EDGE below if Microsoft Edge is installed somewhere else. Usage: python capture_dashboard.py [start-up seconds]
Writes dash_dark.png, dash_light.png, dash_flight.csv and dash_flight.txt next to this script."""
import os, subprocess, sys, time
CLONE = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))     # the project folder (three levels up from this script)
SCRATCH = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable                    # the Python that runs this script
EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
steps = [("take off", 7), ("forward", 7), ("banana", 2), ("back 3 meters", 6), ("rotate left 90 degrees", 5),
         ("left 2 meters", 5), ("up 1 meter", 3), ("go home", 16), ("exit", 1)]


def capture(name, theme):
    args = [EDGE, "--headless=new", "--disable-gpu", "--hide-scrollbars", "--window-size=1280,1425",
            "--virtual-time-budget=6000", "--user-data-dir=" + os.path.join(SCRATCH, "edge_" + name),
            "--screenshot=" + os.path.join(SCRATCH, name + ".png"), "http://127.0.0.1:8765/?theme=" + theme]
    r = subprocess.run(args, capture_output=True, text=True, timeout=90)
    print("captured", name, r.returncode, flush=True)


proc = subprocess.Popen([PY, "obstacles.py", "--console", "--no-voice", "--no-browser",
                        "--telemetry-file", os.path.join(SCRATCH, "dash_flight.csv")],
                       cwd=CLONE, stdin=subprocess.PIPE, stdout=open(os.path.join(SCRATCH, "dash_flight.txt"), "w"),
                       stderr=subprocess.STDOUT, text=True)
time.sleep(float(sys.argv[1]) if len(sys.argv) > 1 else 12)       # window + city start-up
for text, wait in steps:
    proc.stdin.write(text + "\n")
    proc.stdin.flush()
    print("sent", text, flush=True)
    time.sleep(wait)
    if text == "go home":
        capture("dash_dark", "dark")
        capture("dash_light", "light")
proc.stdin.close()
print("exit code", proc.wait(timeout=60))
