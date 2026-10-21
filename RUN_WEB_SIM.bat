@echo off
REM Browser flight simulator on localhost only (nothing is uploaded anywhere).
REM Serves web\ at http://127.0.0.1:8765 and opens it. Close this window to stop.
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] Virtual environment not found at .venv\Scripts\python.exe
    echo Run python -m venv .venv and pip install -r requirements.txt first.
    pause
    exit /b 1
)
echo Exporting the current gains and model into web\params.js ...
.venv\Scripts\python.exe web\export_web_sim.py
if errorlevel 1 (
    echo [ERROR] Export failed - run 04_control\run_stability.py once, then try again.
    pause
    exit /b 1
)
echo.
echo Flight simulator: http://127.0.0.1:8765/index.html
echo The microphone button works in Chrome or Edge on this localhost page.
echo Close this window to stop the server.
start "" "http://127.0.0.1:8765/index.html"
.venv\Scripts\python.exe -m http.server 8765 --bind 127.0.0.1 --directory web
