@echo off
REM Double-click: the drone flies a patrol loop BY ITSELF around a 3D city while
REM 4 obstacles move across its route, curving around them (no operator input).
REM Opens the live 3D viewer - close the viewer window to stop.
REM For the stats + path plot + GIF instead, run: run_autonomous.py --render
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] Virtual environment not found at .venv\Scripts\python.exe
    echo Run python -m venv .venv and pip install -r requirements.txt first.
    pause
    exit /b 1
)

echo Starting autonomous patrol among moving obstacles...
start "Autonomous Patrol - live 3D" cmd /k ".venv\Scripts\python.exe 05_voice_interface\run_autonomous.py --live"
