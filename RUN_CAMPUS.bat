@echo off
REM Fly the drone around a 1:10 model of the Amrita Vishwa Vidyapeetham
REM Coimbatore campus (real building footprints and roads from OpenStreetMap,
REM heights estimated). Same voice commands, same controller as the city demo.
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] Virtual environment not found at .venv\Scripts\python.exe
    pause
    exit /b 1
)
if not exist "01_simulation\models\scene_campus.xml" .venv\Scripts\python.exe 01_simulation\models\make_campus.py
set DEMO_CMD=.venv\Scripts\python.exe 05_voice_interface\demo.py --world campus --source text
if exist "vosk-model-small-en-us-0.15" set DEMO_CMD=.venv\Scripts\python.exe 05_voice_interface\demo.py --world campus --source vosk --model-dir vosk-model-small-en-us-0.15
echo Campus demo: the drone starts next to the Central Library (to its LEFT).
echo Try: left, up, left, land  - lands on the library roof.
start "Commands Guide" cmd /k ".venv\Scripts\python.exe 05_voice_interface\print_commands_guide.py"
timeout /t 2 >nul
start "Drone - Amrita campus (speak/type/click HERE)" cmd /k "%DEMO_CMD%"
