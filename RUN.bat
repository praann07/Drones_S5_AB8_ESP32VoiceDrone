@echo off
REM The one launcher for the whole project. Double-click, pick a number.
REM The demo opens in its own console window so the dashboard, the map and
REM the microphone all work (a window started from a script can hide them).
title Voice-controlled quadrotor - launcher
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] Virtual environment not found at .venv\Scripts\python.exe
    echo Run: python -m venv .venv   then   pip install -r requirements.txt
    pause
    exit /b 1
)
set PY=.venv\Scripts\python.exe
set SRC=--source text
if exist "vosk-model-small-en-us-0.15" set SRC=--source vosk --model-dir vosk-model-small-en-us-0.15

:MENU
cls
echo ======================================================================
echo   VOICE-CONTROLLED QUADROTOR  -  MuJoCo + SINDy inner loop
echo ======================================================================
echo.
echo     [1] Fly the drone     live demo on the Amrita campus, speak or type
echo     [2] Show results      2D and 3D graphs of the flight you just did
echo     [3] Drone parameters  mass, inertia, gains, identified model
echo     [4] Run the tests
echo     [5] Browser simulator on localhost only
echo     [6] Rebuild every result and plot  slow, several minutes
echo.
echo     [0] Exit
echo.
set /p choice="Enter choice: "
if "%choice%"=="1" goto FLY
if "%choice%"=="2" goto RESULTS
if "%choice%"=="3" goto PARAMS
if "%choice%"=="4" goto TESTS
if "%choice%"=="5" goto WEB
if "%choice%"=="6" goto REBUILD
if "%choice%"=="0" exit /b 0
echo Not a valid choice.
pause
goto MENU

:FLY
if not exist "01_simulation\models\scene_campus.xml" %PY% 01_simulation\models\make_campus.py
echo The drone starts next to the Central Library.
echo Try: left, up, left, land   or   go to the kitchen
start "Commands Guide" cmd /k "%PY% 05_voice_interface\print_commands_guide.py"
timeout /t 2 >nul
start "Drone - campus (speak, type or click HERE)" cmd /k "%PY% 05_voice_interface\demo.py --world campus %SRC%"
goto MENU

:RESULTS
start "2D Telemetry Plot" cmd /k "%PY% 05_voice_interface\plot_2d_telemetry.py"
timeout /t 2 >nul
start "3D Trajectory Plot" cmd /k "%PY% 05_voice_interface\plot_3d_trajectory.py"
goto MENU

:PARAMS
start "Drone parameters" cmd /k "%PY% 05_voice_interface\show_drone_params.py"
goto MENU

:TESTS
start "Test suite" cmd /k "%PY% -m pytest tests"
goto MENU

:WEB
start "Browser simulator - close this window to stop" cmd /k "%PY% web\export_web_sim.py && %PY% -m http.server 8765 --bind 127.0.0.1 --directory web"
timeout /t 3 >nul
start "" "http://127.0.0.1:8765"
goto MENU

:REBUILD
call REBUILD_RESULTS.bat
goto MENU
