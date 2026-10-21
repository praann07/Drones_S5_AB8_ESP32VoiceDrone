@echo off
title Drone parameters
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo The Python environment .venv was not found. Run the setup in README.md first.
    pause
    exit /b 1
)
".venv\Scripts\python.exe" 05_voice_interface\show_drone_params.py
pause
