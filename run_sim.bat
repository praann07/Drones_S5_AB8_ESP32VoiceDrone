@echo off
cd /d "%~dp0"
title LiteWing Drone GCS Cockpit Simulator
color 0A

echo ============================================================
echo   ESP32-S3 LiteWing Quadrotor -- Python Voice GCS Simulator
echo ============================================================
echo.
echo   [Controls Summary]
echo   ----------------------------------------------------------
echo   Voice Commands : "Takeoff", "Start Mission", "Run Analysis", "Start Logging"
echo   Spacebar       : Takeoff / Land Toggle
echo   M Key          : Start Waypoint Mission (Points A, B, C, D, HOME)
echo   U Key          : Toggle High-Rate 250Hz Telemetry Logging
echo   D Key          : Trigger SINDy ^& DMDc Data-Driven System Identification
echo   R / H Key      : Return to Home (RTL)
echo   K Key          : Switch Map Layer (Grid / Dark / Satellite / OSM)
echo   W Key          : Cycle Wind Condition (Calm / Light / Strong / Storm)
echo   G Key          : Inject Atmospheric Wind Gust Burst
echo   ----------------------------------------------------------
echo.
echo Launching GCS Cockpit...
python drone_sim\main.py

echo.
pause
