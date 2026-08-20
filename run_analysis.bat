@echo off
cd /d "%~dp0"
title Data-Driven Modeling ^& Simulation (DDMS) Analysis
color 0B

echo ============================================================
echo   6-DOF Drone Flight Data-Driven System Identification
echo   Engines: SINDy (Sparse Nonlinear ODE) + DMDc (Dynamic Modes)
echo ============================================================
echo.
echo Running high-rate telemetry analysis...
echo.

python drone_sim\data_driven_analysis.py

echo.
echo ============================================================
echo   Analysis Complete! Opening generated charts...
echo ============================================================

if exist data_driven_results\trajectory_3d_comparison.png (
    start data_driven_results\trajectory_3d_comparison.png
)
if exist data_driven_results\state_time_series_comparison.png (
    start data_driven_results\state_time_series_comparison.png
)
if exist data_driven_results\dmdc_eigenvalue_spectrum.png (
    start data_driven_results\dmdc_eigenvalue_spectrum.png
)
if exist data_driven_results\ddms_final_summary.md (
    type data_driven_results\sindy_discovered_equations.txt
)

echo.
pause
