@echo off
title Quadrotor Web Telemetry Dashboard
REM Host / ports / page: config.yaml -> bridge.  Set DRONES_PYTHON to use a specific Python.
set "PY=python"
if defined DRONES_PYTHON set "PY=%DRONES_PYTHON%"
echo ========================================================
echo   Quadrotor web telemetry dashboard (paper controller)
echo ========================================================
pushd "%~dp0"
"%PY%" run_gazebo_bridge.py --open-browser %*
popd
pause
