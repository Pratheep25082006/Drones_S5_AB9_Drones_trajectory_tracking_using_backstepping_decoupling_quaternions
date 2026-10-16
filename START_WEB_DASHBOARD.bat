@echo off
title Quadrotor 3D web dashboard (Gazebo backend)
REM Gazebo 11 + flight core (paper controller, noise, Kalman filter, LiDAR, path planner) run in WSL;
REM the 3D web dashboard opens in the Windows browser.  Settings: config.yaml -> web, gazebo.
REM Without Gazebo:  python run_web_dashboard.py --open-browser
set "PY=python"
if defined DRONES_PYTHON set "PY=%DRONES_PYTHON%"
echo =================================================================
echo   QUADROTOR 3D WEB DASHBOARD  -  backend: Gazebo 11 (WSL)
echo =================================================================
pushd "%~dp0"
"%PY%" launch_gazebo_wsl.py --web %*
popd
pause
