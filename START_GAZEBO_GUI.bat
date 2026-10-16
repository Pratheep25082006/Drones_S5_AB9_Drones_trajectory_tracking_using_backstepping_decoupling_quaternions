@echo off
title Gazebo 11 (WSL) - 3D Quadrotor Simulation
REM Starts VcXsrv and Gazebo 11 inside WSL; WSL distro, Pixi path, VcXsrv path etc.: config.yaml -> gazebo
set "PY=python"
if defined DRONES_PYTHON set "PY=%DRONES_PYTHON%"
echo =================================================================
echo   GAZEBO 11 (WSL) - DE MONTE ^& LOHMANN (2013) BACKSTEPPING
echo =================================================================
pushd "%~dp0"
"%PY%" launch_gazebo_wsl.py %*
popd
pause
