@echo off
title Launch Native Gazebo 11 - 3D Quadrotor Simulation
color 0A
echo =================================================================
echo        LAUNCHING NATIVE GAZEBO 11 - 3D QUADROTOR SIMULATION
echo =================================================================
echo.
echo [1/2] Ensuring WSL Ubuntu environment is ready...
wsl -d Ubuntu -u root bash -c "mkdir -p /run/user/0 && chmod 700 /run/user/0 && chmod 777 /tmp/.X11-unix 2>/dev/null"

echo [2/2] Launching Native Gazebo 11 GUI + Flight Controller...
echo.
wsl -d Ubuntu -u root bash /mnt/d/Drones/run_gazebo.sh

pause
