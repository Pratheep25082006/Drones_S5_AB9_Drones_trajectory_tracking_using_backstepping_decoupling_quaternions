@echo off
title 3D Quadrotor Drone Visualizer & HUD
echo ========================================================
echo   Launching 3D Quadrotor Drone Visualizer Server
echo ========================================================
echo.
echo [1/2] Starting Telemetry Bridge & Server on Port 8080...
start /b python d:\Drones\run_gazebo_bridge.py
timeout /t 2 >nul

echo [2/2] Opening 3D Visualizer HUD in Web Browser...
start "" "http://127.0.0.1:8080/gazebo_visualizer.html"

echo.
echo [SUCCESS] 3D Visualizer is live in your browser!
echo Press any key to stop the server...
pause >nul
