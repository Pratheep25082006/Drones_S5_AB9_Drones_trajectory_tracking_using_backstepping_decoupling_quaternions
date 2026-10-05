@echo off
title gym-pybullet-drones 3D Quadrotor Simulation
color 0A
echo =======================================================================
echo     GYM-PYBULLET-DRONES BACKSTEPPING ^& KALMAN FILTER SIMULATION
echo =======================================================================
echo.

set PYTHON_EXE=python
if exist "D:\Anaconda\python.exe" (
    set "PYTHON_EXE=D:\Anaconda\python.exe"
)

echo [INFO] Using Python: %PYTHON_EXE%
echo [INFO] Launching gym-pybullet-drones 3D Interactive Simulation...
echo [INFO] Features: Sensor Noise + Discrete Kalman Filter + Backstepping Control
echo.
echo [FLIGHT MODES via 'python set_target.py ^<mode^>']:
echo   - python set_target.py auto        (Automated Takeoff -^> Spiral -^> Figure-8)
echo   - python set_target.py land        (Touchdown)
echo   - python set_target.py X Y Z       (Fly to any 3D coordinate)
echo.
echo [OPTIONS]:
echo   - Run with raw noise (no filter):  python run_pybullet_simulation.py --no-filter
echo   - Run with clean physics:          python run_pybullet_simulation.py --no-noise
echo.

%PYTHON_EXE% run_pybullet_simulation.py

pause
