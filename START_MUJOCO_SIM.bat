@echo off
title MuJoCo 3D Quadrotor Simulation
REM All settings (including the Windows OpenGL fix) come from config.yaml.
REM Set DRONES_PYTHON to use a specific Python, e.g. set DRONES_PYTHON=C:\path\to\python.exe
set "PY=python"
if defined DRONES_PYTHON set "PY=%DRONES_PYTHON%"
echo =================================================================
echo   MUJOCO 3D QUADROTOR - DE MONTE ^& LOHMANN (2013) BACKSTEPPING
echo =================================================================
echo [INFO] Python: %PY%   Config: %~dp0config.yaml
echo.
echo [COMMANDS while flying]  python set_target.py auto ^| spiral ^| figure8 ^| hover ^| land ^| X Y Z
echo [MOUSE] left drag = orbit, right drag = pan, wheel = zoom, space = pause
echo.
pushd "%~dp0"
"%PY%" run_mujoco_simulation.py %*
popd
pause
