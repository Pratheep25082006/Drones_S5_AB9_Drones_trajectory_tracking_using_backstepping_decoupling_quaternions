@echo off
title gym-pybullet-drones 3D Quadrotor Simulation
REM All settings come from config.yaml.  Set DRONES_PYTHON to use a specific Python.
set "PY=python"
if defined DRONES_PYTHON set "PY=%DRONES_PYTHON%"
echo =======================================================================
echo   GYM-PYBULLET-DRONES - DE MONTE ^& LOHMANN (2013) FULL BACKSTEPPING
echo =======================================================================
echo [INFO] Python: %PY%   Config: %~dp0config.yaml
echo.
echo [COMMANDS while flying]  python set_target.py auto ^| spiral ^| figure8 ^| hover ^| land ^| X Y Z
echo [OPTIONS] --no-noise  --no-filter  --no-avoid  --set key=value
echo.
pushd "%~dp0"
"%PY%" run_pybullet_simulation.py %*
popd
pause
