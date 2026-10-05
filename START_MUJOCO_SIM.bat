@echo off
title MuJoCo 3D Quadrotor Simulation
color 0B
echo =================================================================
echo        MUJOCO 3D QUADROTOR BACKSTEPPING SIMULATION
echo =================================================================
echo.

REM ---- FIX: Force hardware OpenGL (WGL) ----
REM Anaconda ships opengl32sw.dll (software renderer) in Library/bin
REM which shadows the real GPU OpenGL. We fix this by:
REM 1. Setting MUJOCO_GL=wgl  (Windows native hardware OpenGL)
REM 2. Putting System32 FIRST in PATH so opengl32.dll resolves to the GPU driver
set MUJOCO_GL=wgl
set PATH=C:\Windows\System32;%PATH%

set PYTHON_EXE=python
if exist "D:\Anaconda\python.exe" (
    set "PYTHON_EXE=D:\Anaconda\python.exe"
)

echo [INFO] MUJOCO_GL = %MUJOCO_GL%
echo [INFO] Using Python: %PYTHON_EXE%
echo [INFO] Verifying MuJoCo install...
%PYTHON_EXE% -c "import mujoco; print('[INFO] MuJoCo', mujoco.mj_version(), 'OK')" 2>nul
if %errorlevel% neq 0 (
    echo [INFO] Installing mujoco...
    pip install mujoco
)

echo.
echo [INFO] Launching MuJoCo Real-World 3D Flight Arena...
echo [INFO] Environment: Helipad + 4 Obstacle Towers + Slalom Racing Gates
echo [INFO] Features: Sensor Noise Injected + Discrete Kalman Filter Denoising
echo.
echo [FLIGHT MODES via 'python set_target.py <mode>']:
echo   - python set_target.py mission     (Full 5-phase test flight)
echo   - python set_target.py racetrack   (Slalom racing gates circuit)
echo   - python set_target.py spiral      (3D helical spiral climb)
echo   - python set_target.py figure8     (Lemniscate aerobatics)
echo   - python set_target.py land        (Precision helipad touchdown)
echo   - python set_target.py X Y Z       (Fly to any 3D coordinate)
echo.
echo [MOUSE CONTROLS]:
echo   - Left Click + Drag: Orbit Camera
echo   - Right Click + Drag: Pan Camera
echo   - Scroll Wheel: Zoom in/out
echo   - Space: Pause / Resume physics
echo.
%PYTHON_EXE% run_mujoco_simulation.py

pause

