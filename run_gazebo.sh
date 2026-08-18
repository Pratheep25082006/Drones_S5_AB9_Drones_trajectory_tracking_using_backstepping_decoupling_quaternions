#!/bin/bash
# =================================================================
# NATIVE GAZEBO 11 LAUNCHER - SMOOTH STABLE SOFTWARE RENDERING
# =================================================================

if [ "$EUID" -ne 0 ]; then
    exec sudo -E bash "$0" "$@"
fi

# Ensure runtime permissions
chmod 777 /tmp/.X11-unix 2>/dev/null
mkdir -p /run/user/0 && chmod 700 /run/user/0

# === Environment & Display Setup ===
export DISPLAY=:0
export XDG_RUNTIME_DIR=/run/user/0

# Use Mesa LLVMpipe Software Renderer (Fixes WSLg D3D12 X11 window freeze / Not Responding)
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330
unset MESA_LOADER_DRIVER_OVERRIDE

# Localhost IPC binding
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_IP=127.0.0.1

export CONDA_PREFIX=/root/.pixi/envs/gazebo
export PATH=/root/.pixi/envs/gazebo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

for f in /root/.pixi/envs/gazebo/etc/conda/activate.d/*.sh; do
    [ -f "$f" ] && source "$f" 2>/dev/null
done
source /root/.pixi/envs/gazebo/share/gazebo/setup.sh 2>/dev/null

export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu/dri:/root/.pixi/envs/gazebo/lib:$LD_LIBRARY_PATH
export QT_QPA_PLATFORM=xcb

echo "=================================================================="
echo "   NATIVE GAZEBO 11 - 3D QUADROTOR SIMULATION (STABLE)"
echo "=================================================================="
echo "[INFO] Display: $DISPLAY"
echo "[INFO] Renderer: Mesa LLVMpipe (Software - Freeze Free)"
echo "[INFO] Gazebo Master URI: $GAZEBO_MASTER_URI"
echo "[INFO] Launching Gazebo GUI + Controller..."

# Kill any stale processes
killall -9 gzserver gzclient gazebo 2>/dev/null
sleep 1

# Start Quadrotor Backstepping Controller in background
/root/.pixi/envs/gazebo/bin/python3 /mnt/d/Drones/run_gazebo_wsl_native.py &
CTRL_PID=$!

# Launch Gazebo (gzserver + gzclient)
/root/.pixi/envs/gazebo/bin/gazebo --unpause /mnt/d/Drones/models/quadrotor.world

# Clean up controller when Gazebo window closes
kill $CTRL_PID 2>/dev/null
