#!/bin/bash
# =================================================================
# NATIVE GAZEBO 11 LAUNCHER (inside WSL) - settings from config.yaml
# =================================================================

if [ "$EUID" -ne 0 ]; then
    exec sudo -E bash "$0" "$@"
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
eval "$(python3 "$HERE/gazebo_env.py")" || { echo "[ERROR] could not read config.yaml"; exit 1; }

chmod 777 /tmp/.X11-unix 2>/dev/null
mkdir -p /run/user/0 && chmod 700 /run/user/0

export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR=/run/user/0
if [ "$SOFTWARE_RENDERING" = "1" ]; then
    # Mesa LLVMpipe software renderer (fixes WSLg D3D12 X11 window freeze)
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_GL_VERSION_OVERRIDE=3.3
    export MESA_GLSL_VERSION_OVERRIDE=330
    unset MESA_LOADER_DRIVER_OVERRIDE
fi

export CONDA_PREFIX="$PIXI_ENV"
export PATH="$PIXI_ENV/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
for f in "$PIXI_ENV"/etc/conda/activate.d/*.sh; do
    [ -f "$f" ] && source "$f" 2>/dev/null
done
source "$PIXI_ENV/share/gazebo/setup.sh" 2>/dev/null
export LD_LIBRARY_PATH="$EXTRA_LD_LIBRARY_PATH:$EXTRA_LD_LIBRARY_PATH/dri:$PIXI_ENV/lib:$LD_LIBRARY_PATH"
export QT_QPA_PLATFORM=xcb
export GAZEBO_PLUGIN_PATH="$PLUGIN_DIR/build:$GAZEBO_PLUGIN_PATH"
if [ "$POSE_TRANSPORT" = "plugin" ] && [ ! -f "$PLUGIN_DIR/build/$PLUGIN_LIBRARY" ]; then
    echo "[INFO] Building the Gazebo UDP pose plugin (first run) ..."
    bash "$DRONES_DIR/build_gazebo_plugin.sh" || echo "[WARN] plugin build failed - using gz model updates"
fi

echo "=================================================================="
echo "   NATIVE GAZEBO 11 - 3D QUADROTOR SIMULATION"
echo "=================================================================="
echo "[INFO] Project: $DRONES_DIR"
echo "[INFO] Display: $DISPLAY | Gazebo master: $GAZEBO_MASTER_URI"

killall -9 gzserver gzclient gazebo 2>/dev/null
sleep "$CLEANUP_WAIT"

# flight core (paper controller + rigid body) -> pushes the pose into Gazebo
"$PIXI_ENV/bin/python3" "$DRONES_DIR/run_gazebo_wsl_native.py" &
CTRL_PID=$!

"$PIXI_ENV/bin/gazebo" --unpause "$GAZEBO_WORLD"

kill $CTRL_PID 2>/dev/null
