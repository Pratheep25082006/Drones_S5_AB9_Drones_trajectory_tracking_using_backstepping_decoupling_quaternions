#!/bin/bash
# Builds the Gazebo UDP pose plugin once (run inside WSL):  bash build_gazebo_plugin.sh
# Uses the Gazebo installation of config.yaml -> gazebo.pixi_env.
# Installs cmake / g++ with apt-get when they are missing (needs root, e.g. wsl -u root).
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
eval "$(python3 "$HERE/gazebo_env.py")"
if ! command -v cmake >/dev/null || ! command -v g++ >/dev/null || ! command -v make >/dev/null; then
    echo "[INFO] installing build tools (cmake, g++, make) ..."
    SUDO=""; [ "$EUID" -ne 0 ] && SUDO="sudo"
    $SUDO apt-get update -qq && $SUDO apt-get install -y -qq cmake g++ make
fi
mkdir -p "$PLUGIN_DIR/build"
cd "$PLUGIN_DIR/build"
# system cmake / g++ first, Gazebo headers and libraries from the Pixi environment
cmake -DCMAKE_PREFIX_PATH="$PIXI_ENV" -DCMAKE_BUILD_TYPE=Release ..
make -j"$(nproc)"
echo "[OK] built $PLUGIN_DIR/build/$PLUGIN_LIBRARY"
