"""
Native Gazebo 11 Launcher for WSL Ubuntu + Windows X-Server (VcXsrv)
---------------------------------------------------------------------
Automates launching VcXsrv X11 server on Windows and Native Gazebo 11 GUI in WSL Ubuntu.
"""

import os
import sys
import subprocess
import time


def ensure_vcxsrv_running():
    vcxsrv_path = r"C:\Program Files\VcXsrv\vcxsrv.exe"
    if not os.path.exists(vcxsrv_path):
        print("[WARNING] VcXsrv path not found at default location.")
        return

    try:
        subprocess.run("taskkill /f /im vcxsrv.exe 2>nul", shell=True)
        time.sleep(0.5)
        print("[INFO] Starting VcXsrv X-Server background process with -ac...")
        subprocess.Popen([vcxsrv_path, ":0", "-multiwindow", "-clipboard", "-wgl", "-ac"])
        time.sleep(2)
    except Exception as e:
        print("[NOTE] VcXsrv startup:", e)


def kill_stale_gazebo_processes():
    print("[INFO] Cleaning up any previous Gazebo processes in WSL...")
    try:
        subprocess.run(
            ["wsl", "-d", "Ubuntu", "-u", "root", "bash", "-c", "killall -9 gzserver gzclient gazebo 2>/dev/null || true"],
            capture_output=True,
            text=True
        )
        time.sleep(1)
    except Exception:
        pass


def get_wsl_display():
    try:
        res = subprocess.check_output(["wsl", "-d", "Ubuntu", "-u", "root", "bash", "-c", "cat /etc/resolv.conf | grep nameserver | awk '{print $2}'"], text=True).strip()
        if res:
            return f"{res}:0"
    except Exception:
        pass
    return ":0"


def launch_native_gazebo():
    print("=" * 60)
    print("[INFO] Launching Native Gazebo 11 3D Desktop Window...")
    print("=" * 60)

    kill_stale_gazebo_processes()
    ensure_vcxsrv_running()

    display_env = get_wsl_display()

    # Launch Gazebo inside WSL with full environment & master URI
    wsl_cmd = (
        'chmod -R 777 /tmp/.X11-unix 2>/dev/null || true && '
        'export CONDA_PREFIX="/root/.pixi/envs/gazebo" && '
        'export PATH="/root/.pixi/envs/gazebo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" && '
        'export LD_LIBRARY_PATH="/root/.pixi/envs/gazebo/lib" && '
        'export GAZEBO_MASTER_URI="http://127.0.0.1:11345" && '
        'export GAZEBO_RESOURCE_PATH="/root/.pixi/envs/gazebo/share/gazebo-11" && '
        'export GAZEBO_PLUGIN_PATH="/root/.pixi/envs/gazebo/lib" && '
        'export GAZEBO_MODEL_PATH="/root/.pixi/envs/gazebo/share/gazebo-11/models" && '
        'chmod 666 /root/.pixi/envs/gazebo/share/OGRE/plugins.cfg 2>/dev/null || true && '
        'sed -i "s|PluginFolder=/root/.pixi/envs/gazebo/lib/OGRE$|PluginFolder=/root/.pixi/envs/gazebo/lib/OGRE/|" /root/.pixi/envs/gazebo/share/OGRE/plugins.cfg 2>/dev/null || true && '
        'source /root/.pixi/envs/gazebo/share/gazebo/setup.sh 2>/dev/null || true && '
        f'export DISPLAY="{display_env}" && '
        'export LIBGL_ALWAYS_SOFTWARE=1 && '
        '/root/.pixi/envs/gazebo/bin/python3 /mnt/d/Drones/run_gazebo_wsl_native.py & '
        '/root/.pixi/envs/gazebo/bin/gazebo --unpause --verbose /mnt/d/Drones/models/quadrotor.world'
    )





















    cmd = ["wsl", "-d", "Ubuntu", "bash", "-c", wsl_cmd]

    try:
        process = subprocess.Popen(cmd)
        print("[SUCCESS] Native Gazebo 11 process launched! PID:", process.pid)
        print("[NOTE] Opening Gazebo 3D Desktop GUI window on your screen...")
        return process
    except Exception as e:
        print("[ERROR] Failed to launch Gazebo in WSL:", e)
        return None


if __name__ == "__main__":
    launch_native_gazebo()
