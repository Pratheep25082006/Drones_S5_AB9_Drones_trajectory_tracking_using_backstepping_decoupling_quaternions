"""
Launches VcXsrv (Windows X server) and Gazebo 11 inside WSL with the project world.
    python launch_gazebo_wsl.py              # Gazebo + run_gazebo_wsl_native.py (flight computed in WSL)
    python launch_gazebo_wsl.py --web        # Gazebo + web dashboard backend in WSL, browser on Windows
    python launch_gazebo_wsl.py --receiver   # Gazebo only; poses come from run_gazebo_simulation.py
                                             # (pose plugin, or wsl_sync_gazebo.py when not built)
Settings: config.yaml -> gazebo, paths.
"""

import argparse
import os
import shlex
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller.config import banner, add_config_arguments, apply_config_arguments, cfg  # noqa: E402
from quadrotor_controller.gazebo_tools import pixi_bin, wsl_project_dir  # noqa: E402


def wsl(*cmd, user=None, **kw):
    """wsl.exe command line; --exec runs ``cmd`` directly (no login shell that would expand $VARS early)."""
    base = ["wsl", "-d", cfg("gazebo.wsl_distro")]
    if user:
        base += ["-u", user]
    return base + ["--exec"] + list(cmd)


def ensure_vcxsrv_running():
    exe = cfg("gazebo.vcxsrv_exe")
    if not os.path.exists(exe):
        print(f"[WARNING] VcXsrv not found at {exe} (set gazebo.vcxsrv_exe in config.yaml).")
        return
    subprocess.run(["taskkill", "/f", "/im", os.path.basename(exe)], capture_output=True)
    print("[INFO] Starting VcXsrv ...")
    subprocess.Popen([exe] + list(cfg("gazebo.vcxsrv_args")))
    time.sleep(cfg("gazebo.vcxsrv_start_wait"))


def windows_host_display() -> str:
    """X display for Gazebo inside WSL (config gazebo.display).
    auto: WSLg (Windows 11 built-in, ':0') when present, else VcXsrv on the Windows host."""
    d = cfg("gazebo.display")
    if d != "auto":
        return d
    try:
        if subprocess.run(wsl("test", "-e", "/mnt/wslg/.X11-unix/X0"), capture_output=True).returncode == 0:
            return ":0"
    except OSError:
        pass
    # NAT networking: the Windows host is the default gateway of the WSL VM
    # (mirrored networking: no gateway entry -> localhost works)
    try:
        ip = subprocess.check_output(wsl("bash", "-c", "ip route show default | awk '{print $3}' | head -n1",
                                         user=cfg("gazebo.wsl_user")), text=True).strip()
        if ip:
            return f"{ip}:0"
    except (OSError, subprocess.CalledProcessError):
        pass
    return "127.0.0.1:0"


def launch_native_gazebo(receiver: bool = False, no_companion: bool = False, web: bool = False):
    """receiver=False: run_gazebo_wsl_native.py computes the flight inside WSL (recommended).
    receiver=True : only Gazebo; poses come over UDP from run_gazebo_simulation.py."""
    print(banner("[INFO] Launching Gazebo 11 in WSL ..."))
    user = cfg("gazebo.wsl_user")
    subprocess.run(wsl("bash", "-c", "killall -9 gzserver gzclient gazebo 2>/dev/null || true", user=user),
                   capture_output=True)
    time.sleep(cfg("gazebo.cleanup_wait"))
    display = windows_host_display()
    if not display.startswith(":"):                 # X server on the Windows host -> VcXsrv (":0" = WSLg)
        ensure_vcxsrv_running()

    proj = wsl_project_dir()
    pixi = cfg("gazebo.pixi_env")
    plugin_dir = proj + "/" + cfg("paths.gazebo_plugin_dir") + "/build"
    q = shlex.quote
    lines = [
        "chmod -R 777 /tmp/.X11-unix 2>/dev/null",
        f"export CONDA_PREFIX={q(pixi)}",
        f"export PATH={q(pixi + '/bin')}:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        f"export LD_LIBRARY_PATH={q(pixi + '/lib')}:{q(cfg('gazebo.extra_ld_library_path'))}",
        f"source {q(pixi + '/share/gazebo/setup.sh')} 2>/dev/null",
        f"export GAZEBO_MASTER_URI={q(cfg('gazebo.master_uri'))}",
        f"export GAZEBO_IP={q(cfg('gazebo.gazebo_ip'))}",
        f"export GAZEBO_PLUGIN_PATH={q(plugin_dir)}:$GAZEBO_PLUGIN_PATH",
        "export GAZEBO_MODEL_DATABASE_URI=",                    # no online model download at start-up
        f"export DISPLAY={q(display)}",
        "mkdir -p /run/user/0 && chmod 700 /run/user/0 && export XDG_RUNTIME_DIR=/run/user/0",
    ]
    if cfg("gazebo.software_rendering"):
        # Mesa llvmpipe software renderer (avoids the WSLg D3D12 window freeze), same as run_gazebo.sh
        lines += ["export LIBGL_ALWAYS_SOFTWARE=1", "export MESA_GL_VERSION_OVERRIDE=3.3",
                  "export MESA_GLSL_VERSION_OVERRIDE=330", "unset MESA_LOADER_DRIVER_OVERRIDE"]
    if cfg("gazebo.pose_transport") == "plugin":
        lib = plugin_dir + "/" + cfg("gazebo.plugin_library")
        lines.append(f"[ -f {q(lib)} ] || (echo '[INFO] building the Gazebo pose plugin ...'; "
                     f"bash {q(proj + '/build_gazebo_plugin.sh')}) || echo '[WARNING] plugin build failed - using gz model'")
    lines.append(f"echo \"[INFO] DISPLAY=$DISPLAY  world={cfg('paths.gazebo_world')}\"")
    companion = None if no_companion else ("wsl_sync_gazebo.py" if receiver else "run_gazebo_wsl_native.py")
    if web:
        companion = "run_web_dashboard.py --in-gazebo"          # flight core + web backend next to Gazebo
    if companion:
        script_name, *extra = companion.split()
        lines.append(f"{q(pixi_bin('python3'))} -u {q(proj + '/' + script_name)} {' '.join(extra)} &")
        lines.append("COMPANION=$!")
    lines.append(f"{q(pixi_bin('gazebo'))} --unpause --verbose {q(proj + '/' + cfg('paths.gazebo_world'))}")
    lines.append("echo \"[INFO] Gazebo exited (code $?)\"")
    if companion:
        lines.append("kill $COMPANION 2>/dev/null")
    # written to a file: no Windows / wsl.exe quoting of $VARS, '&', newlines
    script_file = os.path.join(HERE, cfg("gazebo.launch_script"))
    with open(script_file, "w", encoding="utf-8", newline="\n") as f:
        f.write("#!/bin/bash\n" + "\n".join(lines) + "\n")
    try:
        print(f"[INFO] X display for Gazebo: {display}" + ("  (WSLg)" if display.startswith(":") else "  (VcXsrv)"))
        process = subprocess.Popen(wsl("bash", proj + "/" + cfg("gazebo.launch_script"), user=user))
        print("[SUCCESS] Gazebo 11 starting (the window can take 10-30 s with software rendering), PID", process.pid)
        return process
    except OSError as e:
        print("[ERROR] Failed to launch Gazebo in WSL:", e)
        return None


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--receiver", action="store_true")
    ap.add_argument("--no-companion", action="store_true", help="start Gazebo only")
    ap.add_argument("--web", action="store_true", help="run the web dashboard backend next to Gazebo and open the browser")
    add_config_arguments(ap)
    a = ap.parse_args()
    apply_config_arguments(a)
    proc = launch_native_gazebo(receiver=a.receiver, no_companion=a.no_companion, web=a.web)
    if proc is not None and a.web:
        import webbrowser
        url = f"http://localhost:{cfg('web.port')}/"
        print(f"[INFO] Web dashboard: {url}  (opens in {cfg('web.open_browser_wait') + cfg('web.wsl_start_wait'):g} s)")
        threading.Timer(cfg("web.open_browser_wait") + cfg("web.wsl_start_wait"), webbrowser.open, args=(url,)).start()
    if proc is not None:
        try:
            proc.wait()                     # keep this console attached to Gazebo; Ctrl+C stops it
        except KeyboardInterrupt:
            subprocess.run(wsl("bash", "-c", "killall -9 gzserver gzclient gazebo 2>/dev/null || true",
                               user=cfg("gazebo.wsl_user")), capture_output=True)
