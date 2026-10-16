"""
Helpers for the Gazebo 11 (WSL) side: paths and environment from config.yaml -> gazebo.
"""

from __future__ import annotations

import os
import subprocess
from typing import Dict, List, Sequence

from .config import PROJECT_ROOT, cfg, project_path


def wsl_path(windows_path: str) -> str:
    """Windows path -> WSL path (D:\\Drones -> /mnt/d/Drones)."""
    if os.name != "nt":
        return windows_path
    try:
        out = subprocess.check_output(["wsl", "-d", cfg("gazebo.wsl_distro"), "wslpath", "-a",
                                       windows_path.replace("\\", "/")], text=True).strip()
        if out:
            return out
    except (OSError, subprocess.CalledProcessError):
        pass
    drive, rest = os.path.splitdrive(os.path.abspath(windows_path))
    return f"/mnt/{drive.rstrip(':').lower()}{rest.replace(os.sep, '/')}"


def wsl_project_dir() -> str:
    d = cfg("gazebo.wsl_project_dir")
    return wsl_path(PROJECT_ROOT) if d == "auto" else d


def pixi_bin(name: str) -> str:
    return f"{cfg('gazebo.pixi_env')}/bin/{name}"


def gz_environment(base: Dict[str, str] | None = None) -> Dict[str, str]:
    """Environment variables for gz / gazebo processes inside WSL."""
    env = dict(os.environ if base is None else base)
    pixi = cfg("gazebo.pixi_env")
    env["CONDA_PREFIX"] = pixi
    env["PATH"] = f"{pixi}/bin:" + env.get("PATH", "")
    env["LD_LIBRARY_PATH"] = f"{cfg('gazebo.extra_ld_library_path')}:{pixi}/lib:" + env.get("LD_LIBRARY_PATH", "")
    env["GAZEBO_MASTER_URI"] = cfg("gazebo.master_uri")
    env["GAZEBO_IP"] = cfg("gazebo.gazebo_ip")
    return env


def gz_model_command(x: float, y: float, z: float, roll: float, pitch: float, yaw: float,
                     model: str = "quadrotor") -> List[str]:
    return [pixi_bin("gz"), "model", "-m", model, "-x", f"{x:.3f}", "-y", f"{y:.3f}", "-z", f"{z:.3f}",
            "-R", f"{roll:.3f}", "-P", f"{pitch:.3f}", "-Y", f"{yaw:.3f}"]


def plugin_library_path() -> str:
    return os.path.join(PROJECT_ROOT, cfg("paths.gazebo_plugin_dir"), "build", cfg("gazebo.plugin_library"))


def pose_transport() -> str:
    """'plugin' if configured and built, otherwise 'gz_model' (one `gz model` process per update)."""
    if cfg("gazebo.pose_transport") == "plugin" and os.path.exists(plugin_library_path()):
        return "plugin"
    if cfg("gazebo.pose_transport") == "plugin":
        print("[gazebo] UDP pose plugin not built yet (run: bash build_gazebo_plugin.sh in WSL) "
              "-> falling back to `gz model` updates")
    return "gz_model"


def udp_host() -> str:
    """Address of the Gazebo side for UDP pose packets."""
    h = cfg("gazebo.udp_host")
    if h != "auto":
        return h
    if os.name != "nt":
        return "127.0.0.1"
    try:   # first IPv4 address of the WSL VM
        out = subprocess.check_output(["wsl", "-d", cfg("gazebo.wsl_distro"), "hostname", "-I"], text=True).split()
        if out:
            return out[0]
    except (OSError, subprocess.CalledProcessError):
        pass
    return "127.0.0.1"


class PoseSender:
    """Sends (x, y, z, roll, pitch, yaw) float32 packets to the Gazebo pose plugin / receiver."""

    FORMAT = "ffffff"

    def __init__(self, host: str | None = None):
        import socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host = host or udp_host()

    def send(self, port: int, pose) -> None:
        import struct
        try:
            self.sock.sendto(struct.pack(self.FORMAT, *[float(v) for v in pose]), (self.host, port))
        except OSError:
            pass


class GazeboPoseOutput:
    """Drone pose + target marker -> Gazebo.

    local=True  (running inside WSL next to Gazebo): UDP pose plugin on 127.0.0.1, or `gz model`
                when the plugin is not built (drone every gazebo.pose_period, marker when it moves).
    local=False (running on Windows): UDP to the WSL side (pose plugin or wsl_sync_gazebo.py)."""

    def __init__(self, local: bool):
        self.local = local
        self.transport = pose_transport() if local else "plugin"
        self.sender = PoseSender("127.0.0.1" if local else None)
        self.env = gz_environment() if local and self.transport != "plugin" else None
        self.proc = None
        self.next_pose = 0.0
        self.last_marker = None

    def describe(self) -> str:
        if not self.local:
            return f"UDP to WSL {self.sender.host}:{cfg('gazebo.udp_port')}"
        return "UDP pose plugin" if self.transport == "plugin" else f"gz model every {cfg('gazebo.pose_period')} s"

    def send(self, t: float, pose, marker) -> None:
        import math
        if self.transport == "plugin":
            self.sender.send(cfg("gazebo.udp_port"), pose)
            self.sender.send(cfg("gazebo.target_udp_port"), (*marker, 0.0, 0.0, 0.0))
            return
        if t >= self.next_pose and (self.proc is None or self.proc.poll() is not None):
            self.next_pose = t + cfg("gazebo.pose_period")
            try:
                self.proc = subprocess.Popen(gz_model_command(*pose), env=self.env,
                                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError:
                self.proc = None
        if self.last_marker is None or math.dist(marker, self.last_marker) > cfg("gazebo.marker_update_distance"):
            self.last_marker = tuple(marker)
            try:
                subprocess.Popen(gz_model_command(*marker, 0.0, 0.0, 0.0, model="target_marker"), env=self.env,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError:
                pass
