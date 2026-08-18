"""
WSL Native Gazebo UDP Pose Receiver
Listens on UDP port 9090 and updates model pose directly inside Linux without spawning wsl.exe processes.
"""
import socket
import struct
import subprocess

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 9090))
print("[WSL Sync] Gazebo UDP Pose Listener active on port 9090.")

gz_bin = "/root/.pixi/envs/gazebo/bin/gz"

while True:
    try:
        data, _ = sock.recvfrom(1024)
        if len(data) == 24:
            x, y, z, roll, pitch, yaw = struct.unpack("ffffff", data)
            cmd = [
                gz_bin, "model",
                "-m", "quadrotor",
                "-x", f"{x:.3f}",
                "-y", f"{y:.3f}",
                "-z", f"{z:.3f}",
                "-R", f"{roll:.3f}",
                "-P", f"{pitch:.3f}",
                "-Y", f"{yaw:.3f}"
            ]
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass
