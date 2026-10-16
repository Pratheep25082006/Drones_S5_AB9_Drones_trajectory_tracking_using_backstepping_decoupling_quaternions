"""
WSL Gazebo UDP pose receiver: listens for (x, y, z, roll, pitch, yaw) float32 packets
from run_gazebo_simulation.py and applies the newest pose with `gz model`
(at most one gz process at a time, every gazebo.pose_period seconds).
Settings: config.yaml -> gazebo.
"""
import os
import socket
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller.config import cfg  # noqa: E402
from quadrotor_controller.gazebo_tools import gz_environment, gz_model_command  # noqa: E402

POSE_FORMAT = "ffffff"
POSE_SIZE = struct.calcsize(POSE_FORMAT)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", cfg("gazebo.udp_port")))
    sock.setblocking(False)
    print(f"[WSL Sync] Gazebo UDP pose listener on port {cfg('gazebo.udp_port')}.")
    env = gz_environment()
    latest, proc, last_sent = None, None, -float("inf")
    period, poll, bufsize = cfg("gazebo.pose_period"), cfg("gazebo.receiver_poll"), cfg("gazebo.udp_buffer")
    while True:
        try:
            while True:                       # drain the socket, keep only the newest pose
                data, _ = sock.recvfrom(bufsize)
                if len(data) == POSE_SIZE:
                    latest = struct.unpack(POSE_FORMAT, data)
        except BlockingIOError:
            pass
        now = time.time()
        if latest is not None and now - last_sent >= period and (proc is None or proc.poll() is not None):
            try:
                proc = subprocess.Popen(gz_model_command(*latest), env=env,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError:
                proc = None
            last_sent = now
        time.sleep(poll)


if __name__ == "__main__":
    main()
