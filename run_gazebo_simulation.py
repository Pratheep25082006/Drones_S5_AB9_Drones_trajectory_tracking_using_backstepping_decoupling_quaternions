"""
Gazebo 11 simulation launched from Windows - full De Monte & Lohmann (2013) controller
-------------------------------------------------------------------------------------
* Starts Gazebo 11 in WSL (pose plugin, or wsl_sync_gazebo.py when the plugin is not built).
* Computes the flight on Windows with PythonFlightSim and streams drone + target poses
  over UDP (gazebo.udp_host / udp_port / target_udp_port).
Settings: config.yaml.  If UDP from Windows does not reach WSL on your machine, use
START_GAZEBO_GUI.bat (the flight then runs inside WSL).

Usage:
    python run_gazebo_simulation.py            # auto mission
    python run_gazebo_simulation.py 5 5 10     # fly to a set point
"""

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller import PythonFlightSim  # noqa: E402
from quadrotor_controller.units import CM_PER_M  # noqa: E402
from quadrotor_controller.gazebo_tools import PoseSender, pose_transport  # noqa: E402
from quadrotor_controller.config import add_config_arguments, apply_config_arguments, cfg, project_path  # noqa: E402

def main():
    ap = argparse.ArgumentParser(description="Gazebo simulation (Windows side)")
    ap.add_argument("target", nargs="*", help="optional set point: X Y Z [heading_deg]")
    ap.add_argument("--no-gazebo", action="store_true", help="do not start Gazebo (stream only)")
    add_config_arguments(ap)
    args = ap.parse_args()
    apply_config_arguments(args)
    initial = " ".join(args.target) if len(args.target) >= 3 else None
    if not args.no_gazebo:
        transport = pose_transport()
        print(f"[INFO] Launching Gazebo 11 (WSL), pose transport: {transport} ...")
        try:
            cmd = [sys.executable, os.path.join(HERE, "launch_gazebo_wsl.py"), "--receiver"]
            if transport == "plugin":
                cmd.append("--no-companion")
            subprocess.Popen(cmd)
        except OSError as e:
            print("[WARN] Could not launch Gazebo:", e)
        time.sleep(cfg("gazebo.launch_wait"))
    sim = PythonFlightSim(target_path=project_path("target_file"), initial_command=initial)
    sender = PoseSender()
    frame_dt = cfg("python_sim.frame_dt")
    print_frames = max(1, int(round(cfg("logging.print_period") / frame_dt)))
    print(f"[INFO] Streaming poses to udp://{sender.host}:{cfg('gazebo.udp_port')} (target :{cfg('gazebo.target_udp_port')}) every {frame_dt} s\n")
    frame = 0
    try:
        while True:
            t0 = time.time()
            sim.advance(frame_dt)
            sender.send(cfg("gazebo.udp_port"), (*sim.plant.x, *sim.plant.euler()))
            if sim.last_ref is not None:
                sender.send(cfg("gazebo.target_udp_port"), (*sim.marker_position(), 0.0, 0.0, 0.0))
            if frame % print_frames == 0:
                print(sim.status_line())
            frame += 1
            rem = frame_dt - (time.time() - t0)
            if rem > 0:
                time.sleep(rem)
    except KeyboardInterrupt:
        print("\n[INFO] Simulation stopped by user.")


if __name__ == "__main__":
    main()
