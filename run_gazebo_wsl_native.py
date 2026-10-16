"""
Native WSL Gazebo 11 runner - full De Monte & Lohmann (2013) backstepping controller
------------------------------------------------------------------------------------
Runs inside WSL (started by run_gazebo.sh / START_GAZEBO_GUI.bat).  The flight is
computed by PythonFlightSim (rigid body + paper controller + noise + Kalman filter).
Poses go to Gazebo through the UDP pose plugin (smooth, every frame) or, if the plugin
is not built, through `gz model` calls.  The reference is shown by the target marker.
Settings: config.yaml (python_sim, gazebo, logging, ...).
"""

import argparse
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller import PythonFlightSim  # noqa: E402
from quadrotor_controller.config import add_config_arguments, apply_config_arguments, banner, cfg, project_path  # noqa: E402
from quadrotor_controller.gazebo_tools import PoseSender, gz_environment, gz_model_command, pose_transport  # noqa: E402
from quadrotor_controller.units import CM_PER_M  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="Gazebo (WSL) runner")
    add_config_arguments(ap)
    apply_config_arguments(ap.parse_args())
    frame_dt, pose_period = cfg("python_sim.frame_dt"), cfg("gazebo.pose_period")
    print_frames = max(1, int(round(cfg("logging.print_period") / frame_dt)))

    print(banner("NATIVE GAZEBO 11 - DE MONTE & LOHMANN (2013) BACKSTEPPING (WSL)"))
    sim = PythonFlightSim(target_path=project_path("target_file"))
    transport = pose_transport()
    sender = PoseSender("127.0.0.1")
    env = gz_environment()
    proc, next_pose, frame, last_marker = None, 0.0, 0, None
    print(f"[INFO] Physics {1 / sim.dt:.0f} Hz (Python rigid body) -> Gazebo via "
          f"{'UDP pose plugin every frame' if transport == 'plugin' else f'gz model every {pose_period} s'}")
    print("[INFO] Commands: python set_target.py X Y Z | auto | spiral | figure8 | hover | land\n")
    try:
        while True:
            t0 = time.time()
            sim.advance(frame_dt)
            pose = (*sim.plant.x, *sim.plant.euler())
            ref = sim.marker_position()
            if transport == "plugin":
                sender.send(cfg("gazebo.udp_port"), pose)
                sender.send(cfg("gazebo.target_udp_port"), (*ref, 0.0, 0.0, 0.0))
            elif sim.t >= next_pose and (proc is None or proc.poll() is not None):
                next_pose = sim.t + pose_period
                try:
                    proc = subprocess.Popen(gz_model_command(*pose), env=env,
                                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except OSError:
                    proc = None
                if last_marker is None or math.dist(ref, last_marker) > cfg("gazebo.marker_update_distance"):
                    last_marker = ref                           # target marker: only when the goal moves
                    try:
                        subprocess.Popen(gz_model_command(*ref, 0.0, 0.0, 0.0, model="target_marker"), env=env,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    except OSError:
                        pass
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
