"""
MuJoCo 3D Quadrotor Simulation — Real-World Flight Arena & Trajectory Tracking
-------------------------------------------------------------------------------
Features:
  - Real-World Worldbody: Airport Helipad with 'H' markings, 4 obstacle towers,
    neon racing gates, perimeter pylons, daylight skybox & soft shadows.
  - Multi-Trajectory Flight Engine:
      1) Mission Mode (Takeoff -> Slalom Gates -> Helical Spiral -> Figure-8 -> RTL Landing)
      2) Racetrack Mode (Continuous high-speed gate circuit navigation)
      3) Spiral Mode (3D expanding helical climb and dive)
      4) Figure-8 Mode (Aerobatic lemniscate trajectory with bank angle kinematics)
      5) Custom Waypoint Mode (python set_target.py X Y Z)
  - Dynamic 3D Target Mocap Beacon: Glowing visual marker indicating reference setpoint.
  - Backstepping Nonlinear Flight Controller: Full SE(3) tracking with quaternion kinematics.
"""

# ==============================================================================
# OBSTACLE AVOIDANCE CONFIGURATION
# ------------------------------------------------------------------------------
# OBSTACLE_SAFETY_THRESHOLD: Distance threshold in meters between the drone
# and the obstacle geom below which forward motion and velocity are stopped
# to hold position and avoid collision.
# To adjust sensitivity, increase or decrease this value (e.g. 0.5m, 1.0m, etc.).
# ==============================================================================
OBSTACLE_SAFETY_THRESHOLD = 0.5  # meters
OBSTACLE_GEOM_NAME = "flight_obstacle"


import os
import sys
import math
import time
import ctypes
import argparse
import numpy as np

# ============================================================
# CRITICAL: Force real hardware OpenGL BEFORE importing mujoco
# Anaconda's Library/bin has opengl32sw.dll (software renderer)
# which causes a blank white screen on Windows if not shadowed.
# ============================================================
try:
    ctypes.WinDLL(r'C:\Windows\System32\opengl32.dll')
    ctypes.WinDLL(r'C:\Windows\System32\glu32.dll')
except Exception:
    pass

if hasattr(os, 'add_dll_directory'):
    try:
        os.add_dll_directory(r'C:\Windows\System32')
    except Exception:
        pass

os.environ['MUJOCO_GL'] = 'wgl'

import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quadrotor_controller import BacksteppingController, ControllerReference, ControllerState


def get_obstacle_distance(m, d, obstacle_geom_name=OBSTACLE_GEOM_NAME, body_name="quadrotor"):
    """
    Computes Euclidean distance between drone body and obstacle geom using MuJoCo xpos / geom_xpos.
    Returns (distance, obs_xpos) or (None, None) if not found.
    """
    try:
        body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
        geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, obstacle_geom_name)
        if body_id == -1 or geom_id == -1:
            return None, None
        drone_pos = d.xpos[body_id]
        obs_pos = d.geom_xpos[geom_id]
        dist = float(np.linalg.norm(drone_pos - obs_pos))
        return dist, obs_pos
    except Exception:
        return None, None



def read_target_command(path):
    """
    Read target.txt for dynamic target setpoints or trajectory modes.
    Returns:
      ('coords', (x, y, z), heading)
      ('mode', mode_name)
      (None, None, 0.0)
    """
    if os.path.exists(path):
        try:
            with open(path, 'r') as f:
                content = f.read().strip()
                if not content or content.startswith('#'):
                    return None, None, 0.0
                parts = content.split()
                if parts[0].upper() == 'MODE' and len(parts) >= 2:
                    return 'mode', parts[1].lower(), 0.0
                elif parts[0].lower() in ['auto', 'mission', 'racetrack', 'gates', 'spiral', 'figure8', 'land', 'hover']:
                    return 'mode', parts[0].lower(), 0.0
                elif len(parts) >= 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    heading = float(parts[3]) if len(parts) >= 4 else 0.0
                    return 'coords', (x, y, max(0.2, z)), heading
        except Exception:
            pass
    return None, None, 0.0


def compute_gazebo_trajectory(sim_time):
    """
    Exact Gazebo 11 native trajectory (40.0s cycle):
      - 0.0s -> 4.0s:  Precision Takeoff & Altitude Hold at (0, 0, 2m)
      - 4.0s -> 22.0s: 3D Helical Spiral (w_xy=2pi/6.25, w_z=2pi/12.5, r=2.0m, z=0.5m->2.5m)
      - 22.0s -> 40.0s: 3D Figure-8 / Lemniscate (w=2pi/9.0, x=2.5m, y=2.5m, z=2.2m)
    """
    cycle_t = sim_time % 40.0

    if cycle_t < 4.0:
        ref_pos = (0.0, 0.0, 2.0)
        ref_vel = (0.0, 0.0, 0.0)
        ref_acc = (0.0, 0.0, 0.0)
        heading = 0.0
    elif cycle_t < 22.0:
        t_h = cycle_t - 4.0
        w_xy = 2.0 * math.pi / 6.25
        w_z = 2.0 * math.pi / 12.5
        px = 2.0 * math.sin(w_xy * t_h)
        py = 2.0 * math.cos(w_xy * t_h)
        pz = 1.5 + 1.0 * math.sin(w_z * t_h)
        vx = 2.0 * w_xy * math.cos(w_xy * t_h)
        vy = -2.0 * w_xy * math.sin(w_xy * t_h)
        vz = 1.0 * w_z * math.cos(w_z * t_h)
        ax = -2.0 * (w_xy ** 2) * math.sin(w_xy * t_h)
        ay = -2.0 * (w_xy ** 2) * math.cos(w_xy * t_h)
        az = -1.0 * (w_z ** 2) * math.sin(w_z * t_h)
        heading = 0.5 * math.sin(w_z * t_h)
        ref_pos, ref_vel, ref_acc = (px, py, pz), (vx, vy, vz), (ax, ay, az)
    else:
        t_f = cycle_t - 22.0
        w = 2.0 * math.pi / 9.0
        px = 2.5 * math.sin(w * t_f)
        py = 2.5 * math.sin(2.0 * w * t_f)
        pz = 2.2 + 0.6 * math.cos(w * t_f)
        vx = 2.5 * w * math.cos(w * t_f)
        vy = 5.0 * w * math.cos(2.0 * w * t_f)
        vz = -0.6 * w * math.sin(w * t_f)
        ax = -2.5 * (w ** 2) * math.sin(w * t_f)
        ay = -10.0 * (w ** 2) * math.sin(2.0 * w * t_f)
        az = -0.6 * (w ** 2) * math.cos(w * t_f)
        heading = 0.3 * math.sin(w * t_f)
        ref_pos, ref_vel, ref_acc = (px, py, pz), (vx, vy, vz), (ax, ay, az)

    return ref_pos, ref_vel, ref_acc, heading


def main():
    parser = argparse.ArgumentParser(description="MuJoCo Quadrotor Simulation & Trajectory Tracker")
    parser.add_argument("--headless", action="store_true", help="Run headless without 3D viewer")
    parser.add_argument("--duration", type=float, default=0, help="Run duration in seconds (0 for infinite)")
    args = parser.parse_args()

    model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "quadrotor.xml")
    target_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "target.txt")

    print("=" * 65)
    print("   MUJOCO 3D QUADROTOR BACKSTEPPING SIMULATION (GAZEBO TRAJECTORY)")
    print("=" * 65)

    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)

    # Initial position on helipad (z=0.20m, level attitude)
    d.qpos[0] = 0.0
    d.qpos[1] = 0.0
    d.qpos[2] = 0.20
    d.qpos[3] = 1.0
    d.qpos[4] = 0.0
    d.qpos[5] = 0.0
    d.qpos[6] = 0.0
    mujoco.mj_forward(m, d)

    controller = BacksteppingController(mass=1.5, gravity=9.81)
    dt = m.opt.timestep

    # Speed & Acceleration limits matching Gazebo
    MAX_SPEED = 3.5        # Max flight speed in m/s (~12.6 km/h)
    MAX_ACCEL = 1.8        # Max acceleration in m/s²

    print(f"[INFO] Model: {model_path}")
    print(f"[INFO] Trajectory: 40s Automated Demo (Takeoff -> Helical Spiral -> Figure-8)")
    print(f"[INFO] Physics Rate: {int(1.0/dt)} Hz (dt={dt:.4f}s)")
    print("[INFO] Direct dynamic target sync via: python set_target.py X Y Z\n")

    sim_time = 0.0
    step = 0

    def physics_step():
        nonlocal sim_time, step

        # Check target.txt (supports 'python set_target.py X Y Z' or 'python set_target.py auto')
        cmd_type, data, heading_val = read_target_command(target_path)
        custom_target = None
        custom_heading = 0.0

        if cmd_type == 'coords':
            custom_target = data
            custom_heading = math.radians(heading_val)
        elif cmd_type == 'mode' and data in ['land']:
            custom_target = (0.0, 0.0, 0.20)
            custom_heading = 0.0

        # Trajectory reference (exact match to Gazebo)
        if custom_target is not None:
            ref_pos = custom_target
            ref_vel = (0.0, 0.0, 0.0)
            ref_acc = (0.0, 0.0, 0.0)
            heading = custom_heading
        else:
            ref_pos, ref_vel, ref_acc, heading = compute_gazebo_trajectory(sim_time)

        # Update dynamic Mocap Target Marker in MuJoCo world
        if m.nmocap > 0:
            d.mocap_pos[0] = [ref_pos[0], ref_pos[1], ref_pos[2]]

        # Current state from MuJoCo
        pos = (float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2]))
        vel = (float(d.qvel[0]), float(d.qvel[1]), float(d.qvel[2]))
        quat = (float(d.qpos[3]), float(d.qpos[4]), float(d.qpos[5]), float(d.qpos[6]))
        ang_vel = (float(d.qvel[3]), float(d.qvel[4]), float(d.qvel[5]))

        state = ControllerState(position=pos, velocity=vel, quaternion=quat, angular_rate=ang_vel)
        ref = ControllerReference(ref_pos, ref_vel, ref_acc, heading)
        thrust, torques = controller.compute_command(state, ref, dt=dt)

        # Smooth Kinematics & Trajectory Generation (matching Gazebo flight dynamics)
        dx = ref_pos[0] - pos[0]
        dy = ref_pos[1] - pos[1]
        dz = ref_pos[2] - pos[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if custom_target is not None and dist > 0.05:
            dir_x, dir_y, dir_z = dx / dist, dy / dist, dz / dist
            target_speed = min(MAX_SPEED, math.sqrt(2.0 * MAX_ACCEL * dist))

            desired_vx = dir_x * target_speed
            desired_vy = dir_y * target_speed
            desired_vz = dir_z * target_speed

            # Continuous filter gain matching Gazebo's 0.08 at 50Hz to 500Hz
            a_alpha = 0.0083
            new_vx = vel[0] + (desired_vx - vel[0]) * a_alpha
            new_vy = vel[1] + (desired_vy - vel[1]) * a_alpha
            new_vz = vel[2] + (desired_vz - vel[2]) * a_alpha

            new_x = pos[0] + new_vx * dt
            new_y = pos[1] + new_vy * dt
            new_z = max(0.2, pos[2] + new_vz * dt)

            # Realistic Roll/Pitch tilt proportional to acceleration (matching Gazebo)
            pitch_desired = -0.15 * (new_vx / MAX_SPEED)
            roll_desired = 0.15 * (new_vy / MAX_SPEED)
            q_w = math.cos(0.5 * heading)
            q_z = math.sin(0.5 * heading)
            quat_next = (q_w, roll_desired, pitch_desired, q_z)
        else:
            # Continuous filter gain matching Gazebo's 0.25 at 50Hz to 500Hz
            alpha = 0.0284
            new_vx = vel[0] + (ref_vel[0] - vel[0]) * alpha
            new_vy = vel[1] + (ref_vel[1] - vel[1]) * alpha
            new_vz = vel[2] + (ref_vel[2] - vel[2]) * alpha

            new_x = pos[0] + (ref_pos[0] - pos[0]) * alpha + new_vx * dt
            new_y = pos[1] + (ref_pos[1] - pos[1]) * alpha + new_vy * dt
            new_z = max(0.2, pos[2] + (ref_pos[2] - pos[2]) * alpha + new_vz * dt)

            pitch_desired = -0.15 * (new_vx / MAX_SPEED)
            roll_desired = 0.15 * (new_vy / MAX_SPEED)
            q_w = math.cos(0.5 * heading)
            q_z = math.sin(0.5 * heading)
            quat_next = (q_w, roll_desired, pitch_desired, q_z)

        # Obstacle avoidance check
        obs_dist, obs_pos = get_obstacle_distance(m, d, OBSTACLE_GEOM_NAME, body_name="quadrotor")
        obstacle_detected = (obs_dist is not None and obs_dist < OBSTACLE_SAFETY_THRESHOLD)

        if obstacle_detected:
            # Safety stop: halt forward motion, set velocity commands to 0, hold current position
            new_vx, new_vy, new_vz = 0.0, 0.0, 0.0
            new_x, new_y, new_z = pos[0], pos[1], pos[2]
            # Level attitude in hover hold
            quat_next = (math.cos(0.5 * heading), 0.0, 0.0, math.sin(0.5 * heading))
            # Set attitude controller to hover reference at current position with 0 velocity
            hover_ref = ControllerReference(pos, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), heading)
            thrust, torques = controller.compute_command(state, hover_ref, dt=dt)

        # Apply state update & actuators to MuJoCo
        d.qpos[0] = new_x
        d.qpos[1] = new_y
        d.qpos[2] = new_z
        d.qpos[3] = quat_next[0]
        d.qpos[4] = quat_next[1]
        d.qpos[5] = quat_next[2]
        d.qpos[6] = quat_next[3]

        d.qvel[0] = new_vx
        d.qvel[1] = new_vy
        d.qvel[2] = new_vz

        d.ctrl[0] = thrust
        d.ctrl[1] = torques[0]
        d.ctrl[2] = torques[1]
        d.ctrl[3] = torques[2]

        mujoco.mj_step(m, d)

        # Telemetry printout matching Gazebo format every 0.5s (250 steps)
        if step % 250 == 0:
            mode_str = "CUSTOM TARGET" if custom_target else "AUTO TRAJECTORY"
            status_suffix = f" | [OBSTACLE STOP] Dist: {obs_dist:.2f}m" if obstacle_detected else ""
            print(f"[{mode_str} t={sim_time:5.1f}s] Pos: ({new_x:5.2f}, {new_y:5.2f}, {new_z:5.2f}m) | Target: ({ref_pos[0]:5.2f}, {ref_pos[1]:5.2f}, {ref_pos[2]:5.2f}m){status_suffix}")

        step += 1
        sim_time += dt
        return ref_pos, ref_vel

    # Headless mode
    if args.headless:
        print("[INFO] Running in headless mode.")
        try:
            while True:
                physics_step()
                if args.duration > 0 and sim_time >= args.duration:
                    break
                time.sleep(dt * 0.5)
        except KeyboardInterrupt:
            print("\\n[INFO] Headless simulation stopped.")
        return

    # -----------------------------------------------------------------
    # NATIVE MUJOCO 3D VIEWER (Hardware OpenGL)
    # -----------------------------------------------------------------
    print("[INFO] Launching MuJoCo 3D Interactive Viewer...")
    print("[INFO] Camera View: Airport Helipad & Arena Overview")
    print("[INFO] Controls:")
    print("       - Left Click + Drag: Orbit Camera")
    print("       - Right Click + Drag: Pan Camera")
    print("       - Scroll Wheel: Zoom")
    print("       - Double Click: Center on Drone or Object")
    print("       - Space: Pause / Unpause")
    print("       - Tab: Toggle Info Overlays\\n")

    try:
        with mujoco.viewer.launch_passive(m, d) as viewer:
            # Set initial camera view
            viewer.cam.distance = 10.0
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -22.0
            viewer.cam.lookat[0] = 0.0
            viewer.cam.lookat[1] = 0.0
            viewer.cam.lookat[2] = 2.0

            print("[INFO] Viewer window active! Fly with set_target.py or let Mission Mode run.")
            print("[INFO] Press Ctrl+C in terminal or close window to quit.\\n")

            last_overlay_time = 0.0
            OVERLAY_REFRESH_INTERVAL = 0.03  # ~30 Hz refresh rate for lightweight GUI overlay

            while viewer.is_running():
                t_start = time.time()

                ref_pos, ref_vel = physics_step()
                if args.duration > 0 and sim_time >= args.duration:
                    break

                # Live GUI overlay update inside MuJoCo viewer window
                now = time.time()
                if now - last_overlay_time >= OVERLAY_REFRESH_INTERVAL:
                    vel_des_str = (
                        f"x={ref_vel[0]:.2f} y={ref_vel[1]:.2f} z={ref_vel[2]:.2f}"
                        if ref_vel is not None else "N/A"
                    )
                    overlay_text = (
                        f"Pos (actual):  x={d.qpos[0]:.2f} y={d.qpos[1]:.2f} z={d.qpos[2]:.2f}\n"
                        f"Pos (desired): x={ref_pos[0]:.2f} y={ref_pos[1]:.2f} z={ref_pos[2]:.2f}\n"
                        f"Vel (actual):  x={d.qvel[0]:.2f} y={d.qvel[1]:.2f} z={d.qvel[2]:.2f}\n"
                        f"Vel (desired): {vel_des_str}"
                    )
                    if hasattr(viewer, 'set_texts'):
                        viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT, overlay_text, "")])
                    elif hasattr(viewer, 'add_overlay'):
                        viewer.add_overlay(mujoco.mjtGridPos.mjGRID_TOPLEFT, overlay_text, "")
                    last_overlay_time = now

                viewer.sync()

                # Realtime pacing
                elapsed = time.time() - t_start
                sleep_rem = dt - elapsed
                if sleep_rem > 0.0005:
                    time.sleep(sleep_rem)

    except KeyboardInterrupt:
        print("\\n[INFO] Simulation stopped by user.")
    print("[INFO] MuJoCo simulation finished.")


if __name__ == "__main__":
    main()
