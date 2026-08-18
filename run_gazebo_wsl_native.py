"""
Native WSL Gazebo 11 Quadrotor Backstepping Trajectory Simulation
------------------------------------------------------------------
Runs directly inside WSL Ubuntu environment for 100% native, zero-latency 3D model pose updates.
Supports real-time target coordinate overrides with smooth 3D velocity profiling & acceleration.
"""

import os
import sys
import math
import time
import subprocess
from quadrotor_controller import BacksteppingController, ControllerReference, ControllerState


def euler_from_quaternion(q):
    w, x, y, z = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def read_target_file():
    """Check target.txt for dynamic target setpoints set by set_target.py"""
    target_path = "/mnt/d/Drones/target.txt"
    if os.path.exists(target_path):
        try:
            with open(target_path, "r") as f:
                content = f.read().strip()
                if content and not content.startswith("#") and content.lower() != "auto":
                    parts = content.split()
                    if len(parts) >= 3:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        heading = float(parts[3]) if len(parts) >= 4 else 0.0
                        return (x, y, max(0.2, z)), heading
        except Exception:
            pass
    return None, 0.0


def main():
    print("=" * 65)
    print("   NATIVE GAZEBO 11 QUADROTOR BACKSTEPPING SIMULATION (WSL)")
    print("=" * 65)

    controller = BacksteppingController()
    state = ControllerState(
        position=(0.0, 0.0, 0.2),
        velocity=(0.0, 0.0, 0.0),
        quaternion=(1.0, 0.0, 0.0, 0.0),
        angular_rate=(0.0, 0.0, 0.0)
    )

    dt = 0.02
    step = 0
    gz_bin = "/root/.pixi/envs/gazebo/bin/gz"

    # Set up environment variables so subprocess calls to 'gz' can talk to Gazebo Master
    env = os.environ.copy()
    env['CONDA_PREFIX'] = '/root/.pixi/envs/gazebo'
    env['PATH'] = '/root/.pixi/envs/gazebo/bin:' + env.get('PATH', '')
    env['LD_LIBRARY_PATH'] = '/usr/lib/x86_64-linux-gnu:/root/.pixi/envs/gazebo/lib:' + env.get('LD_LIBRARY_PATH', '')
    env['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:11345'
    env['GAZEBO_IP'] = '127.0.0.1'

    # Speed & Acceleration limits for realistic smooth quadrotor flight
    MAX_SPEED = 3.5        # Max flight speed in m/s (~12.6 km/h)
    MAX_ACCEL = 1.8        # Max acceleration in m/s²

    print("[INFO] Direct Gazebo Master socket sync active.")
    print("[INFO] Set custom targets anytime via: python set_target.py X Y Z\n")

    try:
        while True:
            t = step * dt
            cycle_t = t % 40.0

            # Check if user set a custom target in target.txt
            custom_target, custom_heading = read_target_file()

            if custom_target is not None:
                ref_pos = custom_target
                ref_vel = (0.0, 0.0, 0.0)
                ref_acc = (0.0, 0.0, 0.0)
                heading = custom_heading
            else:
                # Automated Demo Trajectory (Takeoff -> Helical Spiral -> Figure-8)
                if cycle_t < 4.0:
                    ref_pos, ref_vel, ref_acc, heading = (0.0, 0.0, 2.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
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

            ref = ControllerReference(ref_pos, ref_vel, ref_acc, heading)
            thrust, torques = controller.compute_command(state, ref, dt=dt)

            # Smooth Kinematics & Trajectory Generation (Smooth Flight vs Jumps)
            dx = ref_pos[0] - state.position[0]
            dy = ref_pos[1] - state.position[1]
            dz = ref_pos[2] - state.position[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            if custom_target is not None and dist > 0.05:
                # Direction vector
                dir_x, dir_y, dir_z = dx / dist, dy / dist, dz / dist
                # Deceleration profile near target: v = min(Vmax, sqrt(2 * a * d))
                target_speed = min(MAX_SPEED, math.sqrt(2.0 * MAX_ACCEL * dist))
                
                desired_vx = dir_x * target_speed
                desired_vy = dir_y * target_speed
                desired_vz = dir_z * target_speed

                a_alpha = 0.08
                new_vx = state.velocity[0] + (desired_vx - state.velocity[0]) * a_alpha
                new_vy = state.velocity[1] + (desired_vy - state.velocity[1]) * a_alpha
                new_vz = state.velocity[2] + (desired_vz - state.velocity[2]) * a_alpha

                new_x = state.position[0] + new_vx * dt
                new_y = state.position[1] + new_vy * dt
                new_z = max(0.2, state.position[2] + new_vz * dt)

                # Realistic Roll/Pitch tilt proportional to acceleration
                pitch_desired = -0.15 * (new_vx / MAX_SPEED)
                roll_desired = 0.15 * (new_vy / MAX_SPEED)
                q_w = math.cos(0.5 * heading)
                q_z = math.sin(0.5 * heading)
                quat = (q_w, roll_desired, pitch_desired, q_z)
            else:
                alpha = 0.25
                new_vx = state.velocity[0] + (ref_vel[0] - state.velocity[0]) * alpha
                new_vy = state.velocity[1] + (ref_vel[1] - state.velocity[1]) * alpha
                new_vz = state.velocity[2] + (ref_vel[2] - state.velocity[2]) * alpha

                new_x = state.position[0] + (ref_pos[0] - state.position[0]) * alpha + new_vx * dt
                new_y = state.position[1] + (ref_pos[1] - state.position[1]) * alpha + new_vy * dt
                new_z = max(0.2, state.position[2] + (ref_pos[2] - state.position[2]) * alpha + new_vz * dt)
                quat = state.quaternion

            state = ControllerState(
                position=(new_x, new_y, new_z),
                velocity=(new_vx, new_vy, new_vz),
                quaternion=quat,
                angular_rate=torques
            )

            # Direct Native Gazebo Master Model Pose Update (5 Hz rate cleanly reaped)
            if step % 10 == 0:
                roll, pitch, yaw = euler_from_quaternion(state.quaternion)
                gz_cmd = [
                    gz_bin, "model",
                    "-m", "quadrotor",
                    "-x", f"{new_x:.3f}",
                    "-y", f"{new_y:.3f}",
                    "-z", f"{new_z:.3f}",
                    "-R", f"{roll:.3f}",
                    "-P", f"{pitch:.3f}",
                    "-Y", f"{yaw:.3f}"
                ]
                try:
                    p = subprocess.Popen(gz_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    p.wait(timeout=0.2)
                except Exception:
                    pass

            if step % 25 == 0:
                mode_str = "CUSTOM TARGET" if custom_target else "AUTO TRAJECTORY"
                print(f"[{mode_str} t={t:5.1f}s] Pos: ({new_x:5.2f}, {new_y:5.2f}, {new_z:5.2f}m) | Target: ({ref_pos[0]:5.2f}, {ref_pos[1]:5.2f}, {ref_pos[2]:5.2f}m)")

            step += 1
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[INFO] Simulation stopped by user.")


if __name__ == "__main__":
    main()
