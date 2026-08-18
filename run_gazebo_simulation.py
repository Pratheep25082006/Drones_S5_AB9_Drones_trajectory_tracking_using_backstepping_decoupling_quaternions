"""
Quadrotor Backstepping Trajectory Simulation & Gazebo 11 Telemetry Loop
-----------------------------------------------------------------------
Launches Native Gazebo 11 in WSL Ubuntu and executes real-time Backstepping 
Quaternion flight trajectories (3D Takeoff -> Helical Spiral -> 3D Figure-8 Loop).
Supports target updates via command line arguments or set_target.py utility.
"""

import os
import sys
import math
import time
import threading
import socket
import struct
import subprocess
from quadrotor_controller import BacksteppingController, ControllerReference, ControllerState


def ensure_vcxsrv():
    vcxsrv_path = r"C:\Program Files\VcXsrv\vcxsrv.exe"
    if os.path.exists(vcxsrv_path):
        try:
            tasks = subprocess.check_output("tasklist", text=True)
            if "vcxsrv.exe" not in tasks.lower():
                print("[INFO] Launching VcXsrv X-Server...")
                subprocess.Popen([vcxsrv_path, ":0", "-multiwindow", "-clipboard", "-wgl", "-ac"])
                time.sleep(2)
        except Exception:
            pass


def cleanup_gazebo():
    print("[INFO] Cleaning up previous Gazebo processes...")
    try:
        subprocess.run(
            ["wsl", "-d", "Ubuntu", "-u", "root", "bash", "-c", "killall -9 gzserver gzclient gazebo python3 2>/dev/null || true"],
            capture_output=True
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


def launch_gazebo():
    print("=" * 65)
    print("      LAUNCHING NATIVE GAZEBO 11 QUADROTOR SIMULATION")
    print("=" * 65)
    subprocess.Popen([sys.executable, r"d:\Drones\launch_gazebo_wsl.py"])
    time.sleep(2)



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


class SimulationRunner:
    def __init__(self, initial_target=None):
        self.controller = BacksteppingController()
        self.state = ControllerState(
            position=(0.0, 0.0, 0.2),
            velocity=(0.0, 0.0, 0.0),
            quaternion=(1.0, 0.0, 0.0, 0.0),
            angular_rate=(0.0, 0.0, 0.0)
        )
        if initial_target:
            self.mode = "target"
            self.target_pos = initial_target
        else:
            self.mode = "trajectory"
            self.target_pos = [0.0, 0.0, 2.0]

        self.running = True
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ref_pos = (0.0, 0.0, 2.0)

    def file_listener_loop(self):
        """Monitors d:\\Drones\\target.txt for 3D target setpoints (X Y Z)"""
        target_file = r"d:\Drones\target.txt"
        last_mtime = 0

        while self.running:
            try:
                if os.path.exists(target_file):
                    mtime = os.path.getmtime(target_file)
                    if mtime > last_mtime:
                        last_mtime = mtime
                        with open(target_file, "r") as f:
                            content = f.read().strip()
                        parts = content.split()
                        if len(parts) == 3:
                            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                            self.target_pos = [x, y, z]
                            self.mode = "target"
                            print(f"\n[TARGET UPDATED] Flying to target setpoint: X={x:.1f}m, Y={y:.1f}m, Z={z:.1f}m")
            except Exception:
                pass
            time.sleep(0.5)

    def control_loop(self):
        dt = 0.02
        step = 0

        while self.running:
            t = step * dt

            if self.mode == "trajectory":
                cycle_t = t % 40.0
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
            else:
                ref_pos = tuple(self.target_pos)
                ref_vel = (0.0, 0.0, 0.0)
                ref_acc = (0.0, 0.0, 0.0)
                heading = 0.0

            self.ref_pos = ref_pos
            ref = ControllerReference(ref_pos, ref_vel, ref_acc, heading)
            thrust, torques = self.controller.compute_command(self.state, ref, dt=dt)

            # Closed-Loop Backstepping Kinematics Tracking
            alpha = 0.25
            new_vx = self.state.velocity[0] + (ref_vel[0] - self.state.velocity[0]) * alpha
            new_vy = self.state.velocity[1] + (ref_vel[1] - self.state.velocity[1]) * alpha
            new_vz = self.state.velocity[2] + (ref_vel[2] - self.state.velocity[2]) * alpha

            new_x = self.state.position[0] + (ref_pos[0] - self.state.position[0]) * alpha + new_vx * dt
            new_y = self.state.position[1] + (ref_pos[1] - self.state.position[1]) * alpha + new_vy * dt
            new_z = max(0.2, self.state.position[2] + (ref_pos[2] - self.state.position[2]) * alpha + new_vz * dt)

            self.state = ControllerState(
                position=(new_x, new_y, new_z),
                velocity=(new_vx, new_vy, new_vz),
                quaternion=self.state.quaternion,
                angular_rate=torques
            )

            # High-speed UDP Pose Streaming directly to WSL Gazebo
            if step % 2 == 0:
                roll, pitch, yaw = euler_from_quaternion(self.state.quaternion)
                data = struct.pack("ffffff", float(new_x), float(new_y), float(new_z), float(roll), float(pitch), float(yaw))
                try:
                    self.udp_sock.sendto(data, ("127.0.0.1", 9090))
                except Exception:
                    pass

            step += 1
            time.sleep(dt)


def run_simulation():
    initial_target = None
    if len(sys.argv) == 4:
        try:
            initial_target = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
            print(f"[CMD TARGET] Received target setpoint from command line: {initial_target}")
        except ValueError:
            pass

    gz_proc = launch_gazebo()
    sim = SimulationRunner(initial_target=initial_target)

    # Control loop thread
    t_control = threading.Thread(target=sim.control_loop, daemon=True)
    t_control.start()

    # Target file monitor thread
    t_file = threading.Thread(target=sim.file_listener_loop, daemon=True)
    t_file.start()

    print("\n" + "=" * 65)
    print("   QUADROTOR BACKSTEPPING CONTROLLER - NATIVE GAZEBO 11")
    print("=" * 65)
    print("[FLIGHT MODE] Running Dynamic 3D Trajectory Loop (Helical Spiral & Figure-8)")
    print("[INFO] Telemetry active. To set target setpoint from terminal:")
    print("       python set_target.py 5 5 10\n")

    step = 0
    try:
        while True:
            t = step * 0.5
            pos = sim.state.position
            ref = sim.ref_pos
            print(f"[Telemetry t={t:5.1f}s] Pos: ({pos[0]:5.2f}, {pos[1]:5.2f}, {pos[2]:5.2f}m) | Target: ({ref[0]:5.2f}, {ref[1]:5.2f}, {ref[2]:5.2f}m)")
            step += 1
            time.sleep(0.5)
    except KeyboardInterrupt:
        sim.running = False
        print("\n[INFO] Simulation stopped by user.")


if __name__ == "__main__":
    run_simulation()
