"""
gym-pybullet-drones 3D Quadrotor Simulation — Urban City World & Reactive LiDAR Avoidance
-----------------------------------------------------------------------------------------
Features:
  - Realistic 3D City Environment:
      * Skyscrapers, High-Rise Towers & Urban Building Blocks
      * Central Plaza Helipad, Avenues, Asphalt Roads with Lane Markings
      * Streetlamps & City Trees
  - 10-Beam Omnidirectional LiDAR Proximity Sensor System:
      * 8 Horizontal 360-degree Raycast Beams + 2 Vertical Clearance Beams
      * Real-Time Laser Beam Visualization (Green: Clear, Red: Detected Building)
  - Reactive Obstacle Avoidance & Path Replanning:
      * Artificial Potential Field (APF) Repulsion + Tangential Wall Circulation
      * Steers around building blocks without colliding or getting trapped
  - Nonlinear Quaternion Backstepping Flight Controller
  - Realistic Sensor Noise Injection + Discrete-Time Kalman Filter Denoising
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import List, Optional, Tuple

import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# Ensure project root is on path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from quadrotor_controller import (
    BacksteppingController,
    ControllerReference,
    ControllerState,
    KalmanFilter,
    LidarScan,
    QuadrotorStateEstimator,
    ReactiveObstacleAvoider,
    SensorNoiseModel,
)


def read_target_command(path: str):
    """
    Read target.txt for dynamic target setpoints or trajectory modes.
    Returns:
      ('coords', (x, y, z), heading)
      ('mode', mode_name)
      (None, None, 0.0)
    """
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                content = f.read().strip()
                if not content or content.startswith("#"):
                    return None, None, 0.0
                parts = content.split()
                if parts[0].upper() == "MODE" and len(parts) >= 2:
                    return "mode", parts[1].lower(), 0.0
                elif parts[0].lower() in [
                    "auto",
                    "mission",
                    "racetrack",
                    "spiral",
                    "figure8",
                    "land",
                    "hover",
                ]:
                    return "mode", parts[0].lower(), 0.0
                elif len(parts) >= 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    heading = float(parts[3]) if len(parts) >= 4 else 0.0
                    return "coords", (x, y, max(0.1, z)), heading
        except Exception:
            pass
    return None, None, 0.0


def compute_trajectory(sim_time: float) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float], float]:
    """
    Automated trajectory matching research paper:
      - 0.0s -> 3.0s: Precision Takeoff & Altitude Hold at (0, 0, 1.0m)
      - 3.0s -> 21.0s: 3D Helical Spiral (r=0.8m, T_xy=6.25s, T_z=12.5s)
      - 21.0s -> 40.0s: 3D Figure-8 / Lemniscate (T=9.0s, x=1.0m, y=1.0m, z=1.2m)
    """
    cycle_t = sim_time % 45.0
    start_z = 0.08
    target_z = 1.0

    if cycle_t < 4.0:
        # Smooth S-curve (cubic) takeoff from helipad (z=0.08m) to 1.0m
        tau = cycle_t / 4.0
        s = 3.0 * (tau ** 2) - 2.0 * (tau ** 3)
        s_dot = (6.0 * tau - 6.0 * (tau ** 2)) / 4.0
        s_ddot = (6.0 - 12.0 * tau) / (4.0 ** 2)

        pz = start_z + (target_z - start_z) * s
        vz = (target_z - start_z) * s_dot
        az = (target_z - start_z) * s_ddot

        ref_pos = (0.0, 0.0, pz)
        ref_vel = (0.0, 0.0, vz)
        ref_acc = (0.0, 0.0, az)
        heading = 0.0
    elif cycle_t < 24.0:
        # Gentle Helical Spiral (radius=0.6m, period=9.0s -> max horizontal speed ~0.42 m/s)
        t_h = cycle_t - 4.0
        w_xy = 2.0 * math.pi / 9.0
        w_z = 2.0 * math.pi / 15.0
        r = 0.6

        px = r * math.sin(w_xy * t_h)
        py = r * math.cos(w_xy * t_h)
        pz = 1.0 + 0.3 * math.sin(w_z * t_h)

        vx = r * w_xy * math.cos(w_xy * t_h)
        vy = -r * w_xy * math.sin(w_xy * t_h)
        vz = 0.3 * w_z * math.cos(w_z * t_h)

        ax = -r * (w_xy ** 2) * math.sin(w_xy * t_h)
        ay = -r * (w_xy ** 2) * math.cos(w_xy * t_h)
        az = -0.3 * (w_z ** 2) * math.sin(w_z * t_h)

        heading = 0.25 * math.sin(w_z * t_h)
        ref_pos, ref_vel, ref_acc = (px, py, pz), (vx, vy, vz), (ax, ay, az)
    else:
        # Smooth Figure-8 Trajectory (period=12.0s -> max horizontal speed ~0.37 m/s)
        t_f = cycle_t - 24.0
        w = 2.0 * math.pi / 12.0
        amp = 0.7

        px = amp * math.sin(w * t_f)
        py = amp * math.sin(2.0 * w * t_f) * 0.7
        pz = 1.0 + 0.2 * math.cos(w * t_f)

        vx = amp * w * math.cos(w * t_f)
        vy = amp * 0.7 * 2.0 * w * math.cos(2.0 * w * t_f)
        vz = -0.2 * w * math.sin(w * t_f)

        ax = -amp * (w ** 2) * math.sin(w * t_f)
        ay = -amp * 0.7 * 4.0 * (w ** 2) * math.sin(2.0 * w * t_f)
        az = -0.2 * (w ** 2) * math.cos(w * t_f)

        heading = 0.2 * math.sin(w * t_f)
        ref_pos, ref_vel, ref_acc = (px, py, pz), (vx, vy, vz), (ax, ay, az)

    return ref_pos, ref_vel, ref_acc, heading


class ForceTorqueMixer:
    """
    Inverts the X-configuration quadrotor motor mixing matrix
    to convert desired thrust and torques into individual motor RPMs.
    """

    def __init__(self, arm_length: float, kf: float, km: float) -> None:
        self.d = arm_length / math.sqrt(2.0)
        self.kf = kf
        self.km = km
        self.c = km / kf

        self.M = np.array([
            [1.0, 1.0, 1.0, 1.0],
            [-self.d, -self.d, self.d, self.d],
            [-self.d, self.d, self.d, -self.d],
            [-self.c, self.c, -self.c, self.c],
        ], dtype=np.float64)

        self.M_inv = np.linalg.inv(self.M)

    def compute_rpms(self, thrust: float, torques: Tuple[float, float, float], max_rpm: float = 22000.0) -> np.ndarray:
        cmd = np.array([thrust, torques[0], torques[1], torques[2]], dtype=np.float64)
        forces = self.M_inv @ cmd
        forces = np.clip(forces, 0.0, None)
        rpms = np.sqrt(forces / self.kf)
        return np.clip(rpms, 0.0, max_rpm)


def build_pybullet_city_environment(client_id: int):
    """
    Builds a realistic 3D City Environment in PyBullet:
      - Central Helipad Plaza & Asphalt Avenues
      - High-Rise Skyscrapers & Commercial Towers
      - Flight Corridor Obstacle Building Blocks
      - City Streetlamps and Trees
    """
    # 1. Asphalt Avenues (North-South & East-West)
    road_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1.5, 12.0, 0.002], physicsClientId=client_id)
    road_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[1.5, 12.0, 0.002], rgbaColor=[0.18, 0.18, 0.20, 1.0], physicsClientId=client_id)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=road_col, baseVisualShapeIndex=road_vis, basePosition=[0, 0, 0.002], physicsClientId=client_id)

    road2_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[12.0, 1.5, 0.002], physicsClientId=client_id)
    road2_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[12.0, 1.5, 0.002], rgbaColor=[0.18, 0.18, 0.20, 1.0], physicsClientId=client_id)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=road2_col, baseVisualShapeIndex=road2_vis, basePosition=[0, 0, 0.002], physicsClientId=client_id)

    # 2. Central Helipad Plaza Pad
    pad_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=1.0, height=0.01, physicsClientId=client_id)
    pad_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=1.0, length=0.01, rgbaColor=[0.25, 0.25, 0.28, 1.0], physicsClientId=client_id)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=pad_col, baseVisualShapeIndex=pad_vis, basePosition=[0, 0, 0.005], physicsClientId=client_id)

    ring_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.95, height=0.012, physicsClientId=client_id)
    ring_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.95, length=0.012, rgbaColor=[0.95, 0.8, 0.1, 1.0], physicsClientId=client_id)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=ring_col, baseVisualShapeIndex=ring_vis, basePosition=[0, 0, 0.006], physicsClientId=client_id)

    # 3. Perimeter High-Rise Skyscrapers
    # Skyscraper NE: Metropolitan Blue Glass Tower (h=7m)
    tower1_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.9, 0.9, 3.5], physicsClientId=client_id)
    tower1_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.9, 0.9, 3.5], rgbaColor=[0.15, 0.45, 0.75, 0.95], physicsClientId=client_id)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=tower1_col, baseVisualShapeIndex=tower1_vis, basePosition=[3.8, 3.8, 3.5], physicsClientId=client_id)

    # Skyscraper NW: Financial Dark Glass Tower (h=6m)
    tower2_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.8, 0.8, 3.0], physicsClientId=client_id)
    tower2_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.8, 0.8, 3.0], rgbaColor=[0.12, 0.18, 0.25, 0.95], physicsClientId=client_id)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=tower2_col, baseVisualShapeIndex=tower2_vis, basePosition=[-3.8, 3.8, 3.0], physicsClientId=client_id)

    # Skyscraper SE: Tech Headquarters Brick/Concrete Tower (h=5.5m)
    tower3_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.85, 0.85, 2.75], physicsClientId=client_id)
    tower3_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.85, 0.85, 2.75], rgbaColor=[0.65, 0.35, 0.25, 1.0], physicsClientId=client_id)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=tower3_col, baseVisualShapeIndex=tower3_vis, basePosition=[3.8, -3.8, 2.75], physicsClientId=client_id)

    # Skyscraper SW: Civic Center Granite Tower (h=5m)
    tower4_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.75, 0.75, 2.5], physicsClientId=client_id)
    tower4_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.75, 0.75, 2.5], rgbaColor=[0.65, 0.66, 0.68, 1.0], physicsClientId=client_id)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=tower4_col, baseVisualShapeIndex=tower4_vis, basePosition=[-3.8, -3.8, 2.5], physicsClientId=client_id)

    # 4. Urban Corridor Building Blocks (directly challenging flight paths)
    # Corridor Block A (North-East Flank): x=0.9, y=0.9, h=2.0m
    bldg_a_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 1.0], physicsClientId=client_id)
    bldg_a_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 1.0], rgbaColor=[0.75, 0.35, 0.2, 1.0], physicsClientId=client_id)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=bldg_a_col, baseVisualShapeIndex=bldg_a_vis, basePosition=[0.9, 0.9, 1.0], physicsClientId=client_id)

    # Corridor Block B (West Flank): x=-1.0, y=0.4, h=1.8m
    bldg_b_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.22, 0.28, 0.9], physicsClientId=client_id)
    bldg_b_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.22, 0.28, 0.9], rgbaColor=[0.2, 0.5, 0.6, 1.0], physicsClientId=client_id)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=bldg_b_col, baseVisualShapeIndex=bldg_b_vis, basePosition=[-1.0, 0.4, 0.9], physicsClientId=client_id)

    # Corridor Block C (South Flank): x=0.4, y=-1.0, h=1.6m
    bldg_c_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.28, 0.22, 0.8], physicsClientId=client_id)
    bldg_c_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.28, 0.22, 0.8], rgbaColor=[0.6, 0.6, 0.65, 1.0], physicsClientId=client_id)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=bldg_c_col, baseVisualShapeIndex=bldg_c_vis, basePosition=[0.4, -1.0, 0.8], physicsClientId=client_id)

    # 5. City Trees along Avenues
    tree_locs = [(1.8, 1.8), (-1.8, 1.8), (-1.8, -1.8), (1.8, -1.8)]
    for tx, ty in tree_locs:
        t_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.08, height=0.6, physicsClientId=client_id)
        t_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.08, length=0.6, rgbaColor=[0.35, 0.22, 0.12, 1.0], physicsClientId=client_id)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=t_col, baseVisualShapeIndex=t_vis, basePosition=[tx, ty, 0.3], physicsClientId=client_id)

        c_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.35, physicsClientId=client_id)
        c_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.35, rgbaColor=[0.18, 0.55, 0.18, 0.95], physicsClientId=client_id)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=c_col, baseVisualShapeIndex=c_vis, basePosition=[tx, ty, 0.85], physicsClientId=client_id)


def perform_pybullet_lidar_scan(
    client_id: int,
    drone_pos: Tuple[float, float, float],
    heading: float,
    max_range: float = 6.0,
    drone_id: int = 0,
    draw_debug_lines: bool = False,
    debug_line_ids: Optional[List[int]] = None,
) -> Tuple[LidarScan, List[int]]:
    """
    Fires 10 LiDAR beams (8 horizontal + 2 vertical) in PyBullet.
    Returns LidarScan dataclass and updated debug line IDs.
    """
    ray_starts = []
    ray_ends = []

    # 8 Horizontal Ray Directions in World Frame
    for i in range(8):
        ang = heading + i * (math.pi / 4.0)
        dx = math.cos(ang)
        dy = math.sin(ang)
        # Start ray slightly outside drone hull
        start = [drone_pos[0] + dx * 0.08, drone_pos[1] + dy * 0.08, drone_pos[2]]
        end = [drone_pos[0] + dx * max_range, drone_pos[1] + dy * max_range, drone_pos[2]]
        ray_starts.append(start)
        ray_ends.append(end)

    # Ray 8: Upward (+Z)
    ray_starts.append([drone_pos[0], drone_pos[1], drone_pos[2] + 0.08])
    ray_ends.append([drone_pos[0], drone_pos[1], drone_pos[2] + max_range])

    # Ray 9: Downward (-Z)
    ray_starts.append([drone_pos[0], drone_pos[1], drone_pos[2] - 0.08])
    ray_ends.append([drone_pos[0], drone_pos[1], drone_pos[2] - max_range])

    # Batch Raycast Query
    results = p.rayTestBatch(ray_starts, ray_ends, physicsClientId=client_id)

    ranges = []
    updated_line_ids = [] if debug_line_ids is None else debug_line_ids

    for i in range(8):
        hit_obj, hit_link, hit_frac, hit_pos, hit_norm = results[i]
        if hit_obj != -1 and hit_obj != drone_id:
            dist = 0.08 + hit_frac * (max_range - 0.08)
            ranges.append(float(dist))
            line_end = list(hit_pos)
            color = [1.0, 0.15, 0.1] if dist < 2.0 else [0.2, 0.8, 0.9]
        else:
            ranges.append(max_range)
            line_end = ray_ends[i]
            color = [0.1, 0.85, 0.2]

        if draw_debug_lines:
            prev_id = updated_line_ids[i] if i < len(updated_line_ids) else -1
            line_id = p.addUserDebugLine(
                lineFromXYZ=ray_starts[i],
                lineToXYZ=line_end,
                lineColorRGB=color,
                lineWidth=1.5,
                replaceItemUniqueId=prev_id,
                physicsClientId=client_id,
            )
            if i < len(updated_line_ids):
                updated_line_ids[i] = line_id
            else:
                updated_line_ids.append(line_id)

    # Upward distance
    hit_u, _, frac_u, _, _ = results[8]
    dist_up = float(0.08 + frac_u * (max_range - 0.08)) if (hit_u != -1 and hit_u != drone_id) else max_range

    # Downward distance
    hit_d, _, frac_d, _, _ = results[9]
    dist_down = float(0.08 + frac_d * (max_range - 0.08)) if (hit_d != -1 and hit_d != drone_id) else max_range

    scan = LidarScan(ranges=ranges, dist_up=dist_up, dist_down=dist_down, max_range=max_range)
    return scan, updated_line_ids


def main():
    parser = argparse.ArgumentParser(description="gym-pybullet-drones City Navigation & Reactive LiDAR Avoidance")
    parser.add_argument("--headless", action="store_true", help="Run headless without PyBullet 3D GUI")
    parser.add_argument("--duration", type=float, default=0, help="Run duration in seconds (0 for infinite)")
    parser.add_argument("--noise", dest="noise", action="store_true", default=True, help="Enable sensor measurement noise (default: True)")
    parser.add_argument("--no-noise", dest="noise", action="store_false", help="Disable sensor noise")
    parser.add_argument("--filter", dest="use_filter", action="store_true", default=True, help="Enable Kalman Filter state estimation (default: True)")
    parser.add_argument("--no-filter", dest="use_filter", action="store_false", help="Disable Kalman Filter")
    parser.add_argument("--noise-pos-std", type=float, default=0.05, help="Position noise std in meters (default: 0.05m)")
    parser.add_argument("--noise-vel-std", type=float, default=0.08, help="Velocity noise std in m/s (default: 0.08m/s)")
    parser.add_argument("--noise-rate-std", type=float, default=0.02, help="Angular rate noise std in rad/s (default: 0.02 rad/s)")
    args = parser.parse_args()

    target_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "target.txt")
    use_gui = not args.headless

    print("=" * 75)
    print("   GYM-PYBULLET-DRONES URBAN CITY WORLD -- REACTIVE LIDAR AVOIDANCE")
    print("=" * 75)

    init_xyz = np.array([[0.0, 0.0, 0.08]])
    init_rpy = np.array([[0.0, 0.0, 0.0]])

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=init_xyz,
        initial_rpys=init_rpy,
        physics=Physics.PYB,
        pyb_freq=250,
        ctrl_freq=250,
        gui=use_gui,
        user_debug_gui=False,
    )

    client_id = env.CLIENT
    dt = 1.0 / env.CTRL_FREQ
    mass = float(env.M)
    gravity = float(env.G)

    # Reset environment first so resetSimulation does not wipe city buildings
    obs, info = env.reset()
    sim_time = 0.0
    step = 0

    # Build Urban City Environment in PyBullet
    print("[INFO] Building 3D Urban City Architecture (Skyscrapers, Roads, Building Blocks)...")
    build_pybullet_city_environment(client_id)

    # Controller & Mixer
    controller = BacksteppingController(mass=mass, gravity=gravity)
    controller.k_position = 1.6
    controller.k_velocity = 1.2
    controller.k_tilt = 2.5
    controller.k_yaw = 1.8
    controller.k_rate = 0.5
    mixer = ForceTorqueMixer(arm_length=float(env.L), kf=float(env.KF), km=float(env.KM))

    # Sensor Noise & Kalman Filter
    noise_model = SensorNoiseModel(
        pos_std=args.noise_pos_std,
        vel_std=args.noise_vel_std,
        rate_std=args.noise_rate_std,
        enabled=args.noise,
    )
    estimator = QuadrotorStateEstimator(
        dt=dt,
        process_noise_acc=0.5,
        meas_pos_std=args.noise_pos_std,
        meas_vel_std=args.noise_vel_std,
        initial_pos=(0.0, 0.0, 0.08),
        initial_vel=(0.0, 0.0, 0.0),
    )

    # Reactive Obstacle Avoidance Engine (tuned for smooth detours around urban buildings)
    avoider = ReactiveObstacleAvoider(
        safety_distance=0.55,
        detection_distance=1.6,
        max_repulsive_speed=1.0,
    )

    noise_status = f"ENABLED (std_pos={args.noise_pos_std}m, std_vel={args.noise_vel_std}m/s)" if args.noise else "DISABLED"
    filter_status = "KALMAN FILTER ACTIVE" if args.use_filter else "RAW NOISY FEEDBACK (No Filter)"

    print(f"[INFO] Engine: PyBullet + gym-pybullet-drones (CtrlAviary)")
    print(f"[INFO] Drone Model: Crazyflie 2.x (mass={mass:.3f} kg, L={env.L:.3f} m)")
    print(f"[INFO] Sensor Noise: {noise_status}")
    print(f"[INFO] State Estimation: {filter_status}")
    print(f"[INFO] LiDAR Sensor: 10-beam Omnidirectional Real-Time Raycasting Active")
    print(f"[INFO] Obstacle Avoidance: Reactive Potential Field + Tangential Detour Active")
    print(f"[INFO] Physics Rate: {env.CTRL_FREQ} Hz (dt={dt:.4f}s)")
    print("[INFO] Trajectory: Automated Takeoff -> Helical Spiral -> Figure-8")
    print("[INFO] Interactive Waypoints: python set_target.py X Y Z\n")

    # Add Target Visual Marker
    target_visual_id = -1
    debug_text_id = -1
    debug_line_ids: List[int] = []

    if use_gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=3.2,
            cameraYaw=45.0,
            cameraPitch=-28.0,
            cameraTargetPosition=[0.0, 0.0, 1.2],
            physicsClientId=client_id,
        )

        sphere_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.04, physicsClientId=client_id)
        sphere_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.04, rgbaColor=[1.0, 0.2, 0.3, 0.8], physicsClientId=client_id)
        target_visual_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=sphere_col,
            baseVisualShapeIndex=sphere_vis,
            basePosition=[0.0, 0.0, 1.0],
            physicsClientId=client_id,
        )

    drone_body_id = env.DRONE_IDS[0] if hasattr(env, 'DRONE_IDS') else 0

    try:
        while True:
            if not p.isConnected(physicsClientId=client_id):
                print("\n[INFO] PyBullet GUI window closed by user.")
                break

            t_start = time.time()

            # 1. Target setpoint from file or trajectory
            cmd_type, data, heading_val = read_target_command(target_path)
            custom_target = None
            custom_heading = 0.0

            if cmd_type == "coords":
                custom_target = data
                custom_heading = math.radians(heading_val)
            elif cmd_type == "mode" and data in ["land"]:
                custom_target = (0.0, 0.0, 0.08)
                custom_heading = 0.0

            if custom_target is not None:
                ref_pos = custom_target
                ref_vel = (0.0, 0.0, 0.0)
                ref_acc = (0.0, 0.0, 0.0)
                heading = custom_heading
            else:
                ref_pos, ref_vel, ref_acc, heading = compute_trajectory(sim_time)

            if use_gui and target_visual_id != -1 and p.isConnected(physicsClientId=client_id):
                try:
                    p.resetBasePositionAndOrientation(
                        target_visual_id,
                        [ref_pos[0], ref_pos[1], ref_pos[2]],
                        [0.0, 0.0, 0.0, 1.0],
                        physicsClientId=client_id,
                    )
                except p.error:
                    pass

            # 2. Extract ground-truth drone state
            true_pos = tuple(float(x) for x in obs[0, 0:3])
            q_xyzw = obs[0, 3:7]
            true_quat = (float(q_xyzw[3]), float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2]))
            true_vel = tuple(float(x) for x in obs[0, 10:13])
            true_ang_vel = tuple(float(x) for x in obs[0, 13:16])

            true_state = ControllerState(
                position=true_pos,
                velocity=true_vel,
                quaternion=true_quat,
                angular_rate=true_ang_vel,
            )

            # 3. Add Sensor Noise
            noisy_state = noise_model.apply_noise(true_state)

            # 4. Kalman Filter Estimation
            if args.use_filter:
                feedback_state = estimator.estimate(noisy_state, dt=dt)
            else:
                feedback_state = noisy_state

            noise_err = math.sqrt(sum((noisy_state.position[i] - true_pos[i]) ** 2 for i in range(3)))
            kf_err = math.sqrt(sum((feedback_state.position[i] - true_pos[i]) ** 2 for i in range(3)))

            # 5. Omnidirectional LiDAR Scan (no green lines drawn across arena)
            scan, debug_line_ids = perform_pybullet_lidar_scan(
                client_id=client_id,
                drone_pos=true_pos,
                heading=heading,
                max_range=6.0,
                drone_id=drone_body_id,
                draw_debug_lines=False,
                debug_line_ids=debug_line_ids,
            )

            # 6. Reactive Obstacle Avoidance Replanning
            (safe_vx, safe_vy, safe_vz), is_avoiding, avoid_status = avoider.compute_avoidance_velocity(
                lidar=scan,
                current_vel=feedback_state.velocity,
                desired_vel=ref_vel,
                heading=heading,
            )

            if is_avoiding:
                avoid_pos = (
                    feedback_state.position[0] + safe_vx * dt,
                    feedback_state.position[1] + safe_vy * dt,
                    max(0.15, feedback_state.position[2] + safe_vz * dt),
                )
                avoid_ref = ControllerReference(avoid_pos, (safe_vx, safe_vy, safe_vz), (0.0, 0.0, 0.0), heading)
                thrust, torques = controller.compute_command(feedback_state, avoid_ref, dt=dt)
            else:
                ref = ControllerReference(ref_pos, ref_vel, ref_acc, heading)
                thrust, torques = controller.compute_command(feedback_state, ref, dt=dt)

            # 7. Motor Mixing & Execution
            rpms = mixer.compute_rpms(thrust, torques, max_rpm=float(env.MAX_RPM))
            action = rpms.reshape(1, 4)
            obs, reward, terminated, truncated, info = env.step(action)

            # 8. GUI Text Overlay & Telemetry (side of screen, identical format to MuJoCo)
            if use_gui and step % 25 == 0 and p.isConnected(physicsClientId=client_id):
                min_dist = scan.min_horizontal_dist
                avoid_lbl = f"Avoid ({avoid_status})" if is_avoiding else f"Clear ({min_dist:.1f}m)"
                filter_lbl = "KF Active" if args.use_filter else "No Filter"
                overlay_text = (
                    f"POS [Act: ({true_pos[0]:.2f}, {true_pos[1]:.2f}, {true_pos[2]:.2f}) | Des: ({ref_pos[0]:.2f}, {ref_pos[1]:.2f}, {ref_pos[2]:.2f})]\n"
                    f"STATE [{filter_lbl} | KF Err: {kf_err*100:.1f}cm | Noise: {noise_err*100:.1f}cm]\n"
                    f"LIDAR [{avoid_lbl} | Vel: ({true_vel[0]:.2f}, {true_vel[1]:.2f}, {true_vel[2]:.2f}m/s)]"
                )
                side_pos = [-3.8, 1.2, 3.8]
                try:
                    if debug_text_id != -1:
                        debug_text_id = p.addUserDebugText(
                            overlay_text,
                            textPosition=side_pos,
                            textColorRGB=[0.15, 0.95, 0.35],
                            textSize=0.9,
                            replaceItemUniqueId=debug_text_id,
                            physicsClientId=client_id,
                        )
                    else:
                        debug_text_id = p.addUserDebugText(
                            overlay_text,
                            textPosition=side_pos,
                            textColorRGB=[0.15, 0.95, 0.35],
                            textSize=0.9,
                            physicsClientId=client_id,
                        )
                except p.error:
                    pass

            if step % 125 == 0:  # ~0.5s at 250Hz
                mode_str = "CUSTOM TARGET" if custom_target else "AUTO TRAJECTORY"
                min_lidar = scan.min_horizontal_dist
                status_suffix = f" | [LIDAR {avoid_status}]" if is_avoiding else f" | LiDAR Clear ({min_lidar:.1f}m)"
                filter_str = f" | KF Err: {kf_err*100:4.1f}cm (Noise: {noise_err*100:4.1f}cm)" if (args.noise and args.use_filter) else ""
                print(f"[{mode_str} t={sim_time:5.1f}s] Pos: ({true_pos[0]:5.2f}, {true_pos[1]:5.2f}, {true_pos[2]:5.2f}m) | Target: ({ref_pos[0]:5.2f}, {ref_pos[1]:5.2f}, {ref_pos[2]:5.2f}m){filter_str}{status_suffix}")

            step += 1
            sim_time += dt

            if args.duration > 0 and sim_time >= args.duration:
                break

            if use_gui:
                elapsed = time.time() - t_start
                sleep_rem = dt - elapsed
                if sleep_rem > 0.001:
                    time.sleep(sleep_rem)

    except KeyboardInterrupt:
        print("\n[INFO] Simulation stopped by user.")
    finally:
        try:
            if p.isConnected(physicsClientId=client_id):
                env.close()
        except Exception:
            pass
        print("[INFO] gym-pybullet-drones simulation closed.")


if __name__ == "__main__":
    main()
