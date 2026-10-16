"""
gym-pybullet-drones 3D Quadrotor Simulation - full De Monte & Lohmann (2013) backstepping
-----------------------------------------------------------------------------------------
* Crazyflie (pybullet.drone_model) in PyBullet; physics/control rates from config.
* The paper controller outputs thrust T and torques tau; ForceTorqueMixer inverts the
  CF2X allocation matrix to motor RPMs.
* Noise + Kalman filter, LiDAR (rayTestBatch) + avoidance, same guidance as MuJoCo.
* The city is built from the MuJoCo model (paths.mujoco_model) - one world for both.
All parameters: config.yaml (sections pybullet, controller, noise, estimator, guidance,
avoidance, logging).

Usage:
    python run_pybullet_simulation.py
    python run_pybullet_simulation.py --headless --duration 60 --log pybullet_log.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from typing import List, Optional, Tuple

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller.config import banner, add_config_arguments, apply_config_arguments, cfg, load_config, project_path  # noqa: E402

load_config(next((sys.argv[i + 1] for i, a in enumerate(sys.argv[:-1]) if a == "--config"), None))
try:                                           # use the pip-installed package if present ...
    import gym_pybullet_drones  # noqa: F401
except ImportError:                            # ... otherwise the copy in this project
    sys.path.insert(0, project_path("gym_pybullet_drones"))
import pybullet as p  # noqa: E402
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary  # noqa: E402
from gym_pybullet_drones.utils.enums import DroneModel, Physics  # noqa: E402

from quadrotor_controller import (  # noqa: E402
    GridPlanner,
    WorldMap,
    ControllerState,
    LidarScan,
    PaperBacksteppingController,
    QuadrotorStateEstimator,
    ReactiveObstacleAvoider,
    ReferenceManager,
    SensorNoiseModel,
    heading_from_quaternion,
    quat_to_rot,
)
from quadrotor_controller.units import (  # noqa: E402
    CM_PER_M, PB_ANG_VEL_WORLD, PB_CAM_DIST, PB_CAM_FORWARD, PB_CAM_HORIZONTAL, PB_CAM_PITCH, PB_CAM_PROJECTION,
    PB_CAM_TARGET, PB_CAM_VERTICAL, PB_CAM_YAW, PB_POS, PB_QUAT_XYZW, PB_VEL, PROJ_XX, PROJ_YY)
from quadrotor_controller.world_geometry import load_static_geoms  # noqa: E402


class ForceTorqueMixer:
    """Inverts the X-configuration allocation matrix: [T, tau_x, tau_y, tau_z] -> motor RPMs.
    Motor order/signs follow gym-pybullet-drones CF2X (prop i at (+-d, +-d))."""

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

    def compute_rpms(self, thrust: float, torques: Tuple[float, float, float], max_rpm: float) -> np.ndarray:
        forces = np.clip(self.M_inv @ np.array([thrust, *torques], dtype=np.float64), 0.0, None)
        return np.clip(np.sqrt(forces / self.kf), 0.0, max_rpm)


def build_world_from_mujoco_model(client_id: int) -> int:
    """Rebuilds the static MuJoCo world (buildings, lamps, trees, markings) in PyBullet."""
    n = 0
    for g in load_static_geoms(skip_bodies=("quadrotor",)):
        if g.kind == "box":
            shape = dict(shapeType=p.GEOM_BOX, halfExtents=list(g.size[:3]))
            vis_shape = shape
        elif g.kind in ("cylinder", "capsule"):
            kind = p.GEOM_CYLINDER if g.kind == "cylinder" else p.GEOM_CAPSULE
            shape = dict(shapeType=kind, radius=g.size[0], height=2.0 * g.size[1])
            vis_shape = dict(shapeType=kind, radius=g.size[0], length=2.0 * g.size[1])
        else:                                   # sphere / ellipsoid (largest radius)
            shape = dict(shapeType=p.GEOM_SPHERE, radius=max(g.size[:3]) if g.kind == "ellipsoid" else g.size[0])
            vis_shape = shape
        col = p.createCollisionShape(physicsClientId=client_id, **shape) if g.collide else -1
        vis = p.createVisualShape(rgbaColor=list(g.rgba), physicsClientId=client_id, **vis_shape)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                          basePosition=list(g.pos), baseOrientation=list(g.quat_xyzw), physicsClientId=client_id)
        n += 1
    return n


class PyBulletViewer:
    """PyBullet window: 3D view without side panels, shadows, keyboard camera (follow /
    overview / top, rotate, zoom) that keeps mouse changes, flight trail, target marker and a
    HUD pinned to a corner of the window (like MuJoCo's).
    Settings: config.yaml -> pybullet.gui/camera/keys/trail/hud/marker."""

    def __init__(self, cid: int, drone_id: int, plane_id: int, start):
        self.cid, self.drone = cid, drone_id
        pc = "pybullet."
        g = cfg(pc + "gui")
        if g["remove_drone_axes"]:
            p.removeAllUserDebugItems(physicsClientId=cid)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0 if g["hide_side_panels"] else 1, physicsClientId=cid)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1 if g["shadows"] else 0, physicsClientId=cid)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0 if g["disable_mouse_picking"] else 1, physicsClientId=cid)
        for flag in (p.COV_ENABLE_RGB_BUFFER_PREVIEW, p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                     p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW):
            p.configureDebugVisualizer(flag, 0, physicsClientId=cid)
        try:
            p.configureDebugVisualizer(lightPosition=list(g["light_direction"]),
                                       shadowMapResolution=g["shadow_map_resolution"],
                                       shadowMapWorldSize=g["shadow_map_world_size"], physicsClientId=cid)
        except TypeError:                               # older PyBullet without these options
            p.configureDebugVisualizer(lightPosition=list(g["light_direction"]), physicsClientId=cid)
        ground = cfg("world.materials")[cfg(pc + "ground_material")]["rgba"]
        p.changeVisualShape(plane_id, -1, rgbaColor=ground, textureUniqueId=-1, physicsClientId=cid)

        self.cam = cfg(pc + "camera")
        self.mode = self.cam["start_mode"]
        self.keys = {name: self._keycode(k) for name, k in cfg(pc + "keys").items()}
        self._apply_preset(self.mode, start)
        mk = cfg(pc + "marker")
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=mk["radius"], rgbaColor=mk["rgba"], physicsClientId=cid)
        self.marker = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vis,
                                        basePosition=list(start), physicsClientId=cid)
        self.tr = cfg(pc + "trail")
        self.trail_on = self.tr["enabled"]
        self.trail_ids = [-1] * self.tr["segments"]
        self.trail_k, self.trail_last, self.trail_t = 0, None, -math.inf
        self.hud = cfg(pc + "hud")
        self.text_ids = []
        self.cam_t = self.hud_t = -math.inf

    @staticmethod
    def _keycode(k: str) -> int:
        special = {"LEFT": p.B3G_LEFT_ARROW, "RIGHT": p.B3G_RIGHT_ARROW, "UP": p.B3G_UP_ARROW, "DOWN": p.B3G_DOWN_ARROW}
        return special.get(str(k).upper(), ord(str(k)[0]))

    def _apply_preset(self, mode, drone_pos):
        c = self.cam[mode]
        target = drone_pos if mode == "follow" else c["target"]
        p.resetDebugVisualizerCamera(cameraDistance=c["distance"], cameraYaw=c["yaw"], cameraPitch=c["pitch"],
                                     cameraTargetPosition=list(target), physicsClientId=self.cid)

    @staticmethod
    def controls_help() -> str:
        k = cfg("pybullet.keys")
        return (f"[VIEW] keys: {k['follow']}=follow drone  {k['overview']}=overview  {k['top']}=top  "
                f"{k['trail']}=trail on/off  arrows=rotate  {k['zoom_in']}/{k['zoom_out']}=zoom | "
                "mouse: Ctrl+left-drag rotate, Ctrl+middle-drag pan, wheel zoom")

    def update(self, t: float, drone_pos, ref_pos, text: str) -> None:
        cid = self.cid
        self.hud_lines = text if isinstance(text, (list, tuple)) else [text]
        if t - self.cam_t >= self.cam["update_period"]:
            self.cam_t = t
            self._camera(drone_pos)
            self._hud(self.hud_lines)          # the HUD is placed relative to the camera: move it together
            self.hud_t = t
        p.resetBasePositionAndOrientation(self.marker, list(ref_pos), [0, 0, 0, 1], physicsClientId=cid)
        if self.trail_on and t - self.trail_t >= self.tr["period"]:
            self.trail_t = t
            if self.trail_last is not None:
                self.trail_ids[self.trail_k] = p.addUserDebugLine(
                    self.trail_last, list(drone_pos), lineColorRGB=self.tr["rgb"], lineWidth=self.tr["width"],
                    replaceItemUniqueId=self.trail_ids[self.trail_k], physicsClientId=cid)
                self.trail_k = (self.trail_k + 1) % len(self.trail_ids)
            self.trail_last = list(drone_pos)
        if t - self.hud_t >= self.hud["period"]:
            self.hud_t = t
            self._hud(self.hud_lines)

    def _screen_point(self, info, u: float, v: float):
        """World point that appears at window position (u, v) (0..1 from the top-left corner),
        hud.depth in front of the camera - text placed there stays fixed on the screen."""
        fwd = np.asarray(info[PB_CAM_FORWARD], dtype=float)
        fwd /= np.linalg.norm(fwd)
        right = np.asarray(info[PB_CAM_HORIZONTAL], dtype=float)
        up = np.asarray(info[PB_CAM_VERTICAL], dtype=float)
        right /= np.linalg.norm(right)
        up /= np.linalg.norm(up)
        proj = info[PB_CAM_PROJECTION]
        eye = np.asarray(info[PB_CAM_TARGET], dtype=float) - fwd * info[PB_CAM_DIST]
        d = self.hud["depth"]
        half_w, half_h = d / proj[PROJ_XX], d / proj[PROJ_YY]
        return eye + fwd * d + right * (2.0 * u - 1.0) * half_w + up * (1.0 - 2.0 * v) * half_h

    def _hud(self, lines) -> None:
        cid = self.cid
        info = p.getDebugVisualizerCamera(physicsClientId=cid)
        h = self.hud
        u, v = h["anchor"]
        for k, line in enumerate(lines):
            pos = self._screen_point(info, u, v + k * h["line_spacing"]).tolist()
            kw = dict(textPosition=pos, textColorRGB=h["rgb"], textSize=h["size"], physicsClientId=cid)
            if k < len(self.text_ids):
                kw["replaceItemUniqueId"] = self.text_ids[k]
            try:
                item = p.addUserDebugText(line, **kw)
            except p.error:
                continue
            if k < len(self.text_ids):
                self.text_ids[k] = item
            else:
                self.text_ids.append(item)

    def _camera(self, drone_pos):
        cid = self.cid
        info = p.getDebugVisualizerCamera(physicsClientId=cid)
        yaw, pitch, dist, target = info[PB_CAM_YAW], info[PB_CAM_PITCH], info[PB_CAM_DIST], list(info[PB_CAM_TARGET])  # keep mouse changes
        c = self.cam
        events = p.getKeyboardEvents(physicsClientId=cid)
        for code, state in events.items():
            down = state & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN)
            if not down:
                continue
            if code == self.keys["follow"] and state & p.KEY_WAS_TRIGGERED:
                self.mode = "follow"
                self._apply_preset("follow", drone_pos)
                return
            if code == self.keys["overview"] and state & p.KEY_WAS_TRIGGERED:
                self.mode = "overview"
                self._apply_preset("overview", drone_pos)
                return
            if code == self.keys["top"] and state & p.KEY_WAS_TRIGGERED:
                self.mode = "top"
                self._apply_preset("top", drone_pos)
                return
            if code == self.keys["trail"] and state & p.KEY_WAS_TRIGGERED:
                self.trail_on = not self.trail_on
                if not self.trail_on:
                    for i in self.trail_ids:
                        if i != -1:
                            p.removeUserDebugItem(i, physicsClientId=cid)
                    self.trail_ids = [-1] * len(self.trail_ids)
                    self.trail_last = None
            if code == self.keys["rotate_left"]:
                yaw -= c["rotate_step_deg"]
            elif code == self.keys["rotate_right"]:
                yaw += c["rotate_step_deg"]
            elif code == self.keys["pitch_up"]:
                pitch = min(pitch + c["rotate_step_deg"], c["pitch_limits"][1])
            elif code == self.keys["pitch_down"]:
                pitch = max(pitch - c["rotate_step_deg"], c["pitch_limits"][0])
            elif code == self.keys["zoom_in"]:
                dist = max(dist / c["zoom_factor"], c["min_distance"])
            elif code == self.keys["zoom_out"]:
                dist = min(dist * c["zoom_factor"], c["max_distance"])
        if self.mode == "follow":
            target = list(drone_pos)
        p.resetDebugVisualizerCamera(cameraDistance=dist, cameraYaw=yaw, cameraPitch=pitch,
                                     cameraTargetPosition=target, physicsClientId=cid)


def perform_pybullet_lidar_scan(client_id: int, drone_pos, heading: float, max_range: float, drone_id: int,
                                ray_start: float, beams: int) -> LidarScan:
    """Horizontal rays (heading frame, evenly spaced) + up + down with rayTestBatch."""
    starts, ends = [], []
    step = 2.0 * math.pi / beams
    for i in range(beams):
        dx, dy = math.cos(heading + i * step), math.sin(heading + i * step)
        starts.append([drone_pos[0] + dx * ray_start, drone_pos[1] + dy * ray_start, drone_pos[2]])
        ends.append([drone_pos[0] + dx * max_range, drone_pos[1] + dy * max_range, drone_pos[2]])
    for sign in (1.0, -1.0):
        starts.append([drone_pos[0], drone_pos[1], drone_pos[2] + sign * ray_start])
        ends.append([drone_pos[0], drone_pos[1], drone_pos[2] + sign * max_range])
    res = p.rayTestBatch(starts, ends, physicsClientId=client_id)

    def dist(r):
        hit, _, frac, _, _ = r
        return ray_start + frac * (max_range - ray_start) if hit not in (-1, drone_id) else max_range

    d = [dist(r) for r in res]
    return LidarScan(ranges=d[:beams], dist_up=d[beams], dist_down=d[beams + 1], max_range=max_range)


def main():
    ap = argparse.ArgumentParser(description="gym-pybullet-drones - full paper backstepping controller")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--duration", type=float, default=0.0)
    ap.add_argument("--noise", dest="noise", action="store_true", default=None)
    ap.add_argument("--no-noise", dest="noise", action="store_false")
    ap.add_argument("--filter", dest="use_filter", action="store_true", default=None)
    ap.add_argument("--no-filter", dest="use_filter", action="store_false")
    ap.add_argument("--no-avoid", action="store_true")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--ignore-target-file", action="store_true")
    ap.add_argument("--log", type=str, default=None)
    add_config_arguments(ap)
    args = ap.parse_args()
    apply_config_arguments(args)
    use_filter = cfg("estimator.enabled") if args.use_filter is None else args.use_filter
    pc = "pybullet."

    use_gui = not args.headless
    if use_gui:   # gym-pybullet-drones opens the window itself -> pass the configured window size
        _connect = p.connect
        w_, h_ = cfg(pc + "gui.window_size")
        p.connect = lambda mode, *a, **k: (_connect(mode, *a, options=f"--width={w_} --height={h_}", **k)
                                           if mode == p.GUI and "options" not in k else _connect(mode, *a, **k))
    env = CtrlAviary(drone_model=DroneModel[cfg(pc + "drone_model")], num_drones=1,
                     initial_xyzs=np.array([cfg(pc + "start")]), initial_rpys=np.zeros((1, 3)),
                     physics=Physics[cfg(pc + "physics")], pyb_freq=cfg(pc + "physics_hz"),
                     ctrl_freq=cfg(pc + "control_hz"), gui=use_gui, user_debug_gui=False)
    cid = env.CLIENT
    dt = 1.0 / env.CTRL_FREQ
    obs, _ = env.reset()
    drone_id = env.DRONE_IDS[0]
    if use_gui:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=cid)   # fast world building
    n_geoms = build_world_from_mujoco_model(cid) if cfg(pc + "world_from_mujoco_model") else 0
    if cfg(pc + "disable_default_damping"):
        # PyBullet's default numerical velocity damping is not physical drag (f(v) = 0 in the model)
        for link in range(-1, p.getNumJoints(drone_id, physicsClientId=cid)):
            p.changeDynamics(drone_id, link, linearDamping=0.0, angularDamping=0.0, physicsClientId=cid)

    mass, g = float(env.M), float(env.G)
    inertia = (float(env.J[0, 0]), float(env.J[1, 1]), float(env.J[2, 2]))
    t_max = 4.0 * float(env.KF) * float(env.MAX_RPM) ** 2
    ctl = PaperBacksteppingController(mass=mass, inertia=inertia, gravity=g,
                                      thrust_limits=(cfg(pc + "thrust_min_ratio") * mass * g,
                                                     cfg(pc + "thrust_max_fraction") * t_max))
    mixer = ForceTorqueMixer(arm_length=float(env.L), kf=float(env.KF), km=float(env.KM))
    noise = SensorNoiseModel(enabled=args.noise, seed=args.seed)
    start = tuple(float(x) for x in obs[0, PB_POS])
    est = QuadrotorStateEstimator(dt=dt, initial_pos=start)
    avoider = ReactiveObstacleAvoider(**cfg(pc + "avoidance"))
    refman = ReferenceManager(start_pos=start,
                              target_path=None if args.ignore_target_file else project_path("target_file"),
                              avoider=None if args.no_avoid else avoider,
                              avoid_gain=cfg(pc + "avoid_gain"), setpoint_speed=cfg(pc + "setpoint_speed"),
                              planner=None if args.no_avoid or not cfg("planner.enabled") else GridPlanner(WorldMap.from_city()))
    lidar_range, ray_start, beams = cfg(pc + "lidar_range"), cfg(pc + "lidar_ray_start"), cfg("avoidance.lidar_beams")
    print_every = max(1, int(round(cfg("logging.print_period") / dt)))
    log_every = max(1, int(round(cfg("logging.csv_period") / dt)))

    print(banner("GYM-PYBULLET-DRONES - DE MONTE & LOHMANN (2013) FULL BACKSTEPPING"))
    print(f"[INFO] {cfg(pc + 'drone_model')} mass {mass:.3f} kg | J {inertia} | T_max {t_max:.3f} N | "
          f"control {env.CTRL_FREQ} Hz | world: {n_geoms} geoms from {cfg('paths.mujoco_model')}")
    print(f"[INFO] Noise: {'ON' if noise.enabled else 'OFF'} | Kalman filter: {'ON' if use_filter else 'OFF'} "
          f"| Avoidance: {'OFF' if args.no_avoid else 'ON'}")
    print("[INFO] Commands: python set_target.py X Y Z | auto | spiral | figure8 | hover | land\n")

    viewer = PyBulletViewer(cid, drone_id, env.PLANE_ID, start) if use_gui else None
    if use_gui:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=cid)
    if viewer:
        print(PyBulletViewer.controls_help() + "\n")

    log_f = log_w = None
    if args.log:
        log_f = open(args.log, "w", newline="")
        log_w = csv.writer(log_f)
        log_w.writerow(["t", "phase", "ref_x", "ref_y", "ref_z", "ref_yaw", "x", "y", "z", "yaw",
                        "est_x", "est_y", "est_z", "meas_x", "meas_y", "meas_z", "thrust",
                        "tau_x", "tau_y", "tau_z", "err", "lyapunov", "avoiding"])

    t, step = 0.0, 0
    min_sleep = cfg("logging.realtime_min_sleep")
    try:
        while args.duration <= 0 or t < args.duration:
            if not p.isConnected(physicsClientId=cid):
                break
            t0 = time.time()
            pos = tuple(float(x) for x in obs[0, PB_POS])
            qx, qy, qz, qw = (float(x) for x in obs[0, PB_QUAT_XYZW])
            quat = (qw, qx, qy, qz)
            w_body = quat_to_rot(quat).T @ obs[0, PB_ANG_VEL_WORLD]   # PyBullet gives the angular velocity in the world frame
            true = ControllerState(position=pos, velocity=tuple(float(x) for x in obs[0, PB_VEL]),
                                   quaternion=quat, angular_rate=tuple(float(x) for x in w_body))
            yaw = heading_from_quaternion(quat)
            scan = perform_pybullet_lidar_scan(cid, pos, yaw, lidar_range, drone_id, ray_start, beams)
            ref = refman.update(t, dt, scan, yaw)

            meas = noise.apply_noise(true)
            fb = est.estimate(meas, dt=dt, acc_control=ctl.model_acceleration(meas.quaternion, meas.velocity)) \
                if use_filter else meas
            thrust, tau = ctl.compute(fb, ref, dt)
            obs, _, _, _, _ = env.step(mixer.compute_rpms(thrust, tau, max_rpm=float(env.MAX_RPM)).reshape(1, 4))

            err = math.dist(pos, ref.pos)
            if log_w is not None and step % log_every == 0:
                log_w.writerow([f"{t:.3f}", refman.phase(t), *[f"{v:.4f}" for v in ref.pos], f"{ref.yaw:.4f}",
                                *[f"{v:.4f}" for v in pos], f"{yaw:.4f}", *[f"{v:.4f}" for v in fb.position],
                                *[f"{v:.4f}" for v in meas.position], f"{thrust:.4f}",
                                *[f"{v:.7f}" for v in tau], f"{err:.4f}", f"{ctl.last['lyapunov']:.5f}",
                                int(refman.avoiding)])
            if viewer is not None:
                lid = refman.avoid_status if refman.avoiding else f"clear {scan.min_horizontal_dist:.1f}m"
                route = f" | route: {len(refman.waypoints)} waypoint(s) left" if refman.waypoints else ""
                viewer.update(t, pos, refman.final_goal if refman.mode in ("coords", "land") and refman.final_goal
                              else ref.pos, [
                    f"MODE   {refman.phase(t)}",
                    f"POS    act ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})  ref ({ref.pos[0]:.2f}, {ref.pos[1]:.2f}, "
                    f"{ref.pos[2]:.2f})  err {err * CM_PER_M:.1f} cm",
                    f"STATE  KF {'on' if use_filter else 'off'} | KF err {math.dist(fb.position, pos) * CM_PER_M:.1f} cm | "
                    f"noise {math.dist(meas.position, pos) * CM_PER_M:.1f} cm | T {thrust:.3f} N",
                    f"LIDAR  {lid}{route}"])
            if step % print_every == 0:
                lid = f"[LIDAR {refman.avoid_status}]" if refman.avoiding else f"LiDAR clear ({scan.min_horizontal_dist:.1f}m)"
                print(f"[{refman.phase(t):20s} t={t:6.1f}s] Pos ({pos[0]:5.2f},{pos[1]:5.2f},{pos[2]:5.2f}) "
                      f"Ref ({ref.pos[0]:5.2f},{ref.pos[1]:5.2f},{ref.pos[2]:5.2f}) err {err * CM_PER_M:5.1f}cm "
                      f"| T {thrust:.3f}N | KF {math.dist(fb.position, pos) * CM_PER_M:4.1f}cm "
                      f"(noise {math.dist(meas.position, pos) * CM_PER_M:4.1f}cm) | {lid}")
            t += dt
            step += 1
            if use_gui:
                rem = dt - (time.time() - t0)
                if rem > min_sleep:
                    time.sleep(rem)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        if log_f:
            log_f.close()
            print(f"[INFO] Log written to {args.log}")
        try:
            env.close()
        except Exception:
            pass
        print("[INFO] gym-pybullet-drones simulation closed.")


if __name__ == "__main__":
    main()
