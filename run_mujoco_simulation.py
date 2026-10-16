"""
MuJoCo 3D Quadrotor Simulation - full De Monte & Lohmann (2013) backstepping controller
---------------------------------------------------------------------------------------
* Real rigid-body dynamics: the controller's thrust T and body torques tau act through
  MuJoCo actuators (paths.mujoco_model); MuJoCo integrates the motion.
* Sensor noise + Kalman filter, LiDAR (mj_ray) + smooth reactive avoidance.
* Commands: python set_target.py X Y Z | auto | spiral | figure8 | hover | land
All parameters: config.yaml (sections mujoco, vehicle, controller, noise, estimator,
guidance, avoidance, logging).

Usage:
    python run_mujoco_simulation.py
    python run_mujoco_simulation.py --headless --duration 60 --log mujoco_log.csv
    python run_mujoco_simulation.py --no-noise --no-avoid --set controller.k_x=1.5
"""

import argparse
import csv
import ctypes
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller.config import banner, add_config_arguments, apply_config_arguments, cfg, load_config, project_path  # noqa: E402


def _windows_opengl():
    """Force hardware OpenGL on Windows before importing mujoco (Anaconda ships a software
    opengl32sw.dll that gives a blank viewer)."""
    if os.name != "nt":
        return
    sys32 = os.path.join(os.environ.get("SystemRoot", os.environ.get("windir", "")), "System32")
    for dll in cfg("mujoco.windows_gl_dlls"):
        try:
            ctypes.WinDLL(os.path.join(sys32, dll))
        except OSError:
            pass
    if hasattr(os, "add_dll_directory") and os.path.isdir(sys32):
        try:
            os.add_dll_directory(sys32)
        except OSError:
            pass
    os.environ.setdefault("MUJOCO_GL", cfg("mujoco.windows_gl"))


load_config(next((sys.argv[i + 1] for i, a in enumerate(sys.argv[:-1]) if a == "--config"), None))
_windows_opengl()
import mujoco  # noqa: E402
try:
    import mujoco.viewer  # noqa: E402,F401
except Exception:  # viewer not available on headless servers
    pass

from quadrotor_controller.units import CM_PER_M, MJ_ANG_VEL, MJ_LIN_VEL, MJ_POS, MJ_QUAT  # noqa: E402
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
)


def mujoco_lidar(m, d, body_id, pos, yaw, max_range, groups, beams):
    """Horizontal rays (heading frame, evenly spaced) + up + down using mj_ray."""
    geomid = np.zeros(1, dtype=np.int32)
    p = np.asarray(pos, dtype=np.float64)

    def ray(vec):
        dist = mujoco.mj_ray(m, d, p, np.asarray(vec, dtype=np.float64), groups, 1, body_id, geomid)
        return dist if 0.0 < dist < max_range else max_range

    step = 2.0 * math.pi / beams
    ranges = [ray((math.cos(yaw + i * step), math.sin(yaw + i * step), 0.0)) for i in range(beams)]
    return LidarScan(ranges=ranges, dist_up=ray((0.0, 0.0, 1.0)), dist_down=ray((0.0, 0.0, -1.0)), max_range=max_range)


def main():
    ap = argparse.ArgumentParser(description="MuJoCo quadrotor - full paper backstepping controller")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds (0 = forever)")
    ap.add_argument("--noise", dest="noise", action="store_true", default=None)
    ap.add_argument("--no-noise", dest="noise", action="store_false")
    ap.add_argument("--filter", dest="use_filter", action="store_true", default=None)
    ap.add_argument("--no-filter", dest="use_filter", action="store_false")
    ap.add_argument("--no-avoid", action="store_true", help="disable LiDAR obstacle avoidance")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--ignore-target-file", action="store_true", help="always fly the auto mission")
    ap.add_argument("--log", type=str, default=None, help="write a CSV log (for plots)")
    add_config_arguments(ap)
    args = ap.parse_args()
    apply_config_arguments(args)
    use_filter = cfg("estimator.enabled") if args.use_filter is None else args.use_filter

    m = mujoco.MjModel.from_xml_path(project_path("mujoco_model"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    mass = float(m.body_subtreemass[body_id])
    inertia = tuple(float(x) for x in m.body_inertia[body_id])
    dt = m.opt.timestep
    start = tuple(float(x) for x in d.qpos[MJ_POS])

    ctl = PaperBacksteppingController(mass=mass, inertia=inertia, gravity=-float(m.opt.gravity[2]))
    noise = SensorNoiseModel(enabled=args.noise, seed=args.seed)
    est = QuadrotorStateEstimator(dt=dt, initial_pos=start)
    avoider = ReactiveObstacleAvoider(**cfg("mujoco.avoidance"))
    refman = ReferenceManager(start_pos=start,
                              target_path=None if args.ignore_target_file else project_path("target_file"),
                              avoider=None if args.no_avoid else avoider,
                              avoid_gain=cfg("mujoco.avoid_gain"), setpoint_speed=cfg("mujoco.setpoint_speed"),
                              planner=None if args.no_avoid or not cfg("planner.enabled") else GridPlanner(WorldMap.from_city()))
    lidar_range = cfg("mujoco.lidar_range")
    groups = np.array(cfg("mujoco.lidar_geom_groups"), dtype=np.uint8)
    beams = cfg("avoidance.lidar_beams")
    print_every = max(1, int(round(cfg("logging.print_period") / dt)))
    log_every = max(1, int(round(cfg("logging.csv_period") / dt)))

    g = ctl.gains
    print(banner("MUJOCO QUADROTOR - DE MONTE & LOHMANN (2013) FULL BACKSTEPPING CONTROLLER"))
    print(f"[INFO] Config {os.path.relpath(__import__('quadrotor_controller').config.config_path())} | physics "
          f"{1 / dt:.0f} Hz | mass {mass:.3f} kg | gains k_x={g.k_x} k_v={g.k_v} k_t={g.k_t} k_r={g.k_r} "
          f"k_phi={g.k_phi} k_wz={g.k_wz}")
    print(f"[INFO] Noise: {'ON' if noise.enabled else 'OFF'} | Kalman filter: {'ON' if use_filter else 'OFF'} "
          f"| Avoidance: {'OFF' if args.no_avoid else 'ON'}")
    print("[INFO] Commands: python set_target.py X Y Z | auto | spiral | figure8 | hover | land\n")

    log_f = log_w = None
    if args.log:
        log_f = open(args.log, "w", newline="")
        log_w = csv.writer(log_f)
        log_w.writerow(["t", "phase", "ref_x", "ref_y", "ref_z", "ref_yaw", "x", "y", "z", "yaw",
                        "est_x", "est_y", "est_z", "meas_x", "meas_y", "meas_z", "thrust",
                        "tau_x", "tau_y", "tau_z", "err", "lyapunov", "avoiding"])

    sim = {"t": 0.0, "step": 0, "last": None}

    def physics_step():
        t = sim["t"]
        pos = tuple(float(x) for x in d.qpos[MJ_POS])
        quat = tuple(float(x) for x in d.qpos[MJ_QUAT])
        true = ControllerState(position=pos, velocity=tuple(float(x) for x in d.qvel[MJ_LIN_VEL]),
                               quaternion=quat, angular_rate=tuple(float(x) for x in d.qvel[MJ_ANG_VEL]))
        yaw = heading_from_quaternion(quat)
        scan = mujoco_lidar(m, d, body_id, pos, yaw, lidar_range, groups, beams)
        ref = refman.update(t, dt, scan, yaw)

        meas = noise.apply_noise(true)
        fb = est.estimate(meas, dt=dt, acc_control=ctl.model_acceleration(meas.quaternion, meas.velocity)) \
            if use_filter else meas
        thrust, tau = ctl.compute(fb, ref, dt)

        d.ctrl[0] = thrust
        d.ctrl[1:4] = tau
        if m.nmocap > 0:
            d.mocap_pos[0] = ref.pos
        mujoco.mj_step(m, d)

        err = math.dist(pos, ref.pos)
        if log_w is not None and sim["step"] % log_every == 0:
            log_w.writerow([f"{t:.3f}", refman.phase(t), *[f"{v:.4f}" for v in ref.pos], f"{ref.yaw:.4f}",
                            *[f"{v:.4f}" for v in pos], f"{yaw:.4f}", *[f"{v:.4f}" for v in fb.position],
                            *[f"{v:.4f}" for v in meas.position], f"{thrust:.3f}", *[f"{v:.5f}" for v in tau],
                            f"{err:.4f}", f"{ctl.last['lyapunov']:.5f}", int(refman.avoiding)])
        if sim["step"] % print_every == 0:
            lid = f"[LIDAR {refman.avoid_status}]" if refman.avoiding else f"LiDAR clear ({scan.min_horizontal_dist:.1f}m)"
            print(f"[{refman.phase(t):20s} t={t:6.1f}s] Pos ({pos[0]:5.2f},{pos[1]:5.2f},{pos[2]:5.2f}) "
                  f"Ref ({ref.pos[0]:5.2f},{ref.pos[1]:5.2f},{ref.pos[2]:5.2f}) err {err * CM_PER_M:5.1f}cm "
                  f"| T {thrust:5.2f}N | KF {math.dist(fb.position, pos) * CM_PER_M:4.1f}cm "
                  f"(noise {math.dist(meas.position, pos) * CM_PER_M:4.1f}cm) | {lid}")
        sim["last"] = (pos, ref, meas, fb, scan, err, thrust)
        sim["t"] += dt
        sim["step"] += 1

    try:
        if args.headless:
            while args.duration <= 0 or sim["t"] < args.duration:
                physics_step()
        else:
            cam = cfg("mujoco.camera")
            overlay_period = cfg("mujoco.overlay_period")
            min_sleep = cfg("logging.realtime_min_sleep")
            tr = cfg("mujoco.trail")
            trail, trail_t = [], -math.inf
            panels = cfg("mujoco.show_side_panels")
            with mujoco.viewer.launch_passive(m, d, show_left_ui=panels, show_right_ui=panels) as viewer:
                if cfg("mujoco.hide_rangefinder_rays"):
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = cam["distance"], cam["azimuth"], cam["elevation"]
                viewer.cam.lookat[:] = cam["lookat"]
                if cfg("mujoco.track_drone"):
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                    viewer.cam.trackbodyid = body_id
                last_overlay = -math.inf
                while viewer.is_running() and (args.duration <= 0 or sim["t"] < args.duration):
                    t0 = time.time()
                    physics_step()
                    if time.time() - last_overlay > overlay_period and sim["last"] is not None:
                        pos, ref, meas, fb, scan, err, thrust = sim["last"]
                        txt = (f"MODE  {refman.phase(sim['t'])}\n"
                               f"POS   act ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})  ref ({ref.pos[0]:.2f}, {ref.pos[1]:.2f}, {ref.pos[2]:.2f})  err {err * CM_PER_M:.1f} cm\n"
                               f"STATE {'KF on' if use_filter else 'KF off'} | KF err {math.dist(fb.position, pos) * CM_PER_M:.1f} cm | noise {math.dist(meas.position, pos) * CM_PER_M:.1f} cm | T {thrust:.2f} N\n"
                               f"LIDAR {refman.avoid_status if refman.avoiding else 'clear (%.1f m)' % scan.min_horizontal_dist}")
                        if hasattr(viewer, "set_texts"):
                            viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT, txt, "")])
                        last_overlay = time.time()
                    if tr["enabled"] and sim["t"] - trail_t >= tr["period"]:
                        trail_t = sim["t"]
                        trail.append(np.array(d.qpos[MJ_POS], dtype=np.float64))
                        del trail[:-tr["segments"] - 1]
                        with viewer.lock():
                            scn = viewer.user_scn
                            n = min(len(trail) - 1, scn.maxgeom)
                            for i in range(n):
                                g = scn.geoms[i]
                                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                                                    np.eye(3).flatten(), np.array(tr["rgba"], dtype=np.float32))
                                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, tr["width"], trail[i], trail[i + 1])
                            scn.ngeom = n
                    viewer.sync()
                    rem = dt - (time.time() - t0)
                    if rem > min_sleep:
                        time.sleep(rem)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        if log_f:
            log_f.close()
            print(f"[INFO] Log written to {args.log}")
    print("[INFO] MuJoCo simulation finished.")


if __name__ == "__main__":
    main()
