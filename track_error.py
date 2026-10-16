"""
track_error.py - reproduces the paper's experiments in a pure-Python rigid-body simulation
=========================================================================================
De Monte & Lohmann (2013), Section IV:
  * Fig. 3 - transition between switched set points (reference model, C^4)
  * Fig. 4 - tracking of a vertical helix (T_xy = 6.25 s, T_z = T_phi = 12.5 s)
plus the project's full mission (takeoff -> hover -> helix -> figure-8).

Each experiment is run with
  (a) the full 4-step backstepping law of the paper (PaperBacksteppingController)
  (b) the earlier simplified 2-step law (BacksteppingController) for comparison
and with ideal sensors and with sensor noise + Kalman filter.

Plant: rigid body of config vehicle.* (RK4).  All experiment settings: config.yaml -> experiments.

Usage:
    python track_error.py            # prints tables, writes results_table.md (+ results/*.csv, *.png)
    python track_error.py --set controller.k_x=1.5
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller.units import CM_PER_M, MNM_PER_NM  # noqa: E402
from quadrotor_controller.config import banner, add_config_arguments, apply_config_arguments, cfg, project_path  # noqa: E402
from quadrotor_controller import (  # noqa: E402
    BacksteppingController,
    ControllerReference,
    LoopingMission,
    PaperBacksteppingController,
    QuadrotorStateEstimator,
    RigidBodyQuadrotor,
    SensorNoiseModel,
    SetpointReferenceModel,
    heading_from_quaternion,
    paper_helix,
    quat_to_rot,
)

DT = None          # set from config in main()
# columns of the recorded rows
C_T, C_REF, C_REF_YAW, C_POS, C_YAW, C_SPEED, C_TILT, C_THRUST, C_TAUX = 0, slice(1, 4), 4, slice(5, 8), 8, 9, 10, 11, 12
N_COLS = 13
MASS = INERTIA = GRAVITY = None


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def run(ref_fn, duration, controller="paper", noise=False, start=None, seed=None, record_every=None,
        use_filter=True):
    seed = cfg("experiments.seed") if seed is None else seed
    record_every = cfg("experiments.record_every") if record_every is None else record_every
    start = tuple(ref_fn(0.0).pos) if start is None else start
    plant = RigidBodyQuadrotor(mass=MASS, inertia=INERTIA, position=start, ground_z=None)
    plant.v = np.array(ref_fn(0.0).vel, dtype=float)          # start on the reference (like the paper's plots)
    if controller == "paper":
        ctl = PaperBacksteppingController(mass=MASS, inertia=INERTIA)
    else:
        ctl = BacksteppingController(mass=MASS)
    nm = SensorNoiseModel(enabled=noise, seed=seed)
    est = QuadrotorStateEstimator(dt=DT, initial_pos=start, initial_vel=tuple(plant.v))
    rows = []
    t = 0.0
    prev_tau, chatter = None, []
    last_T = MASS * GRAVITY
    chatter_start = cfg("experiments.chatter_start")
    for i in range(int(round(duration / DT))):
        ref = ref_fn(t)
        true = plant.state()
        meas = nm.apply_noise(true)
        if noise and use_filter:
            if controller == "paper":
                acc = ctl.model_acceleration(meas.quaternion)
            else:  # same model input for the simplified law: g + R(q) e3 T_last / m
                b3 = quat_to_rot(meas.quaternion)[:, 2] * last_T / MASS
                acc = (float(b3[0]), float(b3[1]), float(b3[2] - GRAVITY))
            fb = est.estimate(meas, dt=DT, acc_control=acc)
        else:
            fb = meas
        if controller == "paper":
            T, tau = ctl.compute(fb, ref, DT)
        else:
            T, tau = ctl.compute_command(fb, ControllerReference(ref.pos, ref.vel, ref.acc, ref.yaw), DT)
        last_T = T
        if prev_tau is not None and t > chatter_start:
            chatter.append(sum((a - b) ** 2 for a, b in zip(tau, prev_tau)) / DT ** 2)
        prev_tau = tau
        if i % record_every == 0:
            R3 = 1 - 2 * (plant.q[1] ** 2 + plant.q[2] ** 2)      # cos(tilt) = R[2,2]
            rows.append([t, *ref.pos, ref.yaw, *plant.x, heading_from_quaternion(plant.q),
                         float(np.linalg.norm(plant.v)), math.degrees(math.acos(max(-1, min(1, R3)))), T, tau[0]])
        plant.step(T, tau, DT)
        t += DT
        if not np.all(np.isfinite(plant.x)) or np.max(np.abs(plant.x)) > cfg("experiments.divergence_limit"):
            rows.append([t] + [float("nan")] * (N_COLS - 1))
            break
    run.last_chatter = math.sqrt(sum(chatter) / max(1, len(chatter)))
    return np.array(rows)


def metrics(a, t_from=0.0):
    a = a[a[:, C_T] >= t_from]
    e = np.linalg.norm(a[:, C_POS] - a[:, C_REF], axis=1)
    ey = np.abs([wrap(x) for x in a[:, C_YAW] - a[:, C_REF_YAW]])
    if not np.all(np.isfinite(e)):
        return {"rmse": float("nan"), "max": float("nan"), "yaw_rmse": float("nan"), "vmax": float("nan"),
                "tilt": float("nan"), "T": (float("nan"), float("nan"))}
    return {"rmse": float(np.sqrt(np.mean(e ** 2))), "max": float(e.max()),
            "yaw_rmse": float(np.degrees(np.sqrt(np.mean(ey ** 2)))), "vmax": float(a[:, C_SPEED].max()),
            "tilt": float(a[:, C_TILT].max()), "T": (float(a[:, C_THRUST].min()), float(a[:, C_THRUST].max()))}


def fig3_reference():
    """Switched set points as in the paper's Fig. 3 (config experiments.fig3)."""
    f3 = cfg("experiments.fig3")
    cmds = [((c[0], c[1], c[2]), c[3]) for c in f3["commands"]] * f3["repeats"]
    rm = SetpointReferenceModel(initial_pos=tuple(f3["start"]), tau_min=f3["tau_min"], v_max=f3["v_max"],
                                yaw_tau=f3["yaw_tau"])
    period = f3["period"]
    n_steps = int(round(period * len(cmds) / DT))
    switch_every = int(round(period / DT))
    seq = []
    for i in range(n_steps + 1):
        k = i // switch_every
        if i % switch_every == 0 and k < len(cmds):
            rm.set_target(*cmds[k])
        seq.append(rm.step(DT))
    return lambda tt: seq[min(int(round(tt / DT)), len(seq) - 1)], period * len(cmds)


def save_csv(path, a, header):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(np.round(a, 5).tolist())


def try_plots(outdir, helix, fig3, noise_runs):
    dpi = cfg("experiments.plots.dpi")
    w0, w1 = cfg("experiments.plots.torque_window")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (matplotlib not installed - skipping plots)")
        return
    for name, a, title in [("helix_fig4.png", helix, "Vertical helix (paper Fig. 4) - full backstepping law"),
                           ("setpoints_fig3.png", fig3, "Switched set points (paper Fig. 3) - full backstepping law")]:
        fig, ax = plt.subplots(4, 1, figsize=cfg("experiments.plots.figsize_tracking"), sharex=True)
        labels = ["x (m)", "y (m)", "z (m)", "heading (rad)"]
        colors = cfg("experiments.plots.colors")
        for i in range(len(labels)):          # x, y, z, heading: reference columns 1..4, actual 5..8
            ax[i].plot(a[:, C_T], a[:, C_REF.start + i], "--", color=colors["reference"], label="reference")
            ax[i].plot(a[:, C_T], a[:, C_POS.start + i], color=colors["actual"], label="actual")
            ax[i].set_ylabel(labels[i])
            ax[i].grid(alpha=cfg("experiments.plots.grid_alpha"))
        ax[0].legend(loc="upper right")
        ax[0].set_title(title)
        ax[-1].set_xlabel("time (s)")
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, name), dpi=dpi)
        plt.close(fig)
    fig, ax = plt.subplots(2, 1, figsize=cfg("experiments.plots.figsize_noise"))
    for label, a in noise_runs:
        e = np.linalg.norm(a[:, C_POS] - a[:, C_REF], axis=1)
        ax[0].plot(a[:, C_T], e * CM_PER_M, label=label)
    ax[0].set_ylabel("position error (cm)")
    ax[0].set_title("Helix: position tracking error ...")
    ax[0].grid(alpha=cfg("experiments.plots.grid_alpha"))
    ax[0].legend()
    for label, a in noise_runs:
        if label == "ideal sensors":
            continue
        k = (a[:, C_T] > w0) & (a[:, C_T] < w1)
        ax[1].plot(a[k, C_T], a[k, C_TAUX] * MNM_PER_NM, label=label, lw=1)
    ax[1].set_xlabel("time (s)")
    ax[1].set_ylabel("roll torque command (mN m)")
    ax[1].set_title("... but only the Kalman filter removes the noise from the motor commands")
    ax[1].grid(alpha=cfg("experiments.plots.grid_alpha"))
    ax[1].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "noise_kalman_error.png"), dpi=dpi)
    plt.close(fig)
    print(f"  plots written to {outdir}")


def main():
    global DT, MASS, INERTIA, GRAVITY
    ap = argparse.ArgumentParser(description="Paper experiments (Fig. 3, Fig. 4, mission)")
    add_config_arguments(ap)
    apply_config_arguments(ap.parse_args())
    DT = cfg("experiments.dt")
    MASS, INERTIA, GRAVITY = cfg("vehicle.mass"), tuple(cfg("vehicle.inertia")), cfg("vehicle.gravity")
    outdir = project_path("results_dir")
    os.makedirs(outdir, exist_ok=True)
    print(banner(f"DE MONTE & LOHMANN (2013) - TRACKING EXPERIMENTS (rigid body, RK4 {1 / DT:.0f} Hz)"))

    f4 = cfg("experiments.fig4")
    helix_fn = paper_helix(center=tuple(f4["center"]))   # start on the helix like the paper's plot window
    start_helix = tuple(helix_fn(0.0).pos)
    helix_skip = f4["skip_periods"] * helix_fn.period
    fig3_fn, fig3_T = fig3_reference()
    mis = cfg("experiments.mission")
    mission = LoopingMission(start=tuple(mis["start"]))

    experiments = [
        ("Fig. 3 switched set points", fig3_fn, fig3_T, tuple(cfg("experiments.fig3.start")), 0.0),
        ("Fig. 4 vertical helix", helix_fn, f4["duration"], start_helix, helix_skip),
        ("Mission (takeoff-helix-fig8)", mission, mis["duration"], tuple(mis["start"]), 0.0),
    ]
    results = []
    stored = {}
    for name, fn, T, start, t0 in experiments:
        for ctl in ("paper", "simplified"):
            for noise in (False, True):
                a = run(fn, T, ctl, noise, start)
                m = metrics(a, t0)
                m["chatter"] = run.last_chatter
                results.append((name, ctl, noise, m))
                stored[(name, ctl, noise)] = a
                tag = f"{name:30s} {('full paper law' if ctl == 'paper' else 'simplified 2-step'):17s} {'noise+KF' if noise else 'ideal':8s}"
                print(f"{tag} RMSE {m['rmse'] * CM_PER_M:7.2f} cm  max {m['max'] * CM_PER_M:7.2f} cm  "
                      f"yaw {m['yaw_rmse']:6.2f} deg  vmax {m['vmax']:4.2f} m/s  tilt {m['tilt']:5.1f} deg  chatter {m['chatter']:7.2f}")

    # noise without filter (helix, paper law)
    a_raw = run(helix_fn, f4["duration"], "paper", True, start_helix, use_filter=False)
    m_raw = metrics(a_raw, helix_skip)
    m_raw["chatter"] = run.last_chatter
    print(f"{'Fig. 4 vertical helix':30s} {'full paper law':17s} {'raw noise':8s} RMSE {m_raw['rmse'] * CM_PER_M:7.2f} cm  "
          f"max {m_raw['max'] * CM_PER_M:7.2f} cm  yaw {m_raw['yaw_rmse']:6.2f} deg  chatter {m_raw['chatter']:7.2f}")
    plant_runs = [("ideal sensors", stored[("Fig. 4 vertical helix", "paper", False)]),
                  ("noise, no filter", a_raw),
                  ("noise + Kalman filter", stored[("Fig. 4 vertical helix", "paper", True)])]

    hdr = ["t", "ref_x", "ref_y", "ref_z", "ref_yaw", "x", "y", "z", "yaw", "speed", "tilt_deg", "thrust", "tau_x"]
    save_csv(os.path.join(outdir, "helix_fig4_paper_law.csv"), stored[("Fig. 4 vertical helix", "paper", False)], hdr)
    save_csv(os.path.join(outdir, "setpoints_fig3_paper_law.csv"), stored[("Fig. 3 switched set points", "paper", False)], hdr)
    save_csv(os.path.join(outdir, "mission_paper_law_noise_kf.csv"), stored[("Mission (takeoff-helix-fig8)", "paper", True)], hdr)
    try_plots(outdir, stored[("Fig. 4 vertical helix", "paper", False)],
              stored[("Fig. 3 switched set points", "paper", False)], plant_runs)

    # ---- markdown --------------------------------------------------------
    ref = cfg("experiments.paper_reference")

    def cell(v, scale=CM_PER_M, fmt="{:.2f}"):
        return "diverged" if not math.isfinite(v) else fmt.format(v * scale)

    md = ["## 5. Simulation Results", "",
          "### Position tracking error - paper experiments reproduced",
          "",
          f"Generated by [`track_error.py`](track_error.py): rigid-body model (m = {MASS} kg, J = diag{INERTIA} kg m^2),",
          f"RK4 at {1 / DT:.0f} Hz, gains k = -A from config.yaml: k_x={cfg('controller.k_x')}, k_v={cfg('controller.k_v')}, "
          f"k_t={cfg('controller.k_t')}, k_r={cfg('controller.k_r')}, k_phi={cfg('controller.k_phi')}, k_wz={cfg('controller.k_wz')}.",
          f"Noise: sigma_pos = {cfg('noise.pos_std')} m, sigma_vel = {cfg('noise.vel_std')} m/s, "
          f"sigma_gyro = {cfg('noise.rate_std')} rad/s, fed through the Kalman filter.",
          "",
          "| Experiment | Controller | Sensors | RMSE (cm) | Max error (cm) | Heading RMSE (deg) | Max speed (m/s) | Max tilt (deg) | Torque chatter |",
          "|---|---|---|---:|---:|---:|---:|---:|---:|"]
    for name, ctl, noise, m in results:
        md.append(f"| {name} | {'Full 4-step paper law' if ctl == 'paper' else 'Simplified 2-step (old)'} | "
                  f"{'noise + KF' if noise else 'ideal'} | {cell(m['rmse'])} | {cell(m['max'])} | "
                  f"{cell(m['yaw_rmse'], 1)} | {cell(m['vmax'], 1)} | {cell(m['tilt'], 1, '{:.1f}')} | {cell(m['chatter'], 1)} |")
    if m_raw is not None:
        md.append(f"| Fig. 4 vertical helix | Full 4-step paper law | noise, no filter | {cell(m_raw['rmse'])} | "
                  f"{cell(m_raw['max'])} | {cell(m_raw['yaw_rmse'], 1)} | - | - | {cell(m_raw['chatter'], 1)} |")
    md += ["", f"Helix metrics exclude the first {helix_skip:.2f} s after start, like the paper's plot window.",
           "", "Torque chatter = RMS of d(tau)/dt in N m/s (how jittery the motor commands are); the Kalman",
           "filter's main benefit is here - it removes the sensor noise from the actuator commands.",
           "", f"Paper (AscTec Hummingbird, Vicon): helix max speed {ref['helix_max_speed']} m/s, max tilt "
           f"{ref['helix_max_tilt_deg']} deg; set-point switching max speed {ref['setpoint_max_speed']} m/s, max tilt "
           f"{ref['setpoint_max_tilt_deg']} deg.",
           f"Plots: `{cfg('paths.results_dir')}/helix_fig4.png`, `{cfg('paths.results_dir')}/setpoints_fig3.png`,",
           f"`{cfg('paths.results_dir')}/noise_kalman_error.png`; raw data in `{cfg('paths.results_dir')}/*.csv`."]
    with open(os.path.join(HERE, "results_table.md"), "w", encoding="utf-8") as f:  # README companion file
        f.write("\n".join(md) + "\n")
    print("\n  results_table.md written")


if __name__ == "__main__":
    main()
