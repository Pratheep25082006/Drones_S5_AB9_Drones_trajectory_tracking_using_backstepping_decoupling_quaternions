"""
Unit Tests for gym-pybullet-drones Quadrotor Simulation
"""

import math
import os
import sys
import numpy as np
import pytest

# Ensure root directory is on path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from quadrotor_controller import (
    BacksteppingController,
    ControllerReference,
    ControllerState,
    QuadrotorStateEstimator,
    SensorNoiseModel,
)
from run_pybullet_simulation import ForceTorqueMixer


def test_pybullet_env_initialization():
    """Verify gym-pybullet-drones CtrlAviary loads properly with valid parameters."""
    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=np.array([[0.0, 0.0, 0.5]]),
        gui=False,
    )
    assert env.M > 0, "Drone mass should be positive"
    assert env.L > 0, "Arm length should be positive"
    assert env.KF > 0, "Thrust coefficient should be positive"
    obs, info = env.reset()
    assert obs.shape == (1, 20), f"Expected obs shape (1, 20), got {obs.shape}"
    env.close()


def test_force_torque_mixer():
    """Verify mixer computes symmetric hover RPMs for pure vertical thrust."""
    arm_length = 0.0397
    kf = 3.16e-10
    km = 7.94e-12
    mixer = ForceTorqueMixer(arm_length, kf, km)

    mass = 0.027
    g = 9.81
    hover_thrust = mass * g

    rpms = mixer.compute_rpms(thrust=hover_thrust, torques=(0.0, 0.0, 0.0), max_rpm=np.inf)
    assert len(rpms) == 4
    assert np.allclose(rpms, rpms[0]), "All 4 rotor RPMs should be equal in hover"
    assert rpms[0] > 10000.0, f"Expected hover RPM ~14000, got {rpms[0]}"


def test_pybullet_step_with_noise_and_kalman_filter():
    """Verify stepping gym-pybullet-drones with backstepping controller, noise and Kalman filter."""
    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=np.array([[0.0, 0.0, 0.2]]),
        gui=False,
    )
    obs, info = env.reset()
    dt = 1.0 / env.CTRL_FREQ

    controller = BacksteppingController(mass=env.M, gravity=env.G)
    mixer = ForceTorqueMixer(arm_length=float(env.L), kf=float(env.KF), km=float(env.KM))
    noise_model = SensorNoiseModel(pos_std=0.03, vel_std=0.05, rate_std=0.02, enabled=True, seed=123)
    estimator = QuadrotorStateEstimator(
        dt=dt,
        meas_pos_std=0.03,
        meas_vel_std=0.05,
        initial_pos=(0.0, 0.0, 0.2),
        initial_vel=(0.0, 0.0, 0.0),
    )
    ref = ControllerReference((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)

    noise_errs = []
    kf_errs = []

    for _ in range(100):
        true_pos = tuple(float(x) for x in obs[0, 0:3])
        q_xyzw = obs[0, 3:7]
        true_quat = (float(q_xyzw[3]), float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2]))
        true_vel = tuple(float(x) for x in obs[0, 10:13])
        true_ang_vel = tuple(float(x) for x in obs[0, 13:16])

        true_state = ControllerState(true_pos, true_vel, true_quat, true_ang_vel)
        noisy_state = noise_model.apply_noise(true_state)
        feedback_state = estimator.estimate(noisy_state, dt=dt)

        noise_errs.append(math.sqrt(sum((noisy_state.position[i] - true_pos[i]) ** 2 for i in range(3))))
        kf_errs.append(math.sqrt(sum((feedback_state.position[i] - true_pos[i]) ** 2 for i in range(3))))

        thrust, torques = controller.compute_command(feedback_state, ref, dt=dt)
        rpms = mixer.compute_rpms(thrust, torques, max_rpm=float(env.MAX_RPM))

        obs, reward, terminated, truncated, info = env.step(rpms.reshape(1, 4))
        assert not np.isnan(obs).any(), "Observation contains NaN"

    env.close()

    avg_noise = float(np.mean(noise_errs[30:]))
    avg_kf = float(np.mean(kf_errs[30:]))
    assert avg_kf < avg_noise, f"Kalman filter should reduce error: KF={avg_kf:.4f}, Noise={avg_noise:.4f}"


def test_pybullet_paper_controller_hover():
    """Full paper controller + CF2X mixer hovers the Crazyflie at 1 m (body-frame gyro rates)."""
    import pybullet as p
    from quadrotor_controller import PaperBacksteppingController, SetpointReferenceModel, quat_to_rot

    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=1, initial_xyzs=np.array([[0.0, 0.0, 0.08]]),
                     physics=Physics.PYB, pyb_freq=1000, ctrl_freq=500, gui=False)
    obs, _ = env.reset()
    d = env.DRONE_IDS[0]
    for link in range(-1, p.getNumJoints(d, physicsClientId=env.CLIENT)):
        p.changeDynamics(d, link, linearDamping=0.0, angularDamping=0.0, physicsClientId=env.CLIENT)
    dt = 1.0 / env.CTRL_FREQ
    ctl = PaperBacksteppingController(mass=env.M, inertia=(env.J[0, 0], env.J[1, 1], env.J[2, 2]), gravity=env.G,
                                      thrust_limits=(0.15 * env.M * env.G, 0.9 * 4 * env.KF * env.MAX_RPM ** 2))
    mixer = ForceTorqueMixer(arm_length=float(env.L), kf=float(env.KF), km=float(env.KM))
    rm = SetpointReferenceModel((0.0, 0.0, 0.08))
    rm.set_target((0.0, 0.0, 1.0))
    for _ in range(int(6.0 / dt)):
        ref = rm.step(dt)
        qx, qy, qz, qw = obs[0, 3:7]
        q = (qw, qx, qy, qz)
        w_body = quat_to_rot(q).T @ obs[0, 13:16]
        s = ControllerState(tuple(obs[0, 0:3]), tuple(obs[0, 10:13]), q, tuple(w_body))
        thrust, tau = ctl.compute(s, ref, dt)
        obs, *_ = env.step(mixer.compute_rpms(thrust, tau, max_rpm=float(env.MAX_RPM)).reshape(1, 4))
    pos = obs[0, 0:3]
    env.close()
    assert np.linalg.norm(pos - np.array([0.0, 0.0, 1.0])) < 0.02
