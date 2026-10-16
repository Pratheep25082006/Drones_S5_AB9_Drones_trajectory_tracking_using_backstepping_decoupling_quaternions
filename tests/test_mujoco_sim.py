"""
Unit Tests for MuJoCo 3D Quadrotor Simulation
"""

import os
import sys
import math
import pytest
import numpy as np

# Ensure root directory is on path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import mujoco
from quadrotor_controller import (
    BacksteppingController,
    ControllerReference,
    ControllerState,
    QuadrotorStateEstimator,
    SensorNoiseModel,
)


def test_mujoco_model_loading():
    """Verify models/quadrotor.xml loads valid MJCF XML with correct dimensions"""
    model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "models", "quadrotor.xml"))
    assert os.path.exists(model_path), f"Model XML not found at {model_path}"

    m = mujoco.MjModel.from_xml_path(model_path)
    assert m is not None
    assert m.nq == 7   # 3 position + 4 quaternion
    assert m.nv == 6   # 3 linear vel + 3 angular vel
    assert m.nu == 4   # 1 thrust + 3 torques


def test_mujoco_simulation_step():
    """Verify stepping MuJoCo physics with backstepping controller produces stable numbers"""
    model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "models", "quadrotor.xml"))
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)

    controller = BacksteppingController(mass=1.5, gravity=9.81)
    ref = ControllerReference((0.0, 0.0, 2.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)

    for _ in range(100):
        pos = (float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2]))
        vel = (float(d.qvel[0]), float(d.qvel[1]), float(d.qvel[2]))
        quat = (float(d.qpos[3]), float(d.qpos[4]), float(d.qpos[5]), float(d.qpos[6]))
        ang_vel = (float(d.qvel[3]), float(d.qvel[4]), float(d.qvel[5]))

        state = ControllerState(pos, vel, quat, ang_vel)
        thrust, torques = controller.compute_command(state, ref, dt=m.opt.timestep)

        d.ctrl[0] = thrust
        d.ctrl[1] = torques[0]
        d.ctrl[2] = torques[1]
        d.ctrl[3] = torques[2]

        mujoco.mj_step(m, d)

        assert not np.isnan(d.qpos).any(), "qpos contains NaN"
        assert not np.isinf(d.qpos).any(), "qpos contains Inf"


def test_mujoco_target_tracking():
    """Full paper controller drives the MuJoCo rigid body (actuators only) to a hover point."""
    from quadrotor_controller import PaperBacksteppingController, SetpointReferenceModel

    model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "models", "quadrotor.xml"))
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    ctl = PaperBacksteppingController(mass=float(m.body_subtreemass[bid]), inertia=tuple(m.body_inertia[bid]))
    rm = SetpointReferenceModel(tuple(d.qpos[0:3]), v_max=1.5)
    rm.set_target((1.0, -1.0, 1.5), 0.5)
    dt = m.opt.timestep
    for _ in range(int(12.0 / dt)):
        ref = rm.step(dt)
        s = ControllerState(tuple(d.qpos[0:3]), tuple(d.qvel[0:3]), tuple(d.qpos[3:7]), tuple(d.qvel[3:6]))
        thrust, tau = ctl.compute(s, ref, dt)
        d.ctrl[0] = thrust
        d.ctrl[1:4] = tau
        mujoco.mj_step(m, d)
    assert np.linalg.norm(d.qpos[0:3] - np.array([1.0, -1.0, 1.5])) < 0.02


def test_mujoco_mocap_beacon():
    """Verify mocap body exists and tracks dynamic reference coordinates"""
    model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "models", "quadrotor.xml"))
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)

    assert m.nmocap >= 1, "Expected at least 1 mocap body for the target beacon"
    d.mocap_pos[0] = [4.5, 3.2, 5.0]
    mujoco.mj_forward(m, d)
    assert np.allclose(d.mocap_pos[0], [4.5, 3.2, 5.0])


def test_mujoco_world_arena():
    """Verify real-world arena has expected obstacle towers, gates and helipad geometry"""
    model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "models", "quadrotor.xml"))
    m = mujoco.MjModel.from_xml_path(model_path)
    assert m.ngeom >= 50, f"Expected rich worldbody arena with >= 50 geoms, got {m.ngeom}"


def test_mujoco_with_noise_and_kalman_filter():
    """Verify that MuJoCo stepping under sensor noise + Kalman filter is numerically stable and suppresses error."""
    model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "models", "quadrotor.xml"))
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)

    dt = m.opt.timestep
    noise_model = SensorNoiseModel(pos_std=0.05, vel_std=0.08, enabled=True, seed=123)
    estimator = QuadrotorStateEstimator(dt=dt, meas_pos_std=0.05, meas_vel_std=0.08)
    controller = BacksteppingController(mass=1.5, gravity=9.81)
    ref = ControllerReference((0.0, 0.0, 2.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)

    noise_errors = []
    filter_errors = []

    for _ in range(200):
        true_pos = (float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2]))
        true_vel = (float(d.qvel[0]), float(d.qvel[1]), float(d.qvel[2]))
        true_quat = (float(d.qpos[3]), float(d.qpos[4]), float(d.qpos[5]), float(d.qpos[6]))
        true_ang_vel = (float(d.qvel[3]), float(d.qvel[4]), float(d.qvel[5]))

        true_state = ControllerState(true_pos, true_vel, true_quat, true_ang_vel)
        noisy_state = noise_model.apply_noise(true_state)
        est_state = estimator.estimate(noisy_state, dt=dt)

        noise_errors.append(math.sqrt(sum((noisy_state.position[i] - true_pos[i])**2 for i in range(3))))
        filter_errors.append(math.sqrt(sum((est_state.position[i] - true_pos[i])**2 for i in range(3))))

        thrust, torques = controller.compute_command(est_state, ref, dt=dt)
        d.ctrl[0] = thrust
        d.ctrl[1] = torques[0]
        d.ctrl[2] = torques[1]
        d.ctrl[3] = torques[2]

        mujoco.mj_step(m, d)

        assert not np.isnan(d.qpos).any()
        assert not np.isnan(d.qvel).any()

    # Average Kalman filter error should be lower than average raw sensor noise
    avg_noise_err = float(np.mean(noise_errors[50:]))
    avg_filter_err = float(np.mean(filter_errors[50:]))
    assert avg_filter_err < avg_noise_err, f"KF error ({avg_filter_err:.4f}) should be lower than noise ({avg_noise_err:.4f})"

