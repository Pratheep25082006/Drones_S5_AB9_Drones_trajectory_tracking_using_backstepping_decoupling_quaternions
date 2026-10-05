"""
Unit Tests for Quadrotor Kalman Filter and Sensor Noise Model
"""

import math
import os
import sys
import numpy as np
import pytest

# Ensure project root is on path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from quadrotor_controller import (
    AngularRateFilter,
    BacksteppingController,
    ControllerReference,
    ControllerState,
    KalmanFilter,
    QuadrotorStateEstimator,
    SensorNoiseModel,
)


def test_sensor_noise_model_generation():
    """Verify SensorNoiseModel adds Gaussian noise and supports disable/enable."""
    state = ControllerState(
        position=(1.0, 2.0, 3.0),
        velocity=(0.5, -0.5, 1.0),
        quaternion=(1.0, 0.0, 0.0, 0.0),
        angular_rate=(0.0, 0.0, 0.0),
    )

    noise_model = SensorNoiseModel(pos_std=0.05, vel_std=0.08, rate_std=0.02, enabled=True, seed=42)
    noisy_state = noise_model.apply_noise(state)

    # Position, velocity, and rate should differ from true state due to noise
    assert noisy_state.position != state.position
    assert noisy_state.velocity != state.velocity
    assert noisy_state.angular_rate != state.angular_rate
    assert noisy_state.quaternion == state.quaternion

    # When disabled, should return exact state
    noise_model.enabled = False
    clean_state = noise_model.apply_noise(state)
    assert clean_state.position == state.position
    assert clean_state.velocity == state.velocity


def test_sensor_noise_statistics():
    """Verify noise variance matches specified standard deviations over large sample."""
    state = ControllerState(
        position=(0.0, 0.0, 0.0),
        velocity=(0.0, 0.0, 0.0),
        quaternion=(1.0, 0.0, 0.0, 0.0),
        angular_rate=(0.0, 0.0, 0.0),
    )

    pos_std = 0.05
    noise_model = SensorNoiseModel(pos_std=pos_std, enabled=True, seed=123)

    pos_samples = []
    for _ in range(2000):
        noisy = noise_model.apply_noise(state)
        pos_samples.append(noisy.position[0])

    empirical_std = float(np.std(pos_samples))
    empirical_mean = float(np.mean(pos_samples))

    assert abs(empirical_mean) < 0.01, f"Mean noise should be ~0, got {empirical_mean}"
    assert abs(empirical_std - pos_std) < 0.005, f"Std should be ~{pos_std}, got {empirical_std}"


def test_kalman_filter_dimensions_and_reset():
    """Verify matrix dimensions and reset functionality."""
    kf = KalmanFilter(dt=0.01, initial_pos=(1.0, 2.0, 3.0), initial_vel=(0.1, 0.2, 0.3))

    assert kf.x.shape == (6,)
    assert kf.F.shape == (6, 6)
    assert kf.Q.shape == (6, 6)
    assert kf.P.shape == (6, 6)
    assert kf.B.shape == (6, 3)

    pos, vel = kf.get_state()
    assert pos == (1.0, 2.0, 3.0)
    assert vel == (0.1, 0.2, 0.3)

    kf.reset(pos=(0.0, 0.0, 1.0), vel=(0.0, 0.0, 0.0))
    pos2, vel2 = kf.get_state()
    assert pos2 == (0.0, 0.0, 1.0)
    assert vel2 == (0.0, 0.0, 0.0)


def test_kalman_filter_noise_suppression():
    """
    Simulate a moving quadrotor with noisy sensor measurements.
    Verify that the Kalman Filter significantly reduces RMSE compared to raw noise.
    """
    dt = 0.01
    num_steps = 500
    pos_noise_std = 0.10  # 10 cm noise
    vel_noise_std = 0.15  # 15 cm/s noise

    kf = KalmanFilter(
        dt=dt,
        process_noise_acc=0.5,
        meas_pos_std=pos_noise_std,
        meas_vel_std=vel_noise_std,
        initial_pos=(0.0, 0.0, 0.0),
        initial_vel=(1.0, 0.5, 0.0),
    )

    rng = np.random.default_rng(999)

    true_positions = []
    noisy_positions = []
    filtered_positions = []

    # True trajectory: 3D circular helix
    omega = 1.0
    for i in range(num_steps):
        t = i * dt
        true_px = math.cos(omega * t)
        true_py = math.sin(omega * t)
        true_pz = 0.5 * t
        true_vx = -omega * math.sin(omega * t)
        true_vy = omega * math.cos(omega * t)
        true_vz = 0.5

        # Add Gaussian noise
        meas_px = true_px + rng.normal(0.0, pos_noise_std)
        meas_py = true_py + rng.normal(0.0, pos_noise_std)
        meas_pz = true_pz + rng.normal(0.0, pos_noise_std)
        meas_vx = true_vx + rng.normal(0.0, vel_noise_std)
        meas_vy = true_vy + rng.normal(0.0, vel_noise_std)
        meas_vz = true_vz + rng.normal(0.0, vel_noise_std)

        # Update Kalman Filter
        est_pos, est_vel = kf.step(
            meas_pos=(meas_px, meas_py, meas_pz),
            meas_vel=(meas_vx, meas_vy, meas_vz),
            dt=dt,
        )

        true_positions.append([true_px, true_py, true_pz])
        noisy_positions.append([meas_px, meas_py, meas_pz])
        filtered_positions.append(list(est_pos))

    true_arr = np.array(true_positions)
    noisy_arr = np.array(noisy_positions)
    filt_arr = np.array(filtered_positions)

    # Compute Root Mean Squared Errors
    raw_rmse = float(np.sqrt(np.mean((noisy_arr - true_arr) ** 2)))
    filtered_rmse = float(np.sqrt(np.mean((filt_arr - true_arr) ** 2)))

    # Filtered RMSE should be significantly lower than raw noisy RMSE (> 40% reduction)
    assert filtered_rmse < raw_rmse, f"Filter should reduce noise: Filtered={filtered_rmse:.4f}, Raw={raw_rmse:.4f}"
    noise_reduction_pct = (1.0 - filtered_rmse / raw_rmse) * 100.0
    assert noise_reduction_pct > 40.0, f"Expected >40% noise reduction, got {noise_reduction_pct:.1f}%"


def test_angular_rate_filter():
    """Verify AngularRateFilter attenuates high-frequency rate chatter."""
    rate_filter = AngularRateFilter(alpha=0.2)
    clean_rate = (0.5, 0.2, -0.1)

    rng = np.random.default_rng(42)
    filtered_rates = []
    for _ in range(100):
        noisy_rate = (
            clean_rate[0] + rng.normal(0.0, 0.1),
            clean_rate[1] + rng.normal(0.0, 0.1),
            clean_rate[2] + rng.normal(0.0, 0.1),
        )
        filtered = rate_filter.update(noisy_rate)
        filtered_rates.append(filtered)

    final_est = filtered_rates[-1]
    for i in range(3):
        assert abs(final_est[i] - clean_rate[i]) < 0.05


def test_quadrotor_state_estimator_with_backstepping_controller():
    """Verify QuadrotorStateEstimator integrates seamlessly with BacksteppingController."""
    estimator = QuadrotorStateEstimator(dt=0.002, initial_pos=(0.0, 0.0, 0.2), initial_vel=(0.0, 0.0, 0.0))
    controller = BacksteppingController(mass=1.5, gravity=9.81)

    true_state = ControllerState(
        position=(0.0, 0.0, 0.2),
        velocity=(0.0, 0.0, 0.0),
        quaternion=(1.0, 0.0, 0.0, 0.0),
        angular_rate=(0.0, 0.0, 0.0),
    )
    noise_model = SensorNoiseModel(pos_std=0.05, vel_std=0.08, enabled=True, seed=1)
    ref = ControllerReference((0.0, 0.0, 2.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)

    for _ in range(50):
        noisy_state = noise_model.apply_noise(true_state)
        estimated_state = estimator.estimate(noisy_state, dt=0.002)

        # Backstepping controller computes commands on estimated state
        thrust, torques = controller.compute_command(estimated_state, ref, dt=0.002)

        assert not math.isnan(thrust)
        assert not math.isinf(thrust)
        assert len(torques) == 3
        assert all(not math.isnan(t) for t in torques)
