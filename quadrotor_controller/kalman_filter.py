from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple, Union
import numpy as np

from .controller import ControllerState


class SensorNoiseModel:
    """
    Sensor Noise Model for Quadrotor Simulation.
    Simulates realistic sensor inaccuracies (e.g. GPS/Mocap position noise,
    optical flow/Doppler velocity noise, and IMU gyroscope rate noise).
    """

    def __init__(
        self,
        pos_std: float = 0.05,       # standard deviation in meters (e.g. 5 cm)
        vel_std: float = 0.08,       # standard deviation in m/s (e.g. 8 cm/s)
        rate_std: float = 0.02,      # standard deviation in rad/s (~1.1 deg/s)
        enabled: bool = True,
        seed: Optional[int] = None,
    ) -> None:
        self.pos_std = float(pos_std)
        self.vel_std = float(vel_std)
        self.rate_std = float(rate_std)
        self.enabled = enabled
        self._rng = np.random.default_rng(seed)

    def set_seed(self, seed: Optional[int]) -> None:
        self._rng = np.random.default_rng(seed)

    def apply_noise(self, true_state: ControllerState) -> ControllerState:
        """
        Corrupts ground-truth state with additive Gaussian white noise.
        """
        if not self.enabled:
            return true_state

        pos_noise = self._rng.normal(0.0, self.pos_std, size=3)
        vel_noise = self._rng.normal(0.0, self.vel_std, size=3)
        rate_noise = self._rng.normal(0.0, self.rate_std, size=3)

        noisy_pos = (
            float(true_state.position[0] + pos_noise[0]),
            float(true_state.position[1] + pos_noise[1]),
            float(true_state.position[2] + pos_noise[2]),
        )
        noisy_vel = (
            float(true_state.velocity[0] + vel_noise[0]),
            float(true_state.velocity[1] + vel_noise[1]),
            float(true_state.velocity[2] + vel_noise[2]),
        )
        noisy_rate = (
            float(true_state.angular_rate[0] + rate_noise[0]),
            float(true_state.angular_rate[1] + rate_noise[1]),
            float(true_state.angular_rate[2] + rate_noise[2]),
        )

        return ControllerState(
            position=noisy_pos,
            velocity=noisy_vel,
            quaternion=true_state.quaternion,
            angular_rate=noisy_rate,
        )


class KalmanFilter:
    """
    Discrete-Time Linear Kalman Filter for 3D Quadrotor Translation.
    
    State vector (6-DOF):
        x = [p_x, p_y, p_z, v_x, v_y, v_z]^T
        
    Kinematic State Transition:
        p_{k+1} = p_k + v_k * dt + 0.5 * a_k * dt^2
        v_{k+1} = v_k + a_k * dt
        
    Matrices:
        F: State transition matrix (6x6)
        B: Control input matrix (6x3) for external acceleration u = [ax, ay, az]^T
        Q: Process noise covariance matrix (6x6)
        H: Measurement matrix (6x6 or 3x6)
        R: Measurement noise covariance matrix
        P: State estimation error covariance matrix (6x6)
    """

    def __init__(
        self,
        dt: float = 0.002,
        process_noise_acc: float = 0.5,    # process acceleration noise std (m/s^2)
        meas_pos_std: float = 0.05,        # position measurement noise std (m)
        meas_vel_std: float = 0.08,        # velocity measurement noise std (m/s)
        initial_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        initial_vel: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self.dt = float(dt)
        self.process_noise_acc = float(process_noise_acc)
        self.meas_pos_std = float(meas_pos_std)
        self.meas_vel_std = float(meas_vel_std)

        # State vector: [px, py, pz, vx, vy, vz]
        self.x = np.array([
            initial_pos[0], initial_pos[1], initial_pos[2],
            initial_vel[0], initial_vel[1], initial_vel[2]
        ], dtype=np.float64)

        # Initial estimation error covariance P
        self.P = np.diag([
            meas_pos_std**2, meas_pos_std**2, meas_pos_std**2,
            meas_vel_std**2, meas_vel_std**2, meas_vel_std**2
        ]).astype(np.float64)

        # Build dynamic matrices
        self._update_transition_matrices(self.dt)

    def _update_transition_matrices(self, dt: float) -> None:
        """Builds F, B, and continuous-to-discrete Q matrices for given dt."""
        self.dt = float(dt)
        # F: 6x6 transition matrix
        self.F = np.eye(6, dtype=np.float64)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        # B: 6x3 control matrix
        self.B = np.zeros((6, 3), dtype=np.float64)
        half_dt2 = 0.5 * (dt ** 2)
        self.B[0, 0] = half_dt2
        self.B[1, 1] = half_dt2
        self.B[2, 2] = half_dt2
        self.B[3, 0] = dt
        self.B[4, 1] = dt
        self.B[5, 2] = dt

        # Discrete-time process noise covariance Q using continuous white noise acceleration model
        q_var = self.process_noise_acc ** 2
        q_pos_var = q_var * (dt ** 4) / 4.0
        q_pos_vel = q_var * (dt ** 3) / 2.0
        q_vel_var = q_var * (dt ** 2)

        self.Q = np.zeros((6, 6), dtype=np.float64)
        for i in range(3):
            self.Q[i, i] = q_pos_var
            self.Q[i, i + 3] = q_pos_vel
            self.Q[i + 3, i] = q_pos_vel
            self.Q[i + 3, i + 3] = q_vel_var

        # Default measurement covariance R (6x6)
        self.R_full = np.diag([
            self.meas_pos_std**2, self.meas_pos_std**2, self.meas_pos_std**2,
            self.meas_vel_std**2, self.meas_vel_std**2, self.meas_vel_std**2
        ]).astype(np.float64)

    def reset(
        self,
        pos: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        vel: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        """Resets state and covariance."""
        self.x = np.array([pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]], dtype=np.float64)
        self.P = np.diag([
            self.meas_pos_std**2, self.meas_pos_std**2, self.meas_pos_std**2,
            self.meas_vel_std**2, self.meas_vel_std**2, self.meas_vel_std**2
        ]).astype(np.float64)

    def predict(
        self,
        dt: Optional[float] = None,
        acc_control: Optional[Tuple[float, float, float]] = None,
    ) -> np.ndarray:
        """
        Kalman Filter Prediction Step:
            x_{k|k-1} = F * x_{k-1|k-1} + B * u
            P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q
        """
        if dt is not None and abs(dt - self.dt) > 1e-6:
            self._update_transition_matrices(dt)

        if acc_control is not None:
            u = np.array(acc_control, dtype=np.float64)
            self.x = self.F @ self.x + self.B @ u
        else:
            self.x = self.F @ self.x

        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x

    def update(
        self,
        meas_pos: Tuple[float, float, float],
        meas_vel: Optional[Tuple[float, float, float]] = None,
        R_override: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Kalman Filter Measurement Update Step.
        Supports full [pos, vel] observation or position-only observation.
        """
        if meas_vel is not None:
            # 6-DOF measurement
            z = np.array([
                meas_pos[0], meas_pos[1], meas_pos[2],
                meas_vel[0], meas_vel[1], meas_vel[2]
            ], dtype=np.float64)
            H = np.eye(6, dtype=np.float64)
            R = R_override if R_override is not None else self.R_full
        else:
            # 3-DOF position-only measurement
            z = np.array([meas_pos[0], meas_pos[1], meas_pos[2]], dtype=np.float64)
            H = np.zeros((3, 6), dtype=np.float64)
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            H[2, 2] = 1.0
            R = R_override if R_override is not None else np.diag([
                self.meas_pos_std**2, self.meas_pos_std**2, self.meas_pos_std**2
            ])

        # Innovation: y = z - H * x
        y = z - H @ self.x

        # Innovation covariance: S = H * P * H^T + R
        S = H @ self.P @ H.T + R

        # Optimal Kalman gain: K = P * H^T * S^-1
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update: x = x + K * y
        self.x = self.x + K @ y

        # Covariance update (Joseph form for numerical symmetry & positive-definiteness)
        I_KH = np.eye(6, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        return self.x

    def step(
        self,
        meas_pos: Tuple[float, float, float],
        meas_vel: Optional[Tuple[float, float, float]] = None,
        dt: Optional[float] = None,
        acc_control: Optional[Tuple[float, float, float]] = None,
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """
        Performs predict + update in one call.
        Returns estimated (position, velocity).
        """
        self.predict(dt=dt, acc_control=acc_control)
        self.update(meas_pos, meas_vel)
        return self.get_state()

    def get_state(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """Returns ((px, py, pz), (vx, vy, vz))"""
        pos = (float(self.x[0]), float(self.x[1]), float(self.x[2]))
        vel = (float(self.x[3]), float(self.x[4]), float(self.x[5]))
        return pos, vel


class AngularRateFilter:
    """
    1st-order / Steady-State Kalman Filter for 3D Angular Rates (Gyroscope).
    Denoises high-frequency gyro vibrations before passing to the attitude controller.
    """

    def __init__(self, alpha: float = 0.25) -> None:
        self.alpha = float(alpha)
        self.filtered_rate = np.zeros(3, dtype=np.float64)
        self.initialized = False

    def reset(self, initial_rate: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self.filtered_rate = np.array(initial_rate, dtype=np.float64)
        self.initialized = False

    def update(self, raw_rate: Tuple[float, float, float]) -> Tuple[float, float, float]:
        rate_arr = np.array(raw_rate, dtype=np.float64)
        if not self.initialized:
            self.filtered_rate = rate_arr
            self.initialized = True
        else:
            self.filtered_rate += self.alpha * (rate_arr - self.filtered_rate)

        return (
            float(self.filtered_rate[0]),
            float(self.filtered_rate[1]),
            float(self.filtered_rate[2]),
        )


class QuadrotorStateEstimator:
    """
    End-to-End Quadrotor State Estimator.
    Fuses translational Kalman Filter with angular rate filtering
    to produce clean ControllerState from noisy sensor telemetry.
    """

    def __init__(
        self,
        dt: float = 0.002,
        process_noise_acc: float = 0.5,
        meas_pos_std: float = 0.05,
        meas_vel_std: float = 0.08,
        rate_filter_alpha: float = 0.25,
        initial_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        initial_vel: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self.kf = KalmanFilter(
            dt=dt,
            process_noise_acc=process_noise_acc,
            meas_pos_std=meas_pos_std,
            meas_vel_std=meas_vel_std,
            initial_pos=initial_pos,
            initial_vel=initial_vel,
        )
        self.rate_filter = AngularRateFilter(alpha=rate_filter_alpha)

    def reset(
        self,
        pos: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        vel: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self.kf.reset(pos, vel)
        self.rate_filter.reset()

    def estimate(
        self,
        noisy_state: ControllerState,
        dt: float,
        acc_control: Optional[Tuple[float, float, float]] = None,
    ) -> ControllerState:
        """
        Filters the noisy state and returns an optimal estimated ControllerState.
        """
        est_pos, est_vel = self.kf.step(
            meas_pos=noisy_state.position,
            meas_vel=noisy_state.velocity,
            dt=dt,
            acc_control=acc_control,
        )
        est_rate = self.rate_filter.update(noisy_state.angular_rate)

        return ControllerState(
            position=est_pos,
            velocity=est_vel,
            quaternion=noisy_state.quaternion,
            angular_rate=est_rate,
        )
