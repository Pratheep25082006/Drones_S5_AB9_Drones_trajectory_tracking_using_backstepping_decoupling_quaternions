"""
Rigid-body quadrotor model (paper eqs. (4), (25)-(28)) integrated with RK4.

Used by the Gazebo runners, the web bridge and track_error.py so that the
drone motion actually comes from the controller's thrust and torques.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

from .config import pick
from .controller import ControllerState
from .paper_controller import quat_mul, quat_to_rot


class RigidBodyQuadrotor:
    """x_dot = v,  m v_dot = m g + R(q) [0 0 T]',  q_dot = 1/2 q o (0, w),
    J w_dot = tau - w x (J w).  Optional ground plane at z = ground_z."""

    _GROUND_FROM_CONFIG = object()

    def __init__(self, mass=None, inertia=None, gravity=None, position=None,
                 ground_z=_GROUND_FROM_CONFIG, drag=None):
        """None -> config.yaml (vehicle.*, python_sim.start / ground_z).  ground_z=None: no ground."""
        self.m = float(pick(mass, "vehicle.mass"))
        self.drag = float(pick(drag, "vehicle.drag"))     # linear aerodynamic drag f(v) = -drag * v
        self.J = np.diag(np.asarray(pick(inertia, "vehicle.inertia"), dtype=float))
        self.J_inv = np.linalg.inv(self.J)
        self.g = float(pick(gravity, "vehicle.gravity"))
        self.ground_z = pick(None, "python_sim.ground_z") if ground_z is self._GROUND_FROM_CONFIG else ground_z
        self.x = np.array(pick(position, "python_sim.start"), dtype=float)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.w = np.zeros(3)

    def _deriv(self, x, v, q, w, thrust, tau):
        R = quat_to_rot(q)
        acc = np.array([0.0, 0.0, -self.g]) + R[:, 2] * thrust / self.m - self.drag * v
        q_dot = 0.5 * quat_mul(q, (0.0, w[0], w[1], w[2]))
        w_dot = self.J_inv @ (tau - np.cross(w, self.J @ w))
        return v, acc, q_dot, w_dot

    def step(self, thrust: float, torque: Sequence[float], dt: float) -> None:
        tau = np.asarray(torque, dtype=float)
        s = (self.x, self.v, self.q, self.w)
        k1 = self._deriv(*s, thrust, tau)
        s2 = [a + 0.5 * dt * b for a, b in zip(s, k1)]
        k2 = self._deriv(*s2, thrust, tau)
        s3 = [a + 0.5 * dt * b for a, b in zip(s, k2)]
        k3 = self._deriv(*s3, thrust, tau)
        s4 = [a + dt * b for a, b in zip(s, k3)]
        k4 = self._deriv(*s4, thrust, tau)
        new = [a + dt / 6.0 * (b1 + 2 * b2 + 2 * b3 + b4) for a, b1, b2, b3, b4 in zip(s, k1, k2, k3, k4)]
        self.x, self.v, self.q, self.w = new
        self.q /= np.linalg.norm(self.q)
        if self.ground_z is not None and self.x[2] < self.ground_z:
            # simple ground contact: stop at the ground, kill downward motion
            self.x[2] = self.ground_z
            if self.v[2] < 0.0:
                self.v[:] = 0.0
                self.w[:] = 0.0

    def state(self) -> ControllerState:
        return ControllerState(position=tuple(self.x), velocity=tuple(self.v),
                               quaternion=tuple(self.q), angular_rate=tuple(self.w))

    def euler(self) -> Tuple[float, float, float]:
        w, x, y, z = self.q
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return float(roll), float(pitch), float(yaw)
