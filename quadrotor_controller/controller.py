from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, pi, sin, sqrt
from typing import Tuple


@dataclass
class ControllerState:
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]
    angular_rate: Tuple[float, float, float]


@dataclass
class ControllerReference:
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    acceleration: Tuple[float, float, float]
    heading: float


class BacksteppingController:
    def __init__(self, mass: float = 1.5, gravity: float = 9.81) -> None:
        self.mass = mass
        self.gravity = gravity
        self.k_position = 2.0
        self.k_velocity = 1.5
        self.k_tilt = 3.5
        self.k_yaw = 2.5
        self.k_rate = 0.8

    def compute_command(
        self, state: ControllerState, reference: ControllerReference, dt: float
    ) -> Tuple[float, Tuple[float, float, float]]:
        position_error = self._sub(reference.position, state.position)
        velocity_error = self._sub(reference.velocity, state.velocity)

        desired_acceleration = self._add(
            reference.acceleration,
            self._scale(position_error, self.k_position),
            self._scale(velocity_error, self.k_velocity),
        )

        thrust_vector = self._add(
            (0.0, 0.0, self.mass * self.gravity),
            self._scale(desired_acceleration, self.mass),
        )

        thrust = max(0.0, self._norm(thrust_vector))
        desired_direction = self._normalize(thrust_vector) if thrust > 1e-6 else (0.0, 0.0, 1.0)
        actual_direction = self._thrust_direction(state.quaternion)

        attitude_error = self._cross(actual_direction, desired_direction)
        roll_pitch_torque = self._scale(attitude_error, self.k_tilt)
        yaw_error = self._wrap_angle(reference.heading - self._quaternion_heading(state.quaternion))
        yaw_torque = self.k_yaw * yaw_error

        torques = (
            roll_pitch_torque[0] - self.k_rate * state.angular_rate[0],
            roll_pitch_torque[1] - self.k_rate * state.angular_rate[1],
            yaw_torque - self.k_rate * state.angular_rate[2],
        )
        return thrust, torques


    @staticmethod
    def _sub(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    @staticmethod
    def _add(*vectors: Tuple[float, float, float]) -> Tuple[float, float, float]:
        return tuple(sum(v[i] for v in vectors) for i in range(3))

    @staticmethod
    def _scale(vector: Tuple[float, float, float], scalar: float) -> Tuple[float, float, float]:
        return tuple(vector[i] * scalar for i in range(3))

    @staticmethod
    def _dot(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    @staticmethod
    def _cross(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    @staticmethod
    def _norm(vector: Tuple[float, float, float]) -> float:
        return sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)

    @classmethod
    def _normalize(cls, vector: Tuple[float, float, float]) -> Tuple[float, float, float]:
        norm = cls._norm(vector)
        if norm == 0.0:
            return (0.0, 0.0, 0.0)
        return (vector[0] / norm, vector[1] / norm, vector[2] / norm)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return (angle + pi) % (2.0 * pi) - pi

    @staticmethod
    def _quaternion_multiply(
        a: Tuple[float, float, float, float],
        b: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float, float]:
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    @classmethod
    def _rotate_vector(cls, vector: Tuple[float, float, float], quaternion: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        w, x, y, z = quaternion
        q_conj = (w, -x, -y, -z)
        vec_quat = (0.0, vector[0], vector[1], vector[2])
        rotated = cls._quaternion_multiply(cls._quaternion_multiply(quaternion, vec_quat), q_conj)
        return (rotated[1], rotated[2], rotated[3])

    @classmethod
    def _thrust_direction(cls, quaternion: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        direction = cls._rotate_vector((0.0, 0.0, 1.0), quaternion)
        return cls._normalize(direction)


    @staticmethod
    def _quaternion_heading(quaternion: Tuple[float, float, float, float]) -> float:
        w, x, y, z = quaternion
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return atan2(siny, cosy)


def normalize_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    w, x, y, z = q
    norm = sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = cos(roll / 2.0)
    sr = sin(roll / 2.0)
    cp = cos(pitch / 2.0)
    sp = sin(pitch / 2.0)
    cy = cos(yaw / 2.0)
    sy = sin(yaw / 2.0)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return normalize_quaternion((w, x, y, z))


def decompose_attitude(q: Tuple[float, float, float, float]) -> Tuple[Tuple[float, float, float, float], Tuple[float, float, float, float]]:
    """Decompose unit quaternion q into tilt q_xy and heading q_z rotations.
    
    According to De Monte & Lohmann (2013), equations (10)-(14):
    q = q_xy * q_z
    where q_xy = (qp, qx, qy, 0) and q_z = (qw, 0, 0, qz).
    Quaternion ordering in tuple: (w, x, y, z).
    """
    q = normalize_quaternion(q)
    w, x, y, z = q
    qp = sqrt(z * z + w * w)
    if qp <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)

    qx = (w * x - z * y) / qp
    qy = (w * y + z * x) / qp
    qw = abs(w) / qp
    qz_sign = 1.0 if z * w >= 0.0 else -1.0
    qz = qz_sign * abs(z) / qp

    q_xy = normalize_quaternion((qp, qx, qy, 0.0))
    q_z = normalize_quaternion((qw, 0.0, 0.0, qz))
    return q_xy, q_z

