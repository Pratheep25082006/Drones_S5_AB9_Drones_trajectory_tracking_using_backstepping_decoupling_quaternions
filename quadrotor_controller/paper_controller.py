"""
Full backstepping trajectory tracking controller of

    P. De Monte and B. Lohmann, "Trajectory Tracking Control for a Quadrotor
    Helicopter based on Backstepping using a Decoupling Quaternion
    Parametrization", MED 2013.

Implemented equations (paper numbering)
---------------------------------------
Attitude decomposition   q = q_xy o q_z                              (5)-(14)
Thrust vector            T = R(q) [0 0 T]'                           (15)
Heading kinematics       phi_dot = w_z - l(q_xy)' R_s(q_z) w_xy       (22)-(24)
Dynamic thrust extension T_ddot = u_T                                (31)
Input transformation     tau = w x (J w) + J tau~   ->  w_dot = tau~   (29)-(30)
Position backstepping    z_x, z_v, z_t, z_r and control law          (32)-(46)
Heading backstepping     z_phi, z_wz and control law                 (47)-(54)

Notes on the implementation
---------------------------
* World frame is ENU (z up) and the body frame is FLU, so the thrust points
  along +z_B and gravity is (0, 0, -g).  The paper uses NED / thrust along -z_B;
  the equations are identical after this sign change.
* The map (27)  T_dot = M(q_xy) D(T) R(q_z) [w_xy; T_dot]  is evaluated in its
  equivalent vector form  T_dot = R(q) [T w_y, -T w_x, T_dot]'  (the same
  matrix product written with the full rotation q = q_xy o q_z).  As in the
  paper, w_z does not appear, i.e. translation and heading are decoupled.
* The thrust error z_t is normalised by the mass (z_t = (T - T_d)/m).  This is
  the paper's design with the Lyapunov weight 1/m^2 on z_t; it makes the gains
  independent of the vehicle mass so the same gains work for the 1.5 kg model
  and the 27 g Crazyflie.  For m = 1 kg it is exactly the paper's law.
* Drag: f(v) = -c v (parameter ``drag`` = c, config vehicle.drag; 0 as in the
  paper's experiments); its first two derivatives enter T_d, T_d_dot, T_d_ddot.
* The derivatives of the virtual controls (needed in (45) and (52)) are
  computed analytically from the model; only d/dt[l' R_s] uses a directional
  derivative along the quaternion kinematics (4).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from .config import cfg, pick
from .controller import ControllerState, decompose_attitude
from .trajectory import TrajectoryPoint

Vec3 = Tuple[float, float, float]


def _gain(name: str):
    return field(default_factory=lambda: float(cfg(f"controller.{name}")))


@dataclass
class PaperGains:
    """Controller gains k = -A (config.yaml -> controller; paper Sec. IV values:
    A_x=-1, A_v=-3, A_t=-8, A_r=-12, a_phi=-2, a_wz=-12)."""

    k_x: float = _gain("k_x")
    k_v: float = _gain("k_v")
    k_t: float = _gain("k_t")
    k_r: float = _gain("k_r")
    k_phi: float = _gain("k_phi")
    k_wz: float = _gain("k_wz")


def quat_to_rot(q: Sequence[float]) -> np.ndarray:
    """Rotation matrix R(q) of eq. (3), body -> world, q = (w, x, y, z)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def heading_from_quaternion(q: Sequence[float]) -> float:
    """phi = 2 arcsin(q_z) of the heading quaternion q_z (paper Sec. II-E)."""
    _, q_z = decompose_attitude(tuple(q))
    return 2.0 * math.asin(max(-1.0, min(1.0, q_z[3])))


def heading_coupling(q: Sequence[float]) -> np.ndarray:
    """Row vector h = l(q_xy)' R_s(q_z) of eq. (23)/(24):  phi_dot = w_z - h . w_xy."""
    q_xy, q_z = decompose_attitude(tuple(q))
    qp, qx, qy, _ = q_xy
    qw, _, _, qz = q_z
    l = np.array([qy, -qx]) / qp
    Rs = np.array([[qw * qw - qz * qz, -2.0 * qz * qw],
                   [2.0 * qz * qw, qw * qw - qz * qz]])
    return l @ Rs


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class PaperBacksteppingController:
    """De Monte & Lohmann (2013) position + heading backstepping tracking controller.

    Call ``compute(state, ref, dt)`` at a fixed rate.  It returns the collective
    thrust T [N] (the state of the dynamic extension T_ddot = u_T) and the body
    torques tau [N m].
    """

    def __init__(self, mass: Optional[float] = None, inertia: Optional[Sequence[float]] = None,
                 gravity: Optional[float] = None, gains: Optional[PaperGains] = None,
                 thrust_limits: Tuple[float, float] | None = None,
                 max_torque: Optional[Sequence[float]] = None, drag: Optional[float] = None) -> None:
        """Arguments left as None are taken from config.yaml (vehicle.*, controller.*)."""
        self.mass = float(pick(mass, "vehicle.mass"))
        self.drag = float(pick(drag, "vehicle.drag"))   # f(v) = -drag * v  [1/s]  (paper eq. 26)
        self.g = float(pick(gravity, "vehicle.gravity"))
        J = np.asarray(pick(inertia, "vehicle.inertia"), dtype=float)
        self.J = np.diag(J) if J.ndim == 1 else J
        self.gains = gains or PaperGains()
        hover = self.mass * self.g
        self.T_min, self.T_max = thrust_limits or (cfg("vehicle.thrust_min_ratio") * hover,
                                                   cfg("vehicle.thrust_max_ratio") * hover)
        self.min_a = float(cfg("controller.min_normalised_thrust"))
        self.h_eps = float(cfg("controller.heading_derivative_step"))
        self.max_torque = None if max_torque is None else np.asarray(max_torque, dtype=float)
        self.reset()

    # ------------------------------------------------------------------
    def reset(self, thrust: Optional[float] = None) -> None:
        """Reset the dynamic thrust extension to hover (T = m g, T_dot = 0)."""
        self.T = self.mass * self.g if thrust is None else float(thrust)
        self.T_dot = 0.0
        self.last: Dict[str, object] = {}

    # ------------------------------------------------------------------
    def compute(self, state: ControllerState, ref: TrajectoryPoint, dt: float
                ) -> Tuple[float, Tuple[float, float, float]]:
        G = self.gains
        m = self.mass
        q = np.asarray(state.quaternion, dtype=float)
        q = q / np.linalg.norm(q)
        if q[0] < 0.0:
            q = -q
        R = quat_to_rot(q)
        b3 = R[:, 2]
        x = np.asarray(state.position, dtype=float)
        v = np.asarray(state.velocity, dtype=float)
        w = np.asarray(state.angular_rate, dtype=float)       # body rates
        gvec = np.array([0.0, 0.0, -self.g])
        xT, vT, aT, jT, sT = (np.asarray(d, dtype=float) for d in ref.derivs())

        a = self.T / m            # normalised thrust magnitude
        a_dot = self.T_dot / m

        # ---- model quantities -------------------------------------------------
        c = self.drag
        t_vec = a * b3                                             # T/m
        t_dot = R @ np.array([a * w[1], -a * w[0], a_dot])         # eq. (27) / m
        f = -c * v                                                 # drag f(v), eq. (26)
        v_dot = gvec + t_vec + f                                   # eq. (26)
        f_dot = -c * v_dot
        v_ddot = t_dot + f_dot
        f_ddot = -c * v_ddot

        # ---- step 1: position  (32)-(34) -------------------------------------
        z_x = x - xT
        z_x_d = v - vT
        z_x_dd = v_dot - aT
        v_d = vT - G.k_x * z_x                                     # (33)
        v_d_dot = aT - G.k_x * z_x_d
        v_d_ddot = jT - G.k_x * z_x_dd
        v_d_dddot = sT - G.k_x * (v_ddot - jT)

        # ---- step 2: velocity  (35)-(38) --------------------------------------
        z_v = v - v_d
        z_v_d = v_dot - v_d_dot
        z_v_dd = v_ddot - v_d_ddot
        t_d = v_d_dot - gvec - f - z_x - G.k_v * z_v               # (37) / m
        t_d_dot = v_d_ddot - f_dot - z_x_d - G.k_v * z_v_d
        t_d_ddot = v_d_dddot - f_ddot - z_x_dd - G.k_v * z_v_dd

        # ---- step 3: thrust vector  (39)-(42) ---------------------------------
        z_t = t_vec - t_d
        z_t_d = t_dot - t_d_dot
        w_des = t_d_dot - G.k_t * z_t - z_v                        # desired T_dot/m, (41)
        w_des_dot = t_d_ddot - G.k_t * z_t_d - z_v_d

        u = R.T @ w_des                                            # body frame
        u_dot = R.T @ w_des_dot - np.cross(w, u)
        a_safe = max(a, self.min_a)
        omega_xd = -u[1] / a_safe
        omega_yd = u[0] / a_safe
        a_dot_d = u[2]
        omega_xd_dot = -(u_dot[1] * a_safe - u[1] * a_dot) / a_safe ** 2
        omega_yd_dot = (u_dot[0] * a_safe - u[0] * a_dot) / a_safe ** 2
        a_ddot_d = u_dot[2]

        # ---- step 4: angular rate + thrust rate  (43)-(46) --------------------
        z_r = np.array([w[0] - omega_xd, w[1] - omega_yd, a_dot - a_dot_d])
        y = R.T @ z_t
        GTz = np.array([-a * y[1], a * y[0], y[2]])                # (M D R)' z_t
        cmd = np.array([omega_xd_dot, omega_yd_dot, a_ddot_d]) - G.k_r * z_r - GTz   # (45)
        tau_xy = cmd[:2]
        a_ddot = cmd[2]

        # ---- heading control  (47)-(52) --------------------------------------
        phi = heading_from_quaternion(q)
        h = heading_coupling(q)
        w_xy = w[:2]
        phi_dot = w[2] - h @ w_xy                                  # (24)
        z_phi = wrap_pi(phi - ref.yaw)
        z_phi_d = phi_dot - ref.yaw_rate
        omega_zd = ref.yaw_rate - G.k_phi * z_phi + h @ w_xy       # (48)
        # d/dt h along the quaternion kinematics q_dot = 1/2 q o (0, w)   (4)
        eps = self.h_eps
        q_dot = 0.5 * quat_mul(q, (0.0, w[0], w[1], w[2]))
        q_eps = q + eps * q_dot
        h_dot = (heading_coupling(q_eps / np.linalg.norm(q_eps)) - h) / eps
        omega_zd_dot = ref.yaw_acc - G.k_phi * z_phi_d + h_dot @ w_xy + h @ tau_xy
        z_wz = w[2] - omega_zd
        tau_z = omega_zd_dot - G.k_wz * z_wz - z_phi               # (52)

        tau_tilde = np.array([tau_xy[0], tau_xy[1], tau_z])
        torque = np.cross(w, self.J @ w) + self.J @ tau_tilde      # (29)
        if self.max_torque is not None:
            torque = np.clip(torque, -self.max_torque, self.max_torque)

        # ---- dynamic extension  T_ddot = u_T  (31) ---------------------------
        u_T = m * a_ddot
        self.T_dot += u_T * dt
        self.T += self.T_dot * dt
        if self.T > self.T_max:
            self.T, self.T_dot = self.T_max, min(self.T_dot, 0.0)
        elif self.T < self.T_min:
            self.T, self.T_dot = self.T_min, max(self.T_dot, 0.0)

        self.last = {
            "z_x": z_x, "z_v": z_v, "z_t": z_t, "z_r": z_r, "z_phi": z_phi, "z_wz": z_wz,
            "omega_d": np.array([omega_xd, omega_yd, omega_zd]), "thrust_dot": self.T_dot,
            "heading": phi, "thrust_vector_desired": m * t_d,
            "lyapunov": 0.5 * (z_x @ z_x + z_v @ z_v + z_t @ z_t + z_r @ z_r) + 0.5 * (z_phi ** 2 + z_wz ** 2),
        }
        return float(self.T), (float(torque[0]), float(torque[1]), float(torque[2]))

    def model_acceleration(self, quaternion, velocity=None) -> Tuple[float, float, float]:
        """Acceleration predicted from the currently applied thrust: g + R(q) e3 T / m + f(v).
        Used as the known input u of the Kalman filter prediction step."""
        b3 = quat_to_rot(quaternion)[:, 2]
        a = b3 * self.T / self.mass
        if velocity is not None and self.drag:
            a = a - self.drag * np.asarray(velocity, dtype=float)
        return (float(a[0]), float(a[1]), float(a[2] - self.g))

    # Convenience wrapper with the old API (ControllerReference) ---------------
    def compute_command(self, state: ControllerState, reference, dt: float):
        if isinstance(reference, TrajectoryPoint):
            return self.compute(state, reference, dt)
        ref = TrajectoryPoint(pos=tuple(reference.position), vel=tuple(reference.velocity),
                              acc=tuple(reference.acceleration), yaw=float(reference.heading))
        return self.compute(state, ref, dt)
