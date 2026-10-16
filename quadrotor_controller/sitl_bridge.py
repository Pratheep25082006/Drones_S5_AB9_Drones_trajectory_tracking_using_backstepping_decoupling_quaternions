from __future__ import annotations

import math
import subprocess
import time
from dataclasses import dataclass
from math import cos, sin
from subprocess import Popen
from typing import Dict, Optional, Tuple

from .config import cfg

from .units import COMMAND_LONG_PARAMS, MAV_FORCE_ARM_MAGIC, MS_PER_S, UINT32_MASK, US_PER_S  # noqa: F401  (re-exported)

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False


@dataclass
class VehicleState:
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]
    angular_rate: Tuple[float, float, float]


class SITLBridge:
    def __init__(self, connection_string: str | None = None) -> None:
        self.connected = False
        self.connection_string = connection_string
        self.mav_connection = None
        self.last_attitude_target: Dict[str, float] | None = None
        self.simulator_process: Optional[Popen] = None

    def connect(self, timeout: Optional[float] = None) -> None:
        """Connect to ArduPilot / Gazebo SITL via MAVLink or mock mode."""
        if self.connection_string and HAS_PYMAVLINK:
            self.mav_connection = mavutil.mavlink_connection(self.connection_string)
            print(f"Connecting to MAVLink on {self.connection_string}...")
            self.mav_connection.wait_heartbeat(timeout=cfg("mavlink.connect_timeout") if timeout is None else timeout)
            print("Heartbeat received from vehicle (system %u component %u)" %
                  (self.mav_connection.target_system, self.mav_connection.target_component))
        self.connected = True

    def disconnect(self) -> None:
        """Mark the bridge as disconnected and stop any launched simulator."""
        if self.mav_connection is not None:
            try:
                self.mav_connection.close()
            except Exception:
                pass
            self.mav_connection = None
        self.connected = False
        self.stop_simulator()

    def is_connected(self) -> bool:
        return self.connected

    def launch_simulator(self, command: list[str], cwd: str | None = None, wait: bool = False) -> None:
        """Launch an external simulator process such as Gazebo or ArduPilot SITL."""
        if self.simulator_process is not None:
            raise RuntimeError("Simulator already launched")
        self.simulator_process = subprocess.Popen(command, cwd=cwd)
        if wait:
            self.simulator_process.wait()

    def stop_simulator(self) -> None:
        """Stop a previously launched simulator process."""
        if self.simulator_process is None:
            return
        self.simulator_process.terminate()
        try:
            self.simulator_process.wait(timeout=cfg("mavlink.connect_timeout"))
        except Exception:
            self.simulator_process.kill()
            self.simulator_process.wait()
        finally:
            self.simulator_process = None

    # ------------------------------------------------------------------ pre-flight sequence
    # request streams -> parameters -> EKF position -> GUIDED -> arm  (preflight)
    # ... control loop (send_rates_thrust) ...  ->  LAND and wait for disarm (land)

    def _is_vehicle(self, msg) -> bool:
        return (msg.get_srcSystem() == self.mav_connection.target_system
                and msg.type != mavutil.mavlink.MAV_TYPE_GCS
                and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID)

    def _track(self, msg) -> None:
        """Book-keeping for every received message (state cache, vehicle heartbeat, status texts)."""
        t = msg.get_type()
        if not hasattr(self, "_ned"):
            self._ned = {"pos": [0.0, 0.0, 0.0], "vel": [0.0, 0.0, 0.0], "att": [0.0, 0.0, 0.0], "rates": [0.0, 0.0, 0.0]}
        if t == "LOCAL_POSITION_NED":
            self._ned["pos"] = [msg.x, msg.y, msg.z]
            self._ned["vel"] = [msg.vx, msg.vy, msg.vz]
            self.have_position = True
        elif t == "ATTITUDE":
            self._ned["att"] = [msg.roll, msg.pitch, msg.yaw]
            self._ned["rates"] = [msg.rollspeed, msg.pitchspeed, msg.yawspeed]
        elif t == "HEARTBEAT" and self._is_vehicle(msg):
            self.vehicle_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.vehicle_custom_mode = msg.custom_mode
        elif t == "STATUSTEXT":
            print(f"[AUTOPILOT] {msg.text}")

    def _wait(self, types, timeout: float, accept=None):
        """Wait for a message of one of ``types`` (accepted by ``accept``); None on timeout."""
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            msg = self.mav_connection.recv_match(blocking=True, timeout=min(remaining, cfg("mavlink.poll_period")))
            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            self._track(msg)
            if msg.get_type() in types and (accept is None or accept(msg)):
                return msg

    def _command(self, command: int, params, timeout: Optional[float] = None) -> Optional[int]:
        """COMMAND_LONG and its COMMAND_ACK result (None = no answer)."""
        c = self.mav_connection
        p = list(params) + [0.0] * (COMMAND_LONG_PARAMS - len(params))
        c.mav.command_long_send(c.target_system, c.target_component, command, 0, *p)
        ack = self._wait(("COMMAND_ACK",), cfg("mavlink.param_timeout") if timeout is None else timeout,
                         lambda m: m.command == command)
        return None if ack is None else ack.result

    def request_streams(self, rate_hz: Optional[float] = None) -> None:
        """SET_MESSAGE_INTERVAL for the state messages the controller needs."""
        if self.mav_connection is None:
            return
        rate_hz = cfg("mavlink.stream_rate_hz") if rate_hz is None else rate_hz
        for name in cfg("mavlink.stream_messages"):
            msg_id = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_" + name)
            res = self._command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, (msg_id, US_PER_S / rate_hz))
            ok = res == mavutil.mavlink.MAV_RESULT_ACCEPTED
            print(f"[MAVLINK] stream {name} @ {rate_hz:g} Hz: {'ok' if ok else 'no ACK / refused (using default rate)'}")

    def get_param(self, name: str, timeout: Optional[float] = None) -> Optional[float]:
        if self.mav_connection is None:
            return None
        c = self.mav_connection
        timeout = cfg("mavlink.param_timeout") if timeout is None else timeout
        for _ in range(cfg("mavlink.param_retries")):
            c.mav.param_request_read_send(c.target_system, c.target_component, name.encode(), -1)
            m = self._wait(("PARAM_VALUE",), timeout, lambda m: m.param_id == name)
            if m is not None:
                return float(m.param_value)
        return None

    def set_param(self, name: str, value: float, timeout: Optional[float] = None) -> bool:
        if self.mav_connection is None:
            return False
        c = self.mav_connection
        timeout = cfg("mavlink.param_timeout") if timeout is None else timeout
        tol = cfg("mavlink.param_tolerance")
        for _ in range(cfg("mavlink.param_retries")):
            c.mav.param_set_send(c.target_system, c.target_component, name.encode(), float(value),
                                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            m = self._wait(("PARAM_VALUE",), timeout, lambda m: m.param_id == name)
            if m is not None:
                ok = abs(m.param_value - value) <= tol
                print(f"[MAVLINK] param {name} = {m.param_value:g}" + ("" if ok else f" (requested {value:g})"))
                return ok
        print(f"[MAVLINK] param {name}: no answer")
        return False

    def resolve_hover_throttle(self) -> float:
        """mavlink.hover_throttle, or the autopilot's MOT_THST_HOVER when it is 'auto'."""
        v = cfg("mavlink.hover_throttle")
        if v == "auto":
            got = self.get_param(cfg("mavlink.hover_throttle_param"))
            v = got if got else cfg("mavlink.hover_throttle_fallback")
            print(f"[MAVLINK] hover throttle = {v:.3f} " + ("(from autopilot)" if got else "(fallback)"))
        self.hover_throttle = float(v)
        return self.hover_throttle

    def wait_position(self, timeout: Optional[float] = None) -> bool:
        """Wait until the autopilot publishes a local position (EKF ready, origin set)."""
        if self.mav_connection is None:
            return True
        timeout = cfg("mavlink.position_timeout") if timeout is None else timeout
        print("[MAVLINK] waiting for the position estimate (EKF) ...")
        ok = getattr(self, "have_position", False) or self._wait(("LOCAL_POSITION_NED",), timeout) is not None
        print("[MAVLINK] position estimate ok" if ok else "[MAVLINK] no position estimate")
        return ok

    def set_mode(self, mode_name: Optional[str] = None, timeout: Optional[float] = None) -> bool:
        """Request a flight mode and wait for the heartbeat to confirm it."""
        mode_name = mode_name or cfg("mavlink.guided_mode")
        if self.mav_connection is None or not HAS_PYMAVLINK:
            return True
        c = self.mav_connection
        mapping = c.mode_mapping() or {}
        if mode_name not in mapping:
            raise ValueError(f"mode {mode_name!r} not known for this vehicle ({sorted(mapping)})")
        mode_id = mapping[mode_name]
        timeout = cfg("mavlink.mode_timeout") if timeout is None else timeout
        deadline = time.time() + timeout
        while time.time() < deadline:
            c.mav.set_mode_send(c.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
            hb = self._wait(("HEARTBEAT",), min(cfg("mavlink.resend_period"), max(0.0, deadline - time.time())),
                            lambda m: self._is_vehicle(m) and m.custom_mode == mode_id)
            if hb is not None:
                print(f"[MAVLINK] mode {mode_name}")
                return True
        print(f"[MAVLINK] mode {mode_name} not confirmed")
        return False

    def arm_vehicle(self, arm: bool = True, force: Optional[bool] = None, timeout: Optional[float] = None) -> bool:
        """Arm / disarm and wait for the heartbeat; pre-arm failures are printed from STATUSTEXT."""
        if self.mav_connection is None or not HAS_PYMAVLINK:
            return True
        force = cfg("mavlink.force_arm") if force is None else force
        timeout = cfg("mavlink.arm_timeout") if timeout is None else timeout
        deadline = time.time() + timeout
        ml = mavutil.mavlink
        while time.time() < deadline:
            res = self._command(ml.MAV_CMD_COMPONENT_ARM_DISARM, (1 if arm else 0, MAV_FORCE_ARM_MAGIC if force else 0),
                                cfg("mavlink.resend_period"))
            if res == ml.MAV_RESULT_ACCEPTED:
                hb = self._wait(("HEARTBEAT",), max(0.0, deadline - time.time()),
                                lambda m: self._is_vehicle(m) and bool(m.base_mode & ml.MAV_MODE_FLAG_SAFETY_ARMED) == arm)
                if hb is not None:
                    print("[MAVLINK] armed" if arm else "[MAVLINK] disarmed")
                    return True
            elif res is not None:
                print(f"[MAVLINK] {'arm' if arm else 'disarm'} refused (result {res}); retrying ...")
            time.sleep(cfg("mavlink.poll_period"))
        print(f"[MAVLINK] {'arming' if arm else 'disarming'} failed")
        return False

    def preflight(self) -> None:
        """Streams, parameters, EKF, GUIDED mode and arming; raises RuntimeError when a step fails."""
        if self.mav_connection is None:
            return
        self.request_streams()
        self.resolve_hover_throttle()
        for name, value in cfg("mavlink.params").items():
            if not self.set_param(name, value):
                raise RuntimeError(f"could not set parameter {name}")
        if not self.wait_position():
            raise RuntimeError("no position estimate from the autopilot")
        if not self.set_mode(cfg("mavlink.guided_mode")):
            raise RuntimeError("could not switch to " + cfg("mavlink.guided_mode"))
        if not self.arm_vehicle(True):
            raise RuntimeError("could not arm")

    def send_gcs_heartbeat(self) -> None:
        """1 Hz GCS heartbeat (keeps the link 'alive' for the autopilot's GCS failsafe)."""
        if self.mav_connection is None:
            return
        now = time.time()
        if now - getattr(self, "_last_gcs_hb", -math.inf) >= cfg("mavlink.gcs_heartbeat_period"):
            self._last_gcs_hb = now
            self.mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def in_control(self) -> bool:
        """False when the pilot / a failsafe has changed the mode or the vehicle disarmed."""
        if self.mav_connection is None:
            return True
        guided = (self.mav_connection.mode_mapping() or {}).get(cfg("mavlink.guided_mode"))
        return getattr(self, "vehicle_armed", True) and getattr(self, "vehicle_custom_mode", guided) == guided

    def land(self, timeout: Optional[float] = None) -> bool:
        """Switch to LAND and wait until the autopilot disarms after touchdown."""
        if self.mav_connection is None:
            return True
        timeout = cfg("mavlink.land_timeout") if timeout is None else timeout
        if not self.set_mode(cfg("mavlink.land_mode")):
            return False
        print("[MAVLINK] landing ...")
        if not getattr(self, "vehicle_armed", True):
            return True
        hb = self._wait(("HEARTBEAT",), timeout,
                        lambda m: self._is_vehicle(m) and not (m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
        print("[MAVLINK] landed and disarmed" if hb is not None else "[MAVLINK] landing not confirmed")
        return hb is not None

    def parse_state(
        self,
        local_position: Tuple[float, float, float],
        velocity: Tuple[float, float, float],
        attitude: Tuple[float, float, float],
        angular_rate: Tuple[float, float, float],
    ) -> VehicleState:
        roll, pitch, yaw = attitude
        cr = cos(roll / 2.0)
        sr = sin(roll / 2.0)
        cp = cos(pitch / 2.0)
        sp = sin(pitch / 2.0)
        cy = cos(yaw / 2.0)
        sy = sin(yaw / 2.0)

        quaternion = (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
        return VehicleState(
            position=local_position,
            velocity=velocity,
            quaternion=quaternion,
            angular_rate=angular_rate,
        )

    def receive_mavlink_state(self, current_state: VehicleState) -> VehicleState:
        """Poll incoming MAVLink packets (ATTITUDE, LOCAL_POSITION_NED) and update VehicleState."""
        if self.mav_connection is None or not HAS_PYMAVLINK:
            return current_state


        pos = list(current_state.position)
        vel = list(current_state.velocity)
        quat = current_state.quaternion
        rates = list(current_state.angular_rate)

        while True:
            msg = self.mav_connection.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == "LOCAL_POSITION_NED":
                pos = [msg.x, msg.y, msg.z]
                vel = [msg.vx, msg.vy, msg.vz]
            elif msg_type == "ATTITUDE":
                roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw
                rates = [msg.rollspeed, msg.pitchspeed, msg.yawspeed]
                cr, sr = cos(roll / 2.0), sin(roll / 2.0)
                cp, sp = cos(pitch / 2.0), sin(pitch / 2.0)
                cy, sy = cos(yaw / 2.0), sin(yaw / 2.0)
                quat = (
                    cr * cp * cy + sr * sp * sy,
                    sr * cp * cy - cr * sp * sy,
                    cr * sp * cy + sr * cp * sy,
                    cr * cp * sy - sr * sp * cy,
                )

        return VehicleState(
            position=(pos[0], pos[1], pos[2]),
            velocity=(vel[0], vel[1], vel[2]),
            quaternion=quat,
            angular_rate=(rates[0], rates[1], rates[2]),
        )

    def build_attitude_target(
        self,
        thrust: float,
        roll: float,
        pitch: float,
        yaw: float,
        yaw_rate: float,
    ) -> Dict[str, float]:
        return {
            "thrust": thrust,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "yaw_rate": yaw_rate,
        }

    def send_attitude_target(self, target: Dict[str, float], quaternion: Tuple[float, float, float, float] | None = None) -> None:
        """Send MAVLink SET_ATTITUDE_TARGET message to vehicle."""
        if not self.connected:
            raise RuntimeError("SITLBridge is not connected")
        self.last_attitude_target = target

        if self.mav_connection is not None and HAS_PYMAVLINK:
            q = quaternion if quaternion is not None else [1.0, 0.0, 0.0, 0.0]
            # MAVLink SET_ATTITUDE_TARGET expects thrust normalised 0..1 (hover -> MOT_THST_HOVER)
            hover = cfg("vehicle.mass") * cfg("vehicle.gravity")
            norm_thrust = min(1.0, max(0.0, self.current_hover_throttle() * target["thrust"] / hover))
            self.mav_connection.mav.set_attitude_target_send(
                int(time.time() * MS_PER_S) & UINT32_MASK,
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                0,  # Bitmask: use attitude and rates
                q,
                target.get("roll_rate", 0.0),
                target.get("pitch_rate", 0.0),
                target["yaw_rate"],
                norm_thrust,
            )



# ---------------------------------------------------------------------------
# Frame conversion helpers (ArduPilot/MAVLink: NED world, FRD body;
# this project: ENU world, FLU body)
# ---------------------------------------------------------------------------
import numpy as _np

_P_NED_TO_ENU = _np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_Q_FLU_FRD = _np.diag([1.0, -1.0, -1.0])


def _rot_from_euler_zyx(roll: float, pitch: float, yaw: float) -> _np.ndarray:
    cr, sr, cp, sp, cy, sy = cos(roll), sin(roll), cos(pitch), sin(pitch), cos(yaw), sin(yaw)
    return _np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp],
    ])


def _quat_from_rot(R: _np.ndarray) -> Tuple[float, float, float, float]:
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * _np.sqrt(tr + 1.0)
        q = (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * _np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * _np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    else:
        s = 2.0 * _np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)
    q = _np.asarray(q)
    if q[0] < 0:
        q = -q
    q = q / _np.linalg.norm(q)
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def ned_frd_to_enu_flu(position_ned, velocity_ned, roll, pitch, yaw, rates_frd) -> VehicleState:
    """Convert MAVLink LOCAL_POSITION_NED + ATTITUDE into the project's ENU/FLU VehicleState."""
    P, Q = _P_NED_TO_ENU, _Q_FLU_FRD
    R_ned = _rot_from_euler_zyx(roll, pitch, yaw)          # FRD body -> NED
    R_enu = P @ R_ned @ Q                                   # FLU body -> ENU
    pos = P @ _np.asarray(position_ned, dtype=float)
    vel = P @ _np.asarray(velocity_ned, dtype=float)
    w = Q @ _np.asarray(rates_frd, dtype=float)
    return VehicleState(position=tuple(float(v) for v in pos), velocity=tuple(float(v) for v in vel),
                        quaternion=_quat_from_rot(R_enu), angular_rate=tuple(float(v) for v in w))


def flu_rates_to_frd(rates_flu) -> Tuple[float, float, float]:
    w = _Q_FLU_FRD @ _np.asarray(rates_flu, dtype=float)
    return (float(w[0]), float(w[1]), float(w[2]))


def _receive_enu_state(self: "SITLBridge", current_state: VehicleState) -> VehicleState:
    """Poll LOCAL_POSITION_NED / ATTITUDE and return the state in ENU/FLU."""
    if self.mav_connection is None or not HAS_PYMAVLINK:
        return current_state
    got = False
    while True:
        msg = self.mav_connection.recv_match(blocking=False)
        if msg is None:
            break
        self._track(msg)
        got = got or msg.get_type() in ("LOCAL_POSITION_NED", "ATTITUDE")
    if not hasattr(self, "_ned"):
        return current_state
    if not got and current_state is not None and hasattr(self, "_enu_cache"):
        return self._enu_cache
    s = ned_frd_to_enu_flu(self._ned["pos"], self._ned["vel"], *self._ned["att"], self._ned["rates"])
    self._enu_cache = s
    return s


def _send_rates_thrust(self: "SITLBridge", rates_flu, thrust_newton: float, hover_thrust_newton: float,
                       hover_throttle: Optional[float] = None) -> None:
    """SET_ATTITUDE_TARGET with body rates (FRD) + normalised thrust, attitude ignored.

    The paper controller's angular-velocity reference (omega_xyd, omega_zd) and its
    thrust T are sent; ArduPilot's rate loop tracks them.  Thrust is scaled so that
    hover thrust maps to ``hover_throttle`` (ArduPilot MOT_THST_HOVER).  In ArduPilot
    set GUID_OPTIONS bit 3 so the value is interpreted as thrust, not climb rate.
    """
    if not self.connected:
        raise RuntimeError("SITLBridge is not connected")
    if hover_throttle is None:
        hover_throttle = self.current_hover_throttle()
    norm = max(0.0, min(1.0, hover_throttle * thrust_newton / hover_thrust_newton))
    w = flu_rates_to_frd(rates_flu)
    self.last_attitude_target = {"thrust": norm, "roll_rate": w[0], "pitch_rate": w[1], "yaw_rate": w[2]}
    if self.mav_connection is not None and HAS_PYMAVLINK:
        IGNORE_ATTITUDE = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
        self.mav_connection.mav.set_attitude_target_send(
            int(time.time() * MS_PER_S) & UINT32_MASK,
            self.mav_connection.target_system, self.mav_connection.target_component,
            IGNORE_ATTITUDE, [1.0, 0.0, 0.0, 0.0], w[0], w[1], w[2], norm)


SITLBridge.receive_enu_state = _receive_enu_state
SITLBridge.send_rates_thrust = _send_rates_thrust


def _current_hover_throttle(self: "SITLBridge") -> float:
    """Throttle that holds hover: resolved value, else mavlink.hover_throttle (fallback for 'auto')."""
    v = getattr(self, "hover_throttle", None)
    if v is None:
        v = cfg("mavlink.hover_throttle")
        if v == "auto":
            v = cfg("mavlink.hover_throttle_fallback")
    return float(v)


SITLBridge.current_hover_throttle = _current_hover_throttle
