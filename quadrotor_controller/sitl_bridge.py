from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass
from math import cos, sin
from subprocess import Popen
from typing import Dict, Optional, Tuple

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

    def connect(self, timeout: float = 10.0) -> None:
        """Connect to ArduPilot / Gazebo SITL via MAVLink or mock mode."""
        if self.connection_string and HAS_PYMAVLINK:
            self.mav_connection = mavutil.mavlink_connection(self.connection_string)
            print(f"Connecting to MAVLink on {self.connection_string}...")
            self.mav_connection.wait_heartbeat(timeout=timeout)
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
            self.simulator_process.wait(timeout=5)
        except Exception:
            self.simulator_process.kill()
            self.simulator_process.wait()
        finally:
            self.simulator_process = None

    def set_mode(self, mode_name: str = "GUIDED") -> None:
        """Set vehicle flight mode (e.g. GUIDED or STABILIZE)."""
        if self.mav_connection is not None and HAS_PYMAVLINK:
            mode_id = self.mav_connection.mode_mapping().get(mode_name)
            if mode_id is not None:
                self.mav_connection.mav.set_mode_send(
                    self.mav_connection.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id,
                )

    def arm_vehicle(self, arm: bool = True) -> None:
        """Arm or disarm the vehicle."""
        if self.mav_connection is not None and HAS_PYMAVLINK:
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1 if arm else 0,
                0, 0, 0, 0, 0, 0,
            )

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
            # MAVLink SET_ATTITUDE_TARGET expects thrust normalized 0..1
            norm_thrust = min(1.0, max(0.0, target["thrust"] / 30.0))
            self.mav_connection.mav.set_attitude_target_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                0,  # Bitmask: use attitude and rates
                q,
                target.get("roll_rate", 0.0),
                target.get("pitch_rate", 0.0),
                target["yaw_rate"],
                norm_thrust,
            )

