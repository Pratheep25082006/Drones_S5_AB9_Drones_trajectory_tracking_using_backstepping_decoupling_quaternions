"""
Mock ArduCopter (stand-in for ArduPilot SITL when it is not installed).

Behaves like ArduCopter towards a ground station / companion computer:
* rigid body of config vehicle.* (quadrotor_controller.RigidBodyQuadrotor), NED/FRD on the wire
* HEARTBEAT (1 Hz, mode + armed flag), ATTITUDE and LOCAL_POSITION_NED (rate settable with
  MAV_CMD_SET_MESSAGE_INTERVAL); no position before the "EKF" is ready (ekf_ready_after)
* parameters: PARAM_REQUEST_READ / PARAM_REQUEST_LIST / PARAM_SET -> PARAM_VALUE
* modes STABILIZE / GUIDED / LAND (SET_MODE or MAV_CMD_DO_SET_MODE), COMMAND_ACK for commands
* arming with pre-arm checks (position estimate, armable mode), force arm (param2 = 21196)
* GUIDED: SET_ATTITUDE_TARGET body rates + thrust (GUID_OPTIONS bit 3) tracked by a P rate loop
* LAND: levels, descends at land_descent_rate and disarms after touchdown
* STATUSTEXT messages like the real autopilot

Usage:
    python mock_mavlink_vehicle.py                 # output: config mock_vehicle.out
    python run_ardupilot_sitl.py --connect         # input:  config mavlink.connection
Settings: config.yaml -> mock_vehicle, vehicle.
"""

import argparse
import math
import os
import sys
import time

import numpy as np
from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller import RigidBodyQuadrotor, quat_to_rot  # noqa: E402
from quadrotor_controller.config import add_config_arguments, apply_config_arguments, cfg  # noqa: E402
from quadrotor_controller.units import (  # noqa: E402
    GUID_OPTIONS_THRUST_BIT, MAV_FORCE_ARM_MAGIC, MS_PER_S, US_PER_S)

P = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])   # ENU <-> NED (own inverse)
Q = np.diag([1.0, -1.0, -1.0])                                         # FLU <-> FRD (own inverse)
ml = mavutil.mavlink


class MockCopter:
    def __init__(self, out: str):
        mc = "mock_vehicle."
        self.mav = mavutil.mavlink_connection(out, source_system=1, source_component=1)
        self.body = RigidBodyQuadrotor(position=cfg(mc + "start"), ground_z=cfg(mc + "ground_z"))
        self.hover = self.body.m * self.body.g
        self.params = {k: float(v) for k, v in cfg(mc + "params").items()}
        self.modes = dict(cfg(mc + "modes"))
        self.mode = cfg(mc + "initial_mode")
        self.armed = False
        self.rate_cmd = np.zeros(3)
        self.thrust = 0.0
        self.publish = cfg(mc + "publish_period")
        self.ekf_ready = False
        self.warned_thrust = False
        self.t = 0.0

    # ------------------------------------------------------------------ helpers
    def text(self, s: str, severity=ml.MAV_SEVERITY_INFO) -> None:
        print("[MOCK]", s)
        self.mav.mav.statustext_send(severity, s.encode())

    def ack(self, command: int, result: int) -> None:
        self.mav.mav.command_ack_send(command, result)

    def param_value(self, name: str) -> None:
        names = list(self.params)
        self.mav.mav.param_value_send(name.encode(), self.params[name], ml.MAV_PARAM_TYPE_REAL32,
                                      len(names), names.index(name))

    def set_mode(self, custom_mode: int) -> bool:
        for name, num in self.modes.items():
            if num == custom_mode:
                if name != self.mode:
                    self.mode = name
                    self.text(f"Mode {name}")
                return True
        return False

    def arm(self, arm: bool, force: bool) -> int:
        if arm == self.armed:
            return ml.MAV_RESULT_ACCEPTED
        if arm and not force:
            if self.params.get("ARMING_CHECK", 0) and not self.ekf_ready:
                self.text("PreArm: Need Position Estimate", ml.MAV_SEVERITY_CRITICAL)
                return ml.MAV_RESULT_FAILED
            if self.mode not in cfg("mock_vehicle.armable_modes"):
                self.text(f"Arm: Mode not armable ({self.mode})", ml.MAV_SEVERITY_CRITICAL)
                return ml.MAV_RESULT_FAILED
        self.armed = arm
        self.rate_cmd[:] = 0.0
        self.thrust = 0.0
        self.text("Arming motors" if arm else "Disarming motors")
        return ml.MAV_RESULT_ACCEPTED

    # ------------------------------------------------------------------ input
    def handle(self, msg) -> None:
        typ = msg.get_type()
        if typ == "SET_ATTITUDE_TARGET":
            if self.mode != cfg("mavlink.guided_mode") or not self.armed:
                return
            self.rate_cmd = Q @ np.array([msg.body_roll_rate, msg.body_pitch_rate, msg.body_yaw_rate])
            if int(self.params.get("GUID_OPTIONS", 0)) & GUID_OPTIONS_THRUST_BIT:
                self.thrust = msg.thrust / self.params["MOT_THST_HOVER"] * self.hover
            else:
                if not self.warned_thrust:
                    self.text("Guided: thrust field read as climb rate (GUID_OPTIONS bit 3 not set)", ml.MAV_SEVERITY_WARNING)
                    self.warned_thrust = True
                self.thrust = self.hover
        elif typ == "SET_MODE":
            self.set_mode(msg.custom_mode)
        elif typ == "COMMAND_LONG":
            c = msg.command
            if c == ml.MAV_CMD_COMPONENT_ARM_DISARM:
                self.ack(c, self.arm(msg.param1 > 0.5, int(msg.param2) == MAV_FORCE_ARM_MAGIC))
            elif c == ml.MAV_CMD_DO_SET_MODE:
                self.ack(c, ml.MAV_RESULT_ACCEPTED if self.set_mode(int(msg.param2)) else ml.MAV_RESULT_DENIED)
            elif c == ml.MAV_CMD_SET_MESSAGE_INTERVAL:
                if msg.param2 > 0:
                    self.publish = msg.param2 / US_PER_S
                self.ack(c, ml.MAV_RESULT_ACCEPTED)
            else:
                self.ack(c, ml.MAV_RESULT_UNSUPPORTED)
        elif typ == "PARAM_REQUEST_READ":
            if msg.param_id in self.params:
                self.param_value(msg.param_id)
        elif typ == "PARAM_REQUEST_LIST":
            for name in self.params:
                self.param_value(name)
        elif typ == "PARAM_SET":
            if msg.param_id in self.params:
                self.params[msg.param_id] = float(msg.param_value)
                self.param_value(msg.param_id)

    # ------------------------------------------------------------------ physics
    def land_control(self) -> None:
        """LAND: level the vehicle, hold heading, descend at land_descent_rate."""
        mc = "mock_vehicle."
        b = self.body
        qv = np.asarray(b.q[1:]) * (1.0 if b.q[0] >= 0 else -1.0)
        rate = -cfg(mc + "land_level_gain") * 2.0 * qv
        rate[2] = 0.0
        self.rate_cmd = rate
        acc = cfg(mc + "land_velocity_gain") * (-cfg(mc + "land_descent_rate") - b.v[2])
        self.thrust = b.m * (b.g + acc)
        if b.x[2] - cfg(mc + "ground_z") < cfg(mc + "disarm_height"):
            self.text("Land complete")
            self.arm(False, True)

    def step(self, dt: float) -> None:
        if self.armed:
            if self.mode == cfg("mavlink.land_mode"):
                self.land_control()
            b = self.body
            torque = b.J @ (cfg("mock_vehicle.rate_gain") * (self.rate_cmd - b.w)) + np.cross(b.w, b.J @ b.w)
            b.step(max(0.0, self.thrust), torque, dt)

    def send_state(self) -> None:
        ms = int(self.t * MS_PER_S)
        b = self.body
        if self.ekf_ready:
            self.mav.mav.local_position_ned_send(ms, *(P @ b.x), *(P @ b.v))
        R_frd = P @ quat_to_rot(b.q) @ Q
        roll = math.atan2(R_frd[2, 1], R_frd[2, 2])
        pitch = -math.asin(max(-1.0, min(1.0, R_frd[2, 0])))
        yaw = math.atan2(R_frd[1, 0], R_frd[0, 0])
        self.mav.mav.attitude_send(ms, roll, pitch, yaw, *(Q @ b.w))

    def send_heartbeat(self) -> None:
        base = ml.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (ml.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
        state = ml.MAV_STATE_ACTIVE if self.armed else ml.MAV_STATE_STANDBY
        self.mav.mav.heartbeat_send(ml.MAV_TYPE_QUADROTOR, ml.MAV_AUTOPILOT_ARDUPILOTMEGA, base,
                                    self.modes[self.mode], state)

    def run(self, duration: float) -> None:
        mc = "mock_vehicle."
        dt, min_sleep = cfg(mc + "physics_dt"), cfg(mc + "realtime_min_sleep")
        hb_period, ekf_after = cfg(mc + "heartbeat_period"), cfg(mc + "ekf_ready_after")
        last_pub = last_hb = -math.inf
        t_wall = time.time()
        while duration <= 0 or self.t < duration:
            while True:
                msg = self.mav.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() != "BAD_DATA":
                    self.handle(msg)
            if not self.ekf_ready and self.t >= ekf_after:
                self.ekf_ready = True
                self.text("EKF3 IMU0 is using GPS")
            self.step(dt)
            self.t += dt
            if self.t - last_hb >= hb_period:
                last_hb = self.t
                self.send_heartbeat()
            if self.t - last_pub >= self.publish:
                last_pub = self.t
                self.send_state()
            lag = self.t - (time.time() - t_wall)              # real-time pacing
            if lag > min_sleep:
                time.sleep(lag)


def main():
    ap = argparse.ArgumentParser(description="Mock ArduCopter over MAVLink")
    ap.add_argument("--out", default=None, help="MAVLink output (default mock_vehicle.out)")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds (0 = run until Ctrl+C)")
    add_config_arguments(ap)
    args = ap.parse_args()
    apply_config_arguments(args)
    out = args.out or cfg("mock_vehicle.out")
    print("[MOCK] ArduCopter stand-in running ->", out)
    try:
        MockCopter(out).run(args.duration)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
