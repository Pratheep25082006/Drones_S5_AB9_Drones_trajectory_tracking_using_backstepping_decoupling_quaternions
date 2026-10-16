"""
ArduPilot SITL / MAVLink run with the De Monte & Lohmann (2013) backstepping controller.

The controller runs on this PC (companion-computer style) on the vehicle state received
over MAVLink (converted NED/FRD -> ENU/FLU) and sends its body-rate reference
(omega_xd, omega_yd, omega_zd) and thrust with SET_ATTITUDE_TARGET; ArduPilot's own
rate loop tracks them.

Sequence (like a ground-control station):
  1. start / connect to the vehicle  (mavlink.launch: none | dronekit | command)
  2. request ATTITUDE + LOCAL_POSITION_NED at mavlink.stream_rate_hz
  3. read MOT_THST_HOVER (hover_throttle: auto) and set mavlink.params (GUID_OPTIONS bit 3)
  4. wait for the EKF position estimate, switch to GUIDED, arm (pre-arm messages are printed)
  5. fly to the target with the paper controller (stops if the mode changes / vehicle disarms)
  6. LAND and wait for the autopilot to disarm (mavlink.land_after)
Settings: config.yaml -> mavlink, vehicle, controller.

Usage:
    python mock_mavlink_vehicle.py        # (other terminal) stand-in when SITL is not installed
    python run_ardupilot_sitl.py                                    # mavlink.launch decides
    python run_ardupilot_sitl.py --connect                          # existing SITL / mock (mavlink.connection)
    python run_ardupilot_sitl.py --connect udpin:HOST:PORT --target 1 1 2
    python run_ardupilot_sitl.py --launch dronekit                  # start ArduCopter with dronekit-sitl
"""

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller.units import CM_PER_M  # noqa: E402
from quadrotor_controller.config import add_config_arguments, apply_config_arguments, banner, cfg  # noqa: E402
from quadrotor_controller import (  # noqa: E402
    ControllerState,
    PaperBacksteppingController,
    ReferenceManager,
    SITLBridge,
    VehicleState,
)


def start_vehicle(args, bridge):
    """Returns (connection string, dronekit SITL handle or None)."""
    launch = args.launch or ("none" if args.connect is not None else cfg("mavlink.launch"))
    conn = args.connect or cfg("mavlink.connection")
    if launch == "dronekit":
        import dronekit_sitl
        print("Starting ArduPilot SITL (ArduCopter) with dronekit-sitl ...")
        sitl = dronekit_sitl.start_default()
        return (args.connect or sitl.connection_string()), sitl
    if launch == "command":
        print("Starting:", " ".join(cfg("mavlink.launch_command")))
        bridge.launch_simulator(cfg("mavlink.launch_command"), cwd=HERE)
    elif launch != "none":
        raise SystemExit(f"unknown mavlink.launch {launch!r} (none | dronekit | command)")
    return conn, None


def main():
    ap = argparse.ArgumentParser(description="ArduPilot SITL with the paper backstepping controller")
    ap.add_argument("--connect", nargs="?", const="", default=None,
                    help="connect to an existing vehicle (no value = config mavlink.connection)")
    ap.add_argument("--launch", choices=("none", "dronekit", "command"), default=None,
                    help="how to start SITL (default config mavlink.launch)")
    ap.add_argument("--target", type=float, nargs=3, default=None, help="set point (default mavlink.target)")
    ap.add_argument("--duration", type=float, default=None, help="seconds (default mavlink.duration)")
    ap.add_argument("--no-land", action="store_true", help="do not land at the end")
    add_config_arguments(ap)
    args = ap.parse_args()
    apply_config_arguments(args)
    target = tuple(args.target) if args.target else tuple(cfg("mavlink.target"))
    duration = cfg("mavlink.duration") if args.duration is None else args.duration
    dt = cfg("mavlink.control_dt")
    print_every = max(1, int(round(cfg("logging.print_period") / dt)))
    print(banner("ArduPilot / MAVLink  -  paper backstepping controller"))

    bridge = SITLBridge()
    sitl = None
    try:
        conn, sitl = start_vehicle(args, bridge)
        bridge.connection_string = conn
        print(f"MAVLink connection: {conn}")
        bridge.connect()
        bridge.preflight()
        ctl = PaperBacksteppingController()
        hover = ctl.mass * ctl.g
        state = VehicleState((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        time.sleep(cfg("mavlink.startup_wait"))
        state = bridge.receive_enu_state(state)
        refman = ReferenceManager(start_pos=state.position, target_path=None, initial_mode="hover")
        refman._activate("coords", 0.0, (target, 0.0))
        print(f"Start {tuple(round(v, 2) for v in state.position)} -> target {target}")
        t, step, worst = 0.0, 0, 0.0
        while t < duration:
            t0 = time.time()
            state = bridge.receive_enu_state(state)
            bridge.send_gcs_heartbeat()
            if not bridge.in_control():
                print("[MAVLINK] vehicle left GUIDED or disarmed - giving up control")
                break
            cs = ControllerState(state.position, state.velocity, state.quaternion, state.angular_rate)
            ref = refman.update(t, dt)
            thrust, tau = ctl.compute(cs, ref, dt)
            bridge.send_rates_thrust(ctl.last["omega_d"], thrust, hover)
            err = math.dist(state.position, ref.pos)
            if step % print_every == 0:
                print(f"t={t:5.1f}s pos=({state.position[0]:5.2f},{state.position[1]:5.2f},{state.position[2]:5.2f}) "
                      f"ref=({ref.pos[0]:5.2f},{ref.pos[1]:5.2f},{ref.pos[2]:5.2f}) err={err * CM_PER_M:5.1f}cm thrust={thrust:5.2f}N")
            t += dt
            step += 1
            rem = dt - (time.time() - t0)
            if rem > 0:
                time.sleep(rem)
        final = math.dist(state.position, target)
        print(f"Control run finished: final error to target {final * CM_PER_M:.1f} cm")
        if cfg("mavlink.land_after") and not args.no_land:
            bridge.land()
    except RuntimeError as e:
        print(f"[MAVLINK] pre-flight failed: {e}")
    finally:
        bridge.disconnect()
        if sitl is not None:
            sitl.stop()


if __name__ == "__main__":
    main()
