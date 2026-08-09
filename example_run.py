import argparse
import time
from typing import Tuple
from quadrotor_controller import (

    BacksteppingController,
    ControllerReference,
    ControllerState,
    SITLBridge,
    normalize_quaternion,
    quaternion_from_euler,
)


def update_state(
    state: ControllerState,
    thrust: float,
    torques: Tuple[float, float, float],
    dt: float,
    mass: float = 1.5,
    gravity: float = 9.81,
    inertia: Tuple[float, float, float] = (0.03, 0.03, 0.05),
) -> ControllerState:
    # 1. Rotational dynamics integration
    # Angular acceleration: alpha = torque / inertia
    alpha_x = torques[0] / inertia[0]
    alpha_y = torques[1] / inertia[1]
    alpha_z = torques[2] / inertia[2]

    wx = state.angular_rate[0] + alpha_x * dt
    wy = state.angular_rate[1] + alpha_y * dt
    wz = state.angular_rate[2] + alpha_z * dt

    # Quaternion derivative: q_dot = 0.5 * q * (0, wx, wy, wz)
    w, x, y, z = state.quaternion
    q_vec = (0.0, wx, wy, wz)
    q_dot = BacksteppingController._quaternion_multiply((w, x, y, z), q_vec)

    qw = w + 0.5 * q_dot[0] * dt
    qx = x + 0.5 * q_dot[1] * dt
    qy = y + 0.5 * q_dot[2] * dt
    qz = z + 0.5 * q_dot[3] * dt
    new_q = normalize_quaternion((qw, qx, qy, qz))

    # 2. Translational dynamics integration
    direction = BacksteppingController._thrust_direction(new_q)

    # Global acceleration (z-up ENU convention)
    ax = (thrust / mass) * direction[0]
    ay = (thrust / mass) * direction[1]
    az = (thrust / mass) * direction[2] - gravity

    vx = state.velocity[0] + ax * dt
    vy = state.velocity[1] + ay * dt
    vz = state.velocity[2] + az * dt

    px = state.position[0] + vx * dt
    py = state.position[1] + vy * dt
    pz = state.position[2] + vz * dt

    return ControllerState(
        position=(px, py, pz),
        velocity=(vx, vy, vz),
        quaternion=new_q,
        angular_rate=(wx, wy, wz),
    )




import math
import matplotlib.pyplot as plt


def get_reference(t: float, mode: str) -> ControllerReference:
    if mode == "helix":
        # Vertical helix tracking as in Paper Section IV (Fig. 4)
        # Period T_xy = 6.25 s, T_z = 12.5 s
        w_xy = 2.0 * math.pi / 6.25
        w_z = 2.0 * math.pi / 12.5
        
        px = 1.0 * math.sin(w_xy * t)
        py = 2.0 * math.sin(w_xy * t)
        pz = -1.0 + 0.5 * math.sin(w_z * t)
        
        vx = 1.0 * w_xy * math.cos(w_xy * t)
        vy = 2.0 * w_xy * math.cos(w_xy * t)
        vz = 0.5 * w_z * math.cos(w_z * t)
        
        ax = -1.0 * (w_xy ** 2) * math.sin(w_xy * t)
        ay = -2.0 * (w_xy ** 2) * math.sin(w_xy * t)
        az = -0.5 * (w_z ** 2) * math.sin(w_z * t)
        
        heading = 0.5 * math.sin(w_z * t)
        return ControllerReference((px, py, pz), (vx, vy, vz), (ax, ay, az), heading)
    else:
        # Fixed setpoint target
        return ControllerReference((1.0, 1.0, 2.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run the full backstepping quadrotor controller simulation."
    )
    parser.add_argument(
        "--launch-simulator",
        nargs=argparse.REMAINDER,
        help="Command to launch Gazebo or ArduPilot SITL, e.g. --launch-simulator gazebo --verbose",
    )
    parser.add_argument(
        "--connect",
        type=str,
        default=None,
        help="MAVLink connection string, e.g. udpin:127.0.0.1:14550 or tcp:127.0.0.1:5760",
    )
    parser.add_argument("--mode", type=str, choices=["setpoint", "helix"], default="setpoint", help="Trajectory mode: setpoint or helix.")
    parser.add_argument("--steps", type=int, default=250, help="Number of control steps to run.")
    parser.add_argument("--dt", type=float, default=0.02, help="Time step in seconds.")
    parser.add_argument("--plot", type=str, default="simulation_results.png", help="Path to save output trajectory plot.")
    args = parser.parse_args()

    controller = BacksteppingController(mass=1.5, gravity=9.81)
    bridge = SITLBridge(connection_string=args.connect)

    if args.launch_simulator:
        print("Launching simulator:", " ".join(args.launch_simulator))
        bridge.launch_simulator(args.launch_simulator)

    bridge.connect()
    if args.connect:
        bridge.set_mode("GUIDED")
        bridge.arm_vehicle(True)

    state = ControllerState(
        position=(0.0, 0.0, 0.0),
        velocity=(0.0, 0.0, 0.0),
        quaternion=quaternion_from_euler(0.0, 0.0, 0.0),
        angular_rate=(0.0, 0.0, 0.0),
    )

    times = []
    actual_x, actual_y, actual_z = [], [], []
    ref_x, ref_y, ref_z = [], [], []
    thrust_history = []
    error_history = []

    try:
        for step in range(args.steps):
            t = step * args.dt
            reference = get_reference(t, args.mode)

            if args.connect:
                veh_state = bridge.receive_mavlink_state(
                    VehicleState(state.position, state.velocity, state.quaternion, state.angular_rate)
                )
                state = ControllerState(
                    position=veh_state.position,
                    velocity=veh_state.velocity,
                    quaternion=veh_state.quaternion,
                    angular_rate=veh_state.angular_rate,
                )

            thrust, torques = controller.compute_command(state, reference, dt=args.dt)
            target = bridge.build_attitude_target(
                thrust=thrust,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                yaw_rate=torques[2],
            )
            bridge.send_attitude_target(target, quaternion=state.quaternion)

            pos_err = math.sqrt(sum((state.position[i] - reference.position[i])**2 for i in range(3)))

            times.append(t)
            actual_x.append(state.position[0])
            actual_y.append(state.position[1])
            actual_z.append(state.position[2])
            ref_x.append(reference.position[0])
            ref_y.append(reference.position[1])
            ref_z.append(reference.position[2])
            thrust_history.append(thrust)
            error_history.append(pos_err)

            if step % 25 == 0 or step == args.steps - 1:
                print(f"t={t:5.2f}s | pos=({state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f}) | ref=({reference.position[0]:.2f}, {reference.position[1]:.2f}, {reference.position[2]:.2f}) | thrust={thrust:.2f}N | err={pos_err:.3f}m")

            if not args.connect:
                state = update_state(state, thrust, torques, args.dt)

            
            # Short sleep when not running high speed
            if args.connect:
                time.sleep(args.dt)

        print("\n--- Simulation Summary ---")
        print(f"Mode: {args.mode}")
        print(f"Total Steps: {args.steps} ({args.steps * args.dt:.1f} sec)")
        print(f"Final Position Error: {error_history[-1]:.4f} m")
        print(f"Average Tracking Error: {sum(error_history)/len(error_history):.4f} m")

        # Plot results
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axes[0].plot(times, ref_x, 'r--', label='x ref')
        axes[0].plot(times, actual_x, 'r-', label='x actual')
        axes[0].plot(times, ref_y, 'g--', label='y ref')
        axes[0].plot(times, actual_y, 'g-', label='y actual')
        axes[0].set_ylabel('Position X/Y (m)')
        axes[0].legend(loc='upper right')
        axes[0].grid(True)
        axes[0].set_title(f'Quadrotor Backstepping Controller ({args.mode.capitalize()} Mode)')

        axes[1].plot(times, ref_z, 'b--', label='z ref')
        axes[1].plot(times, actual_z, 'b-', label='z actual')
        axes[1].set_ylabel('Position Z (m)')
        axes[1].legend(loc='upper right')
        axes[1].grid(True)

        axes[2].plot(times, error_history, 'k-', label='Tracking Error (m)')
        axes[2].plot(times, thrust_history, 'm:', label='Thrust Command (N)')
        axes[2].set_xlabel('Time (s)')
        axes[2].set_ylabel('Error / Thrust')
        axes[2].legend(loc='upper right')
        axes[2].grid(True)

        plt.tight_layout()
        plt.savefig(args.plot, dpi=150)
        print(f"Plot saved to {args.plot}")

    finally:
        bridge.disconnect()


if __name__ == "__main__":
    main()


