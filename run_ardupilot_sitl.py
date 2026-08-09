import time
import argparse
import math
import matplotlib.pyplot as plt
import dronekit_sitl
from quadrotor_controller import (
    BacksteppingController,
    ControllerReference,
    ControllerState,
    SITLBridge,
    VehicleState,
    quaternion_from_euler,
)


def main():
    parser = argparse.ArgumentParser(description="Run ArduPilot SITL with Quadrotor Backstepping Controller")
    parser.add_argument("--steps", type=int, default=100, help="Number of control loop steps")
    parser.add_argument("--dt", type=float, default=0.02, help="Time step delta")
    args = parser.parse_args()

    print("==================================================")
    print("Starting ArduPilot SITL (ArduCopter)...")
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
    print(f"ArduPilot SITL active on: {connection_string}")
    print("==================================================")

    bridge = SITLBridge(connection_string=connection_string)
    controller = BacksteppingController(mass=1.5, gravity=9.81)

    try:
        print("Connecting MAVLink bridge to ArduPilot...")
        bridge.connect(timeout=15.0)
        print("Setting mode to GUIDED and Arming vehicle...")
        bridge.set_mode("GUIDED")
        bridge.arm_vehicle(True)


        state = ControllerState(
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            quaternion=quaternion_from_euler(0.0, 0.0, 0.0),
            angular_rate=(0.0, 0.0, 0.0),
        )
        reference = ControllerReference(
            position=(1.0, 1.0, 2.0),
            velocity=(0.0, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0),
            heading=0.0,
        )

        print("\n--- Starting Live Control Loop with ArduPilot ---")
        for step in range(args.steps):
            t = step * args.dt
            # Polling telemetry from ArduPilot SITL
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

            if step % 10 == 0 or step == args.steps - 1:
                print(f"step={step:03d} | t={t:5.2f}s | pos=({state.position[0]:.2f}, {state.position[1]:.2f}, {state.position[2]:.2f}) | thrust={thrust:.2f}N | torques=({torques[0]:.2f}, {torques[1]:.2f}, {torques[2]:.2f})")

            time.sleep(args.dt)

        print("\nLive ArduPilot SITL control run finished successfully!")


    finally:
        print("Stopping SITL...")
        bridge.disconnect()
        sitl.stop()
        print("SITL stopped.")


if __name__ == "__main__":
    main()
