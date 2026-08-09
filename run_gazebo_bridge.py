import argparse
import asyncio
import json
import math
import os
import sys
import threading
import time
import http.server
import socketserver
import websockets
import dronekit_sitl

from quadrotor_controller import (
    BacksteppingController,
    ControllerReference,
    ControllerState,
    SITLBridge,
    VehicleState,
    quaternion_from_euler,
    normalize_quaternion,
)

# Global Telemetry State
latest_telemetry = {
    "step": 0,
    "time": 0.0,
    "mode": "setpoint",
    "pos": [0.0, 0.0, 0.0],
    "ref": [1.0, 1.0, 2.0],
    "custom_target": [1.0, 1.0, 2.0],
    "vel": [0.0, 0.0, 0.0],
    "quat": [1.0, 0.0, 0.0, 0.0],
    "thrust": 14.715,
    "torques": [0.0, 0.0, 0.0],
    "error": 0.0,
    "status": "Initializing",
    "connection": "Disconnected",
}

connected_clients = set()


def get_reference(t: float, mode: str) -> ControllerReference:
    if mode == "helix":
        w_xy = 2.0 * math.pi / 6.25
        w_z = 2.0 * math.pi / 12.5
        px = 1.0 * math.sin(w_xy * t)
        py = 2.0 * math.sin(w_xy * t)
        pz = 1.0 + 0.5 * math.sin(w_z * t)
        vx = 1.0 * w_xy * math.cos(w_xy * t)
        vy = 2.0 * w_xy * math.cos(w_xy * t)
        vz = 0.5 * w_z * math.cos(w_z * t)
        ax = -1.0 * (w_xy ** 2) * math.sin(w_xy * t)
        ay = -2.0 * (w_xy ** 2) * math.sin(w_xy * t)
        az = -0.5 * (w_z ** 2) * math.sin(w_z * t)
        heading = 0.5 * math.sin(w_z * t)
        return ControllerReference((px, py, pz), (vx, vy, vz), (ax, ay, az), heading)
    else:
        tx, ty, tz = latest_telemetry.get("custom_target", [1.0, 1.0, 2.0])
        return ControllerReference((tx, ty, tz), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0)



def update_simulation_physics(
    state: ControllerState,
    thrust: float,
    torques: tuple[float, float, float],
    dt: float,
    mass: float = 1.5,
    gravity: float = 9.81,
    inertia: tuple[float, float, float] = (0.03, 0.03, 0.05),
) -> ControllerState:
    alpha_x = torques[0] / inertia[0]
    alpha_y = torques[1] / inertia[1]
    alpha_z = torques[2] / inertia[2]

    wx = state.angular_rate[0] + alpha_x * dt
    wy = state.angular_rate[1] + alpha_y * dt
    wz = state.angular_rate[2] + alpha_z * dt

    w, x, y, z = state.quaternion
    q_vec = (0.0, wx, wy, wz)
    q_dot = BacksteppingController._quaternion_multiply((w, x, y, z), q_vec)

    qw = w + 0.5 * q_dot[0] * dt
    qx = x + 0.5 * q_dot[1] * dt
    qy = y + 0.5 * q_dot[2] * dt
    qz = z + 0.5 * q_dot[3] * dt
    new_q = normalize_quaternion((qw, qx, qy, qz))

    direction = BacksteppingController._thrust_direction(new_q)
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


def control_loop_thread(args):
    global latest_telemetry
    controller = BacksteppingController(mass=1.5, gravity=9.81)

    sitl = None
    conn_str = args.connect
    if not conn_str:
        print("[Bridge] No connection string provided. Booting local ArduPilot SITL...")
        sitl = dronekit_sitl.start_default()
        conn_str = sitl.connection_string()
        print(f"[Bridge] ArduPilot SITL running on {conn_str}")

    bridge = SITLBridge(connection_string=conn_str)

    try:
        bridge.connect(timeout=15.0)
        latest_telemetry["connection"] = f"Connected ({conn_str})"
        bridge.set_mode("GUIDED")
        bridge.arm_vehicle(True)
        latest_telemetry["status"] = "Armed & Controlling (GUIDED)"

        state = ControllerState(
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            quaternion=quaternion_from_euler(0.0, 0.0, 0.0),
            angular_rate=(0.0, 0.0, 0.0),
        )

        step = 0
        dt = args.dt
        start_t = time.time()

        while True:
            t = step * dt
            ref = get_reference(t, latest_telemetry["mode"])

            # Receive MAVLink telemetry if connected to external SITL
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

            thrust, torques = controller.compute_command(state, ref, dt=dt)
            target = bridge.build_attitude_target(
                thrust=thrust,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                yaw_rate=torques[2],
            )
            bridge.send_attitude_target(target, quaternion=state.quaternion)

            pos_err = math.sqrt(sum((state.position[i] - ref.position[i])**2 for i in range(3)))

            # Update shared telemetry
            latest_telemetry.update({
                "step": step,
                "time": round(t, 2),
                "pos": [round(p, 3) for p in state.position],
                "ref": [round(p, 3) for p in ref.position],
                "vel": [round(v, 3) for v in state.velocity],
                "quat": [round(q, 4) for q in state.quaternion],
                "thrust": round(thrust, 2),
                "torques": [round(t, 3) for t in torques],
                "error": round(pos_err, 3),
            })

            if not args.connect:
                state = update_simulation_physics(state, thrust, torques, dt)

            step += 1
            time.sleep(dt)

    except Exception as e:
        print(f"[Bridge Error] {e}")
        latest_telemetry["status"] = f"Error: {e}"
    finally:
        bridge.disconnect()
        if sitl:
            sitl.stop()


async def ws_handler(websocket):
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                if "set_mode" in data:
                    latest_telemetry["mode"] = data["set_mode"]
                    print(f"[Web] Mode switched to {data['set_mode']}")
                if "set_target" in data:
                    target_vec = [float(x) for x in data["set_target"]]
                    latest_telemetry["custom_target"] = target_vec
                    latest_telemetry["mode"] = "setpoint"
                    print(f"[Web] Target setpoint updated to: {target_vec}")

            except Exception:
                pass
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.remove(websocket)


async def broadcast_loop():
    while True:
        if connected_clients:
            msg = json.dumps(latest_telemetry)
            await asyncio.gather(*[client.send(msg) for client in connected_clients], return_exceptions=True)
        await asyncio.sleep(0.04)  # 25 FPS update


def start_http_server(port=8080):
    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    with socketserver.TCPServer(("", port), QuietHandler) as httpd:
        print(f"[HTTP] Gazebo 3D Web Visualizer available at http://127.0.0.1:{port}/gazebo_visualizer.html")
        httpd.serve_forever()


async def async_main(args):
    # Start WebSocket Server
    ws_server = await websockets.serve(ws_handler, "127.0.0.1", 8081)
    print("[WebSocket] Bridge broadcasting telemetry on ws://127.0.0.1:8081")
    await broadcast_loop()


def main():
    parser = argparse.ArgumentParser(description="Gazebo & ArduPilot 3D Web Bridge")
    parser.add_argument("--connect", type=str, default=None, help="MAVLink connection string")
    parser.add_argument("--dt", type=float, default=0.02, help="Control loop dt")
    parser.add_argument("--port", type=int, default=8080, help="HTTP server port")
    args = parser.parse_args()

    # Start Control Thread
    t_control = threading.Thread(target=control_loop_thread, args=(args,), daemon=True)
    t_control.start()

    # Start HTTP Server Thread
    t_http = threading.Thread(target=start_http_server, args=(args.port,), daemon=True)
    t_http.start()

    # Run Asyncio event loop for WebSockets
    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        print("[Bridge] Shutting down.")


if __name__ == "__main__":
    main()
