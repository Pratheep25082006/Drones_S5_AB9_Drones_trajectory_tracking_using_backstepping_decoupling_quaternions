"""
Web telemetry bridge - full De Monte & Lohmann (2013) controller
----------------------------------------------------------------
* Pure simulation (default): quadrotor_controller.PythonFlightSim
  (rigid body 500 Hz + paper controller + noise + Kalman filter).
* --connect <connection string> : ArduPilot SITL / real vehicle over MAVLink (e.g. mavlink.connection).
  The paper controller runs at 50 Hz on the received state and sends its
  body-rate reference + thrust with SET_ATTITUDE_TARGET.
* --gazebo : also stream the pose to the WSL Gazebo receiver (gazebo.udp_port).

Dashboard: http://<bridge.host>:<bridge.http_port>/<bridge.dashboard>, WebSocket on bridge.ws_port.
Settings: config.yaml (bridge, python_sim, mavlink, gazebo, vehicle).
"""

import argparse
import asyncio
import http.server
from http import HTTPStatus
import json
import math
import os
import socketserver
import sys
import threading
import time

import websockets

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller.config import add_config_arguments, apply_config_arguments, cfg, project_path  # noqa: E402
from quadrotor_controller.gazebo_tools import PoseSender  # noqa: E402
from quadrotor_controller import (  # noqa: E402
    ControllerState,
    PaperBacksteppingController,
    PythonFlightSim,
    ReferenceManager,
    SITLBridge,
    VehicleState,
    parse_target_text,
)

latest_telemetry = {}


def init_telemetry():
    start = list(cfg("python_sim.start"))
    latest_telemetry.update({
        "step": 0, "time": 0.0, "mode": "auto", "phase": "", "pos": start, "ref": start,
        "custom_target": list(cfg("trajectory.hover_point")), "vel": [0.0, 0.0, 0.0],
        "quat": [1.0, 0.0, 0.0, 0.0], "thrust": cfg("vehicle.mass") * cfg("vehicle.gravity"),
        "torques": [0.0, 0.0, 0.0], "error": 0.0, "status": "Initializing", "connection": "Disconnected",
    })
command_queue = []
command_lock = threading.Lock()
connected_clients = set()


def queue_command(text):
    with command_lock:
        command_queue.append(text)


def pop_commands():
    with command_lock:
        cmds = list(command_queue)
        command_queue.clear()
    return cmds


def euler_from_quaternion(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def simulation_thread(args):
    sim = PythonFlightSim(target_path=project_path("target_file"))
    latest_telemetry["connection"] = f"Pure Simulation (Python rigid body {1 / sim.dt:.0f} Hz)"
    latest_telemetry["status"] = "Paper backstepping controller active"
    sender = PoseSender() if args.gazebo else None
    step = 0
    while True:
        t0 = time.time()
        for c in pop_commands():
            sim.command(c)
        sim.advance(args.dt)
        tm = sim.telemetry()
        latest_telemetry.update(tm)
        latest_telemetry["step"] = step
        if sender is not None:
            sender.send(cfg("gazebo.udp_port"), (*sim.plant.x, *sim.plant.euler()))
            if sim.last_ref is not None:
                sender.send(cfg("gazebo.target_udp_port"), (*sim.marker_position(), 0.0, 0.0, 0.0))
        step += 1
        rem = args.dt - (time.time() - t0)
        if rem > 0:
            time.sleep(rem)


def mavlink_thread(args):
    bridge = SITLBridge(connection_string=args.connect)
    try:
        bridge.connect()
        bridge.set_mode()
        bridge.arm_vehicle(True)
        latest_telemetry["connection"] = f"MAVLink {args.connect}"
        latest_telemetry["status"] = "Armed - paper controller sending body rates + thrust"
    except Exception as e:
        latest_telemetry["status"] = f"MAVLink connection failed: {e}"
        return
    ctl = PaperBacksteppingController()
    state = VehicleState((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    state = bridge.receive_enu_state(state)
    refman = ReferenceManager(start_pos=state.position, target_path=project_path("target_file"))
    t, step = 0.0, 0
    hover = ctl.mass * ctl.g
    while True:
        t0 = time.time()
        for c in pop_commands():
            kind, data, heading = parse_target_text(c)
            if kind == "coords":
                refman._activate("coords", t, (data, heading))
            elif kind == "mode":
                refman._activate(data, t, None)
        state = bridge.receive_enu_state(state)
        cs = ControllerState(state.position, state.velocity, state.quaternion, state.angular_rate)
        ref = refman.update(t, args.dt)
        thrust, tau = ctl.compute(cs, ref, args.dt)
        bridge.send_rates_thrust(ctl.last["omega_d"], thrust, hover)
        latest_telemetry.update({
            "step": step, "time": round(t, 2), "mode": refman.mode, "phase": refman.phase(t),
            "pos": [round(v, 3) for v in state.position], "ref": [round(v, 3) for v in ref.pos],
            "vel": [round(v, 3) for v in state.velocity], "quat": [round(v, 4) for v in state.quaternion],
            "thrust": round(thrust, 2), "torques": [round(v, 4) for v in tau],
            "error": round(math.dist(state.position, ref.pos), 3),
        })
        t += args.dt
        step += 1
        rem = args.dt - (time.time() - t0)
        if rem > 0:
            time.sleep(rem)


async def ws_handler(websocket, *_):
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except ValueError:
                continue
            if "set_target" in data:
                x, y, z = (float(v) for v in data["set_target"][:3])
                latest_telemetry["custom_target"] = [x, y, z]
                queue_command(f"{x} {y} {z}")
                print(f"[Web] Target set to ({x}, {y}, {z})")
            elif "set_mode" in data:
                mode = str(data["set_mode"]).lower()
                if mode == "helix":
                    queue_command("MODE spiral")
                elif mode == "setpoint":
                    x, y, z = latest_telemetry.get("custom_target", [0.0, 0.0, 1.0])
                    queue_command(f"{x} {y} {z}")
                else:
                    queue_command(f"MODE {mode}")
                print(f"[Web] Mode -> {mode}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.discard(websocket)


async def broadcast_loop():
    while True:
        if connected_clients:
            msg = json.dumps(latest_telemetry)
            await asyncio.gather(*[c.send(msg) for c in list(connected_clients)], return_exceptions=True)
        await asyncio.sleep(cfg("bridge.broadcast_period"))


def start_http_server(host, port):
    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=HERE, **kw)

        def do_GET(self):
            if self.path.split("?")[0].lstrip("/") in ("", cfg("bridge.dashboard")):
                with open(os.path.join(HERE, cfg("bridge.dashboard")), encoding="utf-8") as f:
                    page = f.read()
                settings = {"ws_host": cfg("bridge.host"), "ws_port": cfg("bridge.ws_port"),
                            "reconnect_ms": cfg("bridge.reconnect_ms")}
                body = page.replace("/*DASHBOARD_CONFIG*/null", json.dumps(settings)).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            super().do_GET()

        def log_message(self, *args):
            pass

    socketserver.TCPServer.allow_reuse_address = True
    try:
        with socketserver.TCPServer((host, port), QuietHandler) as httpd:
            print(f"[HTTP] Dashboard: http://{host}:{port}/{cfg('bridge.dashboard')}")
            httpd.serve_forever()
    except OSError as e:
        print(f"[HTTP] Port {port}: {e}")


async def async_main():
    host, port = cfg("bridge.host"), cfg("bridge.ws_port")
    async with websockets.serve(ws_handler, host, port):
        print(f"[WebSocket] Telemetry on ws://{host}:{port}")
        await broadcast_loop()


def main():
    ap = argparse.ArgumentParser(description="Web telemetry bridge (paper backstepping controller)")
    ap.add_argument("--connect", type=str, default=None, help="MAVLink connection string (see mavlink.connection)")
    ap.add_argument("--gazebo", action="store_true", help="also stream the pose to Gazebo (gazebo.udp_port)")
    ap.add_argument("--open-browser", action="store_true", help="open the dashboard in the web browser")
    add_config_arguments(ap)
    args = ap.parse_args()
    apply_config_arguments(args)
    args.dt = cfg("bridge.frame_dt") if not args.connect else cfg("mavlink.control_dt")
    init_telemetry()

    target = mavlink_thread if args.connect else simulation_thread
    threading.Thread(target=target, args=(args,), daemon=True).start()
    threading.Thread(target=start_http_server, args=(cfg("bridge.host"), cfg("bridge.http_port")), daemon=True).start()
    if args.open_browser:
        import webbrowser
        threading.Timer(cfg("gazebo.launch_wait"), webbrowser.open,
                        args=(f"http://{cfg('bridge.host')}:{cfg('bridge.http_port')}/{cfg('bridge.dashboard')}",)).start()
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        print("[Bridge] Shutting down.")


if __name__ == "__main__":
    main()
