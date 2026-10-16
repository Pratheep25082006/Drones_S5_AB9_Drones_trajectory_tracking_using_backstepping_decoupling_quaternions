"""
Web 3D dashboard - backend (flight core + Gazebo) and browser front end
=======================================================================
Backend (this script):
  * PythonFlightSim: rigid body 500 Hz + De Monte & Lohmann (2013) controller + sensor noise
    + Kalman filter + LiDAR on the city map + reactive avoidance + global path planner.
  * Streams the drone pose and target marker to Gazebo 11 (the Gazebo window shows the same flight).
  * Serves the front end (web.page) and a small JSON API - Python standard library only, so it
    also runs inside WSL next to Gazebo with Gazebo's own Python.

Front end (drone_dashboard.html, opened in the browser):
  * 3D city (the same generated world as MuJoCo / PyBullet / Gazebo), drone, trails, planned route
  * live trajectory values and charts on the side
  * targets: type X Y Z or click the ground / a roof; "Check route" tells whether the drone can fly
    between the buildings (gap widths, clearance) before it flies
  * export of all trajectory values per second (CSV / JSON, also saved in results/)

Run:
    START_WEB_DASHBOARD.bat                       Gazebo + backend inside WSL, browser on Windows (recommended)
    python run_web_dashboard.py --gazebo --open-browser   backend on Windows, Gazebo in WSL (UDP)
    python run_web_dashboard.py --open-browser            web only (no Gazebo)
Settings: config.yaml -> web, planner, python_sim, gazebo, world.

API:  GET  /api/world  /api/telemetry  /api/route  /api/export.csv  /api/export.json
      POST /api/command {"text": "X Y Z [deg]" | "MODE auto|spiral|figure8|hover|land"}
      POST /api/check_route {"target": [x, y, z]}      POST /api/clear_log
"""

import argparse
import csv
import io
import json
import math
import os
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from quadrotor_controller import GridPlanner, PythonFlightSim  # noqa: E402
from quadrotor_controller.config import add_config_arguments, apply_config_arguments, cfg, project_path  # noqa: E402
from quadrotor_controller.guidance import parse_target_text  # noqa: E402
from quadrotor_controller.units import DEG_PER_TURN, MS_PER_S  # noqa: E402
from quadrotor_controller.world_builder import CityBuilder  # noqa: E402

RAD_TO_DEG = DEG_PER_TURN / (2.0 * math.pi)

EXPORT_COLUMNS = [
    "time_s", "mode", "phase",
    "x_m", "y_m", "z_m", "x_ref_m", "y_ref_m", "z_ref_m", "error_m",
    "vx_mps", "vy_mps", "vz_mps", "speed_mps",
    "roll_deg", "pitch_deg", "yaw_deg", "p_radps", "q_radps", "r_radps",
    "thrust_N", "tau_x_Nm", "tau_y_Nm", "tau_z_Nm",
    "kf_error_m", "noise_m", "lyapunov",
    "goal_x_m", "goal_y_m", "goal_z_m", "route_points_left", "lidar",
]


# ---------------------------------------------------------------------- world for the browser
def world_json() -> dict:
    b = CityBuilder()
    prims = b.build()
    mats = b.materials()
    w = cfg("world")
    return {
        "prims": [{"n": p.name, "k": p.kind, "s": list(p.size), "p": list(p.pos), "m": p.material,
                   "yaw": p.yaw, "c": p.collide} for p in prims],
        "materials": {k: {"rgba": v["rgba"]} for k, v in mats.items()},
        "ground_size": w["ground_size"], "extent": w["extent"], "sky": w["sky"], "sun": w["sun"],
        "plaza_top": w["plaza"]["top"],
    }


# ---------------------------------------------------------------------- backend
class FlightBackend:
    def __init__(self, gazebo: str, initial: str | None = None):
        wc = "web."
        self.sim = PythonFlightSim(target_path=project_path("target_file"), initial_command=initial)
        self.preview_planner = GridPlanner(self.sim.world) if self.sim.world is not None else None
        self.frame_dt = cfg(wc + "frame_dt")
        self.export_period = cfg(wc + "export_period")
        self.lock = threading.Lock()
        self.commands = []
        self.rows = []
        self.next_row = 0.0
        self.telemetry = {}
        self.route = {"id": -1, "points": []}
        self.status = "starting"
        self.gz = None
        if gazebo != "off":
            from quadrotor_controller.gazebo_tools import GazeboPoseOutput
            self.gz = GazeboPoseOutput(local=(gazebo == "local"))
            self.status = "Gazebo: " + self.gz.describe()
        else:
            self.status = "web only (no Gazebo)"

    # -------------------------------------------------- commands
    def command(self, text: str) -> bool:
        kind, _, _ = parse_target_text(text)
        if kind is None:
            return False
        with self.lock:
            self.commands.append(text)
        return True

    def clear_log(self) -> None:
        with self.lock:
            self.rows = []
            self.next_row = self.sim.t

    # -------------------------------------------------- loop
    def run(self) -> None:
        sim = self.sim
        while True:
            t0 = time.time()
            with self.lock:
                cmds, self.commands = self.commands, []
            for c in cmds:
                sim.command(c)
                print(f"[WEB] command: {c}")
            sim.advance(self.frame_dt)
            tm = self._telemetry()
            with self.lock:
                self.telemetry = tm
                if sim.ref.route_id != self.route["id"]:
                    self.route = {"id": sim.ref.route_id,
                                  "points": [[round(float(v), 3) for v in q] for q in sim.ref.route]}
                if sim.t + self.frame_dt / 2 >= self.next_row:
                    self.next_row += self.export_period
                    self.rows.append(self._row(tm))
            if self.gz is not None:
                self.gz.send(sim.t, (*sim.plant.x, *sim.plant.euler()), sim.marker_position())
            rem = self.frame_dt - (time.time() - t0)
            if rem > 0:
                time.sleep(rem)

    def _telemetry(self) -> dict:
        sim = self.sim
        tm = sim.telemetry()
        roll, pitch, yaw = sim.plant.euler()
        tm.update({
            "euler_deg": [round(v * RAD_TO_DEG, 2) for v in (roll, pitch, yaw)],
            "speed": round(float(np.linalg.norm(sim.plant.v)), 3),
            "marker": [round(float(v), 3) for v in sim.marker_position()],
            "route_id": sim.ref.route_id,
            "avoiding": bool(sim.ref.avoiding),
            "status": self.status,
            "rows": len(self.rows),
        })
        return tm

    @staticmethod
    def _row(tm: dict) -> list:
        p, r, v, e = tm["pos"], tm["ref"], tm["vel"], tm["euler_deg"]
        return [tm["time"], tm["mode"], tm["phase"], *p, *r, tm["error"], *v, tm["speed"], *e, *tm["rates"],
                tm["thrust"], *tm["torques"], tm["est_error"], tm["noise"], tm["lyapunov"], *tm["goal"],
                tm["waypoints_left"], tm["lidar"]]

    # -------------------------------------------------- route check (before flying)
    def check_route(self, target) -> dict:
        pl = self.preview_planner
        with self.lock:
            start = tuple(self.telemetry.get("pos", cfg("python_sim.start")))
        target = tuple(float(v) for v in target[:3])
        if pl is None:
            return {"ok": False, "verdict": "Avoidance is off (python_sim.avoid: false) - no route check."}
        t0 = time.time()
        pts = [start] + pl.plan(start, target)
        note = pl.note
        goal = pts[-1]
        world = pl.world
        ib = cfg("planner.ignore_below")
        lo, hi = world.boxes(ib)
        # sample the curve: clearance to obstacles, gaps between buildings (probe left / right)
        step, probe = cfg("web.gap_probe_step"), cfg("web.gap_probe_range")
        samples = []
        for a, b in zip(pts[:-1], pts[1:]):
            n = max(1, int(math.ceil(math.dist(a, b) / step)))
            samples += [tuple(a[i] + (b[i] - a[i]) * k / n for i in range(3)) for k in range(n)]
        samples.append(goal)
        length = sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))
        min_clear, narrow, between = math.inf, math.inf, 0
        for k, s in enumerate(samples):
            gap = np.maximum(np.maximum(lo - np.asarray(s), np.asarray(s) - hi), 0.0)
            min_clear = min(min_clear, float(np.min(np.linalg.norm(gap, axis=1))) if len(gap) else math.inf)
            nxt = samples[min(k + 1, len(samples) - 1)]
            prv = samples[max(k - 1, 0)]
            d = (nxt[0] - prv[0], nxt[1] - prv[1])
            dn = math.hypot(*d)
            if dn == 0.0:
                continue
            left = world.raycast(s, (-d[1] / dn, d[0] / dn, 0.0), probe)
            right = world.raycast(s, (d[1] / dn, -d[0] / dn, 0.0), probe)
            if left < probe and right < probe and s[2] > ib:
                between += 1
                narrow = min(narrow, left + right)
        moved = math.dist(goal, target) > cfg("planner.resolution") / 2
        direct = len(pts) == 2
        how = "straight" if direct else "on a curved route"
        if "over the top" in note:
            kind = "over"
            verdict = (f"No gap between the buildings is wide enough at this height - the drone climbs over them "
                       f"(max {max(p[2] for p in pts):.1f} m).")
        elif between:
            kind = "between"
            verdict = (f"Yes - the drone can fly between the buildings ({how}): narrowest gap {narrow:.1f} m, "
                       f"clearance {cfg('planner.clearance'):.1f} m kept on each side.")
        elif direct:
            kind, verdict = "direct", "Direct path is clear - the drone flies straight to the target."
        else:
            kind, verdict = "around", "The route curves around the obstacles in open space."
        if moved:
            verdict = (f"Target is inside an obstacle - moved to ({goal[0]:.1f}, {goal[1]:.1f}, {goal[2]:.1f}). "
                       + verdict)
        speed = cfg("web.estimate_speed")
        return {
            "ok": True, "kind": kind, "verdict": verdict, "note": note, "moved": moved,
            "target": list(target), "goal": [round(v, 3) for v in goal],
            "points": [[round(v, 3) for v in q] for q in pts],
            "length": round(length, 2), "eta": round(length / speed, 1),
            "min_clearance": round(min_clear, 2) if math.isfinite(min_clear) else None,
            "narrowest_gap": round(narrow, 2) if math.isfinite(narrow) else None,
            "between_samples": between, "max_alt": round(max(p[2] for p in pts), 2),
            "plan_ms": round((time.time() - t0) * MS_PER_S),
        }

    # -------------------------------------------------- export
    def export(self, fmt: str):
        with self.lock:
            rows = list(self.rows)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        name = f"trajectory_{stamp}.{fmt}"
        if fmt == "csv":
            buf = io.StringIO()
            wr = csv.writer(buf, lineterminator="\n")
            wr.writerow(EXPORT_COLUMNS)
            wr.writerows(rows)
            data = buf.getvalue()
        else:
            data = json.dumps({"columns": EXPORT_COLUMNS, "period_s": self.export_period,
                               "rows": [dict(zip(EXPORT_COLUMNS, r)) for r in rows]}, indent=1)
        out_dir = project_path("results_dir")
        try:
            os.makedirs(out_dir, exist_ok=True)
            with open(os.path.join(out_dir, name), "w", encoding="utf-8", newline="") as f:
                f.write(data)
            print(f"[WEB] exported {len(rows)} rows -> {os.path.join(cfg('paths.results_dir'), name)}")
        except OSError as e:
            print("[WEB] could not save export:", e)
        return name, data


# ---------------------------------------------------------------------- HTTP
def make_handler(backend: FlightBackend, world: dict):
    page_path = os.path.join(HERE, cfg("web.page"))
    vendor_dir = os.path.join(HERE, cfg("web.vendor_dir"))
    front = {k: cfg("web." + k) for k in ("poll_ms", "history_s", "trail_points", "pick_altitude",
                                          "roof_clearance", "chart_period_ms")}
    front.update({"setpoint_speed": cfg("python_sim.setpoint_speed"), "export_period": cfg("web.export_period"),
                  "clearance": cfg("planner.clearance")})
    world_bytes = json.dumps(world).encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def _send(self, body: bytes, ctype: str, extra=None):
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj):
            self._send(json.dumps(obj).encode("utf-8"), "application/json")

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/" + cfg("web.page")):
                with open(page_path, encoding="utf-8") as f:
                    page = f.read().replace("/*WEB_CONFIG*/null", json.dumps(front))
                self._send(page.encode("utf-8"), "text/html; charset=utf-8")
            elif path.startswith("/vendor/"):
                name = os.path.basename(path)                         # no sub-directories / '..'
                fp = os.path.join(vendor_dir, name)
                if name.endswith(".js") and os.path.isfile(fp):
                    with open(fp, "rb") as f:
                        self._send(f.read(), "application/javascript")
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)
            elif path == "/api/world":
                self._send(world_bytes, "application/json")
            elif path == "/api/telemetry":
                with backend.lock:
                    tm = dict(backend.telemetry)
                self._json(tm)
            elif path == "/api/route":
                with backend.lock:
                    r = dict(backend.route)
                self._json(r)
            elif path in ("/api/export.csv", "/api/export.json"):
                fmt = path.rsplit(".", 1)[1]
                name, data = backend.export(fmt)
                self._send(data.encode("utf-8"), "text/csv" if fmt == "csv" else "application/json",
                           {"Content-Disposition": f'attachment; filename="{name}"'})
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                data = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                data = {}
            path = self.path.split("?")[0]
            if path == "/api/command":
                self._json({"ok": backend.command(str(data.get("text", "")))})
            elif path == "/api/check_route":
                try:
                    self._json(backend.check_route(data["target"]))
                except (KeyError, TypeError, ValueError) as e:
                    self._json({"ok": False, "verdict": f"bad target: {e}"})
            elif path == "/api/clear_log":
                backend.clear_log()
                self._json({"ok": True})
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

        def log_message(self, *args):
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description="Web 3D dashboard (backend + front end)")
    ap.add_argument("target", nargs="*", help="optional first set point: X Y Z [heading_deg]")
    ap.add_argument("--gazebo", action="store_true", help="start Gazebo in WSL and stream the flight to it (Windows)")
    ap.add_argument("--in-gazebo", action="store_true", help="running inside WSL next to Gazebo (used by launch_gazebo_wsl.py --web)")
    ap.add_argument("--open-browser", action="store_true")
    add_config_arguments(ap)
    args = ap.parse_args()
    apply_config_arguments(args)
    gazebo = "local" if args.in_gazebo else ("remote" if args.gazebo else "off")
    if args.gazebo:
        from quadrotor_controller.gazebo_tools import pose_transport
        import subprocess
        cmd = [sys.executable, os.path.join(HERE, "launch_gazebo_wsl.py"), "--receiver"]
        if pose_transport() == "plugin":
            cmd.append("--no-companion")
        print("[WEB] starting Gazebo 11 in WSL ...")
        try:
            subprocess.Popen(cmd)
        except OSError as e:
            print("[WEB] could not start Gazebo:", e)

    print("[WEB] building the city map ...")
    world = world_json()
    backend = FlightBackend(gazebo, " ".join(args.target) if len(args.target) >= 3 else None)
    threading.Thread(target=backend.run, daemon=True).start()

    host = cfg("web.wsl_host") if args.in_gazebo else cfg("web.host")
    port = cfg("web.port")
    server = ThreadingHTTPServer((host, port), make_handler(backend, world))
    url = f"http://localhost:{port}/"
    print("=" * cfg("logging.banner_width"))
    print(f"  WEB DASHBOARD  ->  {url}")
    print(f"  backend: PythonFlightSim + paper controller | {backend.status}")
    print("=" * cfg("logging.banner_width"))
    if args.open_browser:
        import webbrowser
        threading.Timer(cfg("web.open_browser_wait"), webbrowser.open, args=(url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[WEB] stopped.")


if __name__ == "__main__":
    main()
