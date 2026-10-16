"""Web dashboard backend: HTTP API, route check (between buildings / moved target), per-second export."""
import csv
import io
import json
import os
import sys
import threading
import urllib.request
from http.server import ThreadingHTTPServer

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import run_web_dashboard as web  # noqa: E402


def make_backend():
    b = web.FlightBackend("off")
    b.sim.ref.target_path = None              # ignore the user's target.txt
    return b


def test_route_check_verdicts():
    b = make_backend()
    b.telemetry = {"pos": [0.0, 0.0, 1.0]}
    r = b.check_route([3.2, 3.2, 1.0])                    # inside the clock tower
    assert r["ok"] and r["moved"]
    r = b.check_route([2.0, 20.0, 4.0])                   # avenue between two blocks
    assert r["kind"] == "between" and r["narrowest_gap"] > 2 * 1.3
    r = b.check_route([25.0, -20.0, 4.0])                 # enclosed at 4 m -> over the top
    assert r["kind"] == "over" and r["max_alt"] > 4.0


def test_export_one_row_per_second_and_http_api(tmp_path):
    from quadrotor_controller.config import cfg, set_value
    original = cfg("paths.results_dir")
    set_value("paths.results_dir", str(tmp_path))          # keep the project's results/ clean
    try:
        _run_export_and_api(tmp_path)
    finally:
        set_value("paths.results_dir", original)


def _run_export_and_api(tmp_path):
    b = make_backend()
    b.command("2 3 2")
    sim = b.sim
    for _ in range(int(round(5.0 / b.frame_dt))):           # run the loop body without real-time sleeps
        with b.lock:
            cmds, b.commands = b.commands, []
        for c in cmds:
            sim.command(c)
        sim.advance(b.frame_dt)
        tm = b._telemetry()
        b.telemetry = tm
        if sim.t + b.frame_dt / 2 >= b.next_row:
            b.next_row += b.export_period
            b.rows.append(b._row(tm))
    _, data = b.export("csv")
    rows = list(csv.DictReader(io.StringIO(data)))
    assert len(rows) == 6 and list(rows[0].keys()) == web.EXPORT_COLUMNS      # t = 0, 1, ..., 5 s
    times = [float(r["time_s"]) for r in rows[1:]]
    assert all(abs((t2 - t1) - 1.0) < 0.05 for t1, t2 in zip(times, times[1:]))

    srv = ThreadingHTTPServer(("127.0.0.1", 0), web.make_handler(b, {"prims": []}))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        tm = json.load(urllib.request.urlopen(base + "/api/telemetry"))
        assert "pos" in tm and "euler_deg" in tm
        req = urllib.request.Request(base + "/api/command", data=json.dumps({"text": "MODE hover"}).encode(),
                                     headers={"Content-Type": "application/json"})
        assert json.load(urllib.request.urlopen(req))["ok"]
        page = urllib.request.urlopen(base + "/").read().decode()
        assert "/*WEB_CONFIG*/null" not in page and "poll_ms" in page
        assert urllib.request.urlopen(base + "/vendor/three.min.js").status == 200
    finally:
        srv.shutdown()
    assert any(p.suffix == ".csv" for p in tmp_path.iterdir())       # export also saved to the results folder
