"""End-to-end MAVLink pre-flight against the mock ArduCopter (streams, params, EKF, mode, arm, land)."""
import os
import subprocess
import sys
import time

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)
pytest.importorskip("pymavlink")
from quadrotor_controller.sitl_bridge import SITLBridge  # noqa: E402

PORT = 14571


@pytest.fixture
def mock_vehicle():
    p = subprocess.Popen([sys.executable, os.path.join(ROOT, "mock_mavlink_vehicle.py"),
                          "--out", f"udpout:127.0.0.1:{PORT}", "--duration", "40",
                          "--set", "mock_vehicle.ekf_ready_after=0.5", "--set", "mock_vehicle.start=[0.0, 0.0, 0.25]"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.3)
    yield p
    p.kill()
    p.wait()


def test_preflight_arm_and_land(mock_vehicle):
    bridge = SITLBridge(connection_string=f"udpin:127.0.0.1:{PORT}")
    bridge.connect(timeout=10)
    try:
        bridge.preflight()
        assert bridge.vehicle_armed
        assert bridge.hover_throttle == pytest.approx(0.35)          # read from MOT_THST_HOVER
        assert bridge.get_param("GUID_OPTIONS") == pytest.approx(8)  # set by preflight
        assert bridge.in_control()
        assert bridge.land(timeout=20)                                # on the ground: lands + disarms at once
        assert not bridge.vehicle_armed
    finally:
        bridge.disconnect()
