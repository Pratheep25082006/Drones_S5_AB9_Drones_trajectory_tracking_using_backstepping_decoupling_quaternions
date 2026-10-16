"""Tests for the C^4 reference trajectories, reference model and guidance layer."""
import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from quadrotor_controller import (  # noqa: E402
    LidarScan,
    LoopingMission,
    ReactiveObstacleAvoider,
    ReferenceManager,
    SetpointReferenceModel,
    paper_helix,
    parse_target_text,
)


def _fd_mismatch(fn, times, h=1e-4):
    worst = 0.0
    for t in times:
        p, a, b = fn(t), fn(t - h), fn(t + h)
        for k in range(4):
            num = (np.array(b.derivs()[k]) - np.array(a.derivs()[k])) / (2 * h)
            worst = max(worst, float(np.abs(num - np.array(p.derivs()[k + 1])).max()))
    return worst


def test_paper_helix_matches_fig4_parameters():
    # parameters read off the paper's Fig. 4 (passed explicitly: this tests the function, not config.yaml)
    h = paper_helix(center=(0, 0, 1), amp_x=1.0, amp_y=2.0, amp_z=0.5, period_xy=6.25, period_z=12.5,
                    yaw_amp=1.0, scale=1.0)
    xs = [h(t).pos for t in np.linspace(0, 12.5, 2001)]
    x, y, z = np.array(xs).T
    assert abs(x.max() - 1.0) < 1e-3 and abs(y.max() - 2.0) < 1e-3
    assert abs(z.min() - 0.5) < 1e-3 and abs(z.max() - 1.5) < 1e-3
    # y lags x by a quarter of the 6.25 s period
    assert abs(h(6.25 / 4).pos[1] - 0.0) < 1e-9 and abs(h(0).pos[1] + 2.0) < 1e-9


def test_derivatives_are_consistent_up_to_snap():
    assert _fd_mismatch(paper_helix(), np.linspace(0.3, 20, 50)) < 1e-3
    m = LoopingMission()
    assert _fd_mismatch(m, np.linspace(0.3, 150, 400)) < 1e-2


def test_mission_is_c4_continuous_and_speed_bounded():
    from quadrotor_controller import cfg, figure8
    m = LoopingMission()
    h, f8 = paper_helix(), figure8()
    # the mission may never be faster than its fastest manoeuvre
    bound = max(max(np.linalg.norm(h(t).vel), np.linalg.norm(f8(t).vel)) for t in np.arange(0, 13, 0.01))
    vmax = 0.0
    for t in np.arange(0.01, 150, 0.01):
        vmax = max(vmax, float(np.linalg.norm(m(t).vel)))
    assert vmax <= bound * 1.001
    joints = np.cumsum([m.t0] + [d for _, d, _ in m.cycle.parts])
    for t in joints:
        a, b = m(t - 1e-7), m(t + 1e-7)
        for da, db in zip(a.derivs(), b.derivs()):
            assert np.abs(np.array(da) - np.array(db)).max() < 1e-3


def test_setpoint_reference_model_converges_with_speed_limit():
    rm = SetpointReferenceModel((0, 0, 1), v_max=1.5)
    rm.set_target((5, 5, 10))
    vmax = 0.0
    for _ in range(int(30 / 0.002)):
        p = rm.step(0.002)
        vmax = max(vmax, float(np.linalg.norm(p.vel)))
    assert vmax <= 1.51
    assert np.linalg.norm(np.array(p.pos) - np.array([5, 5, 10])) < 0.01


def test_parse_target_text():
    assert parse_target_text("5 5 10") == ("coords", (5.0, 5.0, 10.0), 0.0)
    kind, pos, hdg = parse_target_text("1 2 3 90")
    assert kind == "coords" and abs(hdg - math.pi / 2) < 1e-12
    assert parse_target_text("MODE spiral")[:2] == ("mode", "spiral")
    assert parse_target_text("auto")[:2] == ("mode", "auto")
    assert parse_target_text("garbage")[0] is None


def test_reference_manager_mode_switching_is_continuous():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "target.txt")
        open(path, "w").write("MODE auto")
        rm = ReferenceManager((0, 0, 0.2), target_path=path, poll_period=0.0)
        dt, t, prev = 0.002, 0.0, None
        script = {10.0: "3 -2 2.5 45", 25.0: "MODE spiral", 45.0: "MODE figure8", 70.0: "land"}
        while t < 90.0:
            for ts, cmd in script.items():
                if abs(t - ts) < dt / 2:
                    open(path, "w").write(cmd)
            ref = rm.update(t, dt)
            if prev is not None:
                assert np.linalg.norm(np.array(ref.pos) - np.array(prev.pos)) < 0.02   # no jumps
            prev = ref
            t += dt
        assert ref.pos[2] < 0.3   # landed reference


def test_avoider_ignore_behind_and_side_hysteresis():
    av = ReactiveObstacleAvoider(safety_distance=1.0, detection_distance=2.0, ignore_behind=True)
    # wall behind while flying forward -> ignored
    behind = LidarScan(ranges=[8, 8, 8, 8, 1.5, 8, 8, 8], dist_up=8, dist_down=2)
    v, avoiding, _ = av.compute_avoidance_velocity(behind, (1, 0, 0), (1, 0, 0))
    assert not avoiding and v == (1, 0, 0)
    # head-on wall: chosen side is kept even when the goal direction tilts the other way
    front = LidarScan(ranges=[1.5, 8, 8, 8, 8, 8, 8, 8], dist_up=8, dist_down=2)
    v1, a1, _ = av.compute_avoidance_velocity(front, (1, 0, 0), (1, 0.05, 0))
    v2, a2, _ = av.compute_avoidance_velocity(front, (1, 0, 0), (1, -0.05, 0))
    assert a1 and a2 and np.sign(v1[1]) == np.sign(v2[1])
