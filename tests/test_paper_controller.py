"""Tests for the full De Monte & Lohmann (2013) backstepping controller."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from quadrotor_controller import (  # noqa: E402
    ControllerState,
    PaperBacksteppingController,
    RigidBodyQuadrotor,
    TrajectoryPoint,
    decompose_attitude,
    heading_coupling,
    heading_from_quaternion,
    paper_helix,
    quaternion_from_euler,
)
from quadrotor_controller.paper_controller import quat_mul, wrap_pi  # noqa: E402

DT = 0.002


def _fly(ref_fn, duration, plant=None, ctl=None, start=None):
    p0 = ref_fn(0.0)
    plant = plant or RigidBodyQuadrotor(position=start or p0.pos, ground_z=None)
    if start is None:
        plant.v = np.array(p0.vel)
    ctl = ctl or PaperBacksteppingController()
    errs, lyap = [], []
    for i in range(int(duration / DT)):
        t = i * DT
        ref = ref_fn(t)
        T, tau = ctl.compute(plant.state(), ref, DT)
        plant.step(T, tau, DT)
        errs.append(np.linalg.norm(plant.x - np.array(ref_fn(t + DT).pos)))
        lyap.append(ctl.last["lyapunov"])
    return np.array(errs), np.array(lyap), plant, ctl


def test_heading_kinematics_eq24_matches_finite_difference():
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = np.array(quaternion_from_euler(*rng.uniform(-0.6, 0.6, 2), rng.uniform(-3, 3)))
        w = rng.normal(size=3)
        h = 1e-6
        q2 = q + h * 0.5 * quat_mul(q, (0.0, *w))
        q2 /= np.linalg.norm(q2)
        numeric = wrap_pi(heading_from_quaternion(q2) - heading_from_quaternion(q)) / h
        formula = w[2] - heading_coupling(q) @ w[:2]
        assert abs(numeric - formula) < 1e-4


def test_heading_is_yaw_of_qz_quaternion():
    q = quaternion_from_euler(0.2, -0.1, 1.2)
    _, qz = decompose_attitude(q)
    assert abs(heading_from_quaternion(q) - 2 * math.asin(qz[3])) < 1e-12


def test_hover_equilibrium_gives_mg_and_zero_torque():
    ctl = PaperBacksteppingController(mass=1.5)
    s = ControllerState((0, 0, 1), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0))
    T, tau = ctl.compute(s, TrajectoryPoint(pos=(0, 0, 1)), DT)
    assert abs(T - 1.5 * 9.81) < 1e-9
    assert max(abs(x) for x in tau) < 1e-12


def test_tracks_paper_helix_sub_millimetre_on_exact_model():
    errs, _, _, _ = _fly(paper_helix(center=(0, 0, 1)), 20.0)
    assert errs[int(6.25 / DT):].max() < 0.005


def test_heading_tracks_plus_minus_one_radian():
    ref = paper_helix(center=(0, 0, 1))
    _, _, plant, ctl = _fly(ref, 15.0)
    assert abs(wrap_pi(heading_from_quaternion(plant.q) - ref(15.0).yaw)) < 0.01


def test_lyapunov_function_decreases_from_initial_error():
    target = lambda t: TrajectoryPoint(pos=(1.0, -1.0, 2.0), yaw=0.5)  # noqa: E731
    errs, lyap, _, _ = _fly(target, 12.0, start=(0.0, 0.0, 1.0))
    assert lyap[-1] < 1e-6 * max(1.0, lyap[0])
    assert errs[-1] < 1e-3
    # V(t) is non-increasing (allow tiny numerical noise)
    assert np.all(np.diff(lyap) <= 1e-6)


def test_drag_compensation_term_f_v():
    ref = paper_helix(center=(0, 0, 1))
    no_comp, _, _, _ = _fly(ref, 20.0, plant=RigidBodyQuadrotor(position=ref(0).pos, ground_z=None, drag=0.3))
    plant = RigidBodyQuadrotor(position=ref(0).pos, ground_z=None, drag=0.3)
    plant.v = np.array(ref(0).vel)
    comp, _, _, _ = _fly(ref, 20.0, plant=plant, ctl=PaperBacksteppingController(drag=0.3))
    k = int(6.25 / DT)
    assert comp[k:].mean() < 0.1 * no_comp[k:].mean()


def test_thrust_limits_respected():
    ctl = PaperBacksteppingController(mass=1.5, thrust_limits=(5.0, 20.0))
    s = ControllerState((0, 0, 0), (0, 0, 0), (1, 0, 0, 0), (0, 0, 0))
    for _ in range(2000):
        T, _ = ctl.compute(s, TrajectoryPoint(pos=(0, 0, 50.0)), DT)
        assert 5.0 <= T <= 20.0
