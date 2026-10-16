"""
Reference trajectories for the De Monte & Lohmann (2013) backstepping controller.

The control law needs x_T and its first four derivatives (x_T in C^4) and the
heading phi_T with two derivatives (phi_T in C^2).  Every trajectory here returns
a ``TrajectoryPoint`` carrying those derivatives analytically.

Frame: ENU (z up), heading about +z.  All shape/timing parameters come from
config.yaml (trajectory.*, reference_model.*, guidance.offset_tau); function
arguments override them.

* ``paper_helix`` (paper Fig. 4), ``figure8``, ``hover``, ``takeoff``/``rest_move``
* ``ramped``      : time warp with smooth speed ramps (starts/ends at rest, C^4)
* ``Sequence``    : piecewise rest-to-rest segments;  ``LoopingMission`` built from them
* ``MissionTrajectory`` : C^4 blending of overlapping segments (weights sum to 1)
* ``SetpointReferenceModel`` : (tau s + 1)^-n reference model for set points (paper Fig. 3)
* ``SmoothOffset`` : smooth 3D offset (obstacle avoidance on trajectories)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence as Seq, Tuple

from .config import cfg, pick

Vec3 = Tuple[float, float, float]
ZERO3: Vec3 = (0.0, 0.0, 0.0)
DERIVATIVE_ORDER = 4          # x_T in C^4 (paper Sec. III-A)
YAW_DERIVATIVE_ORDER = 2      # phi_T in C^2 (paper Sec. III-B)


@dataclass
class TrajectoryPoint:
    """Reference x_T, its derivatives up to 4th order, and heading phi_T up to 2nd."""

    pos: Vec3 = ZERO3
    vel: Vec3 = ZERO3
    acc: Vec3 = ZERO3
    jerk: Vec3 = ZERO3
    snap: Vec3 = ZERO3
    yaw: float = 0.0
    yaw_rate: float = 0.0
    yaw_acc: float = 0.0

    def derivs(self) -> List[Vec3]:
        return [self.pos, self.vel, self.acc, self.jerk, self.snap]

    def yaw_derivs(self) -> List[float]:
        return [self.yaw, self.yaw_rate, self.yaw_acc]

    @staticmethod
    def from_derivs(d: Seq[Vec3], y: Seq[float]) -> "TrajectoryPoint":
        return TrajectoryPoint(tuple(d[0]), tuple(d[1]), tuple(d[2]), tuple(d[3]), tuple(d[4]),
                               float(y[0]), float(y[1]), float(y[2]))

    def shifted(self, offset: "TrajectoryPoint") -> "TrajectoryPoint":
        d = [_add(a, b) for a, b in zip(self.derivs(), offset.derivs())]
        return TrajectoryPoint.from_derivs(d, self.yaw_derivs())


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _hover_point() -> Vec3:
    return tuple(cfg("trajectory.hover_point"))


# ---------------------------------------------------------------------------
# Elementary trajectories
# ---------------------------------------------------------------------------

def _sin_derivs(amp: float, w: float, t: float, phase: float = 0.0) -> List[float]:
    """d^k/dt^k [amp sin(w t + phase)] for k = 0..4."""
    a = w * t + phase
    s, c = math.sin(a), math.cos(a)
    return [amp * s, amp * w * c, -amp * w ** 2 * s, -amp * w ** 3 * c, amp * w ** 4 * s]


def hover(point: Optional[Vec3] = None, yaw: float = 0.0) -> Callable[[float], TrajectoryPoint]:
    point = tuple(point) if point is not None else _hover_point()

    def f(t: float) -> TrajectoryPoint:
        return TrajectoryPoint(pos=point, yaw=yaw)
    return f


def paper_helix(center: Optional[Vec3] = None, amp_x=None, amp_y=None, amp_z=None, period_xy=None,
                period_z=None, yaw_amp=None, scale: Optional[float] = None) -> Callable[[float], TrajectoryPoint]:
    """Vertical helix of the paper's Fig. 4 (config trajectory.helix): y lags x by a
    quarter of the horizontal period; height and heading share the second period."""
    center = tuple(center) if center is not None else _hover_point()
    sc = pick(scale, "trajectory.scale")
    ax = pick(amp_x, "trajectory.helix.amp_x") * sc
    ay = pick(amp_y, "trajectory.helix.amp_y") * sc
    az = pick(amp_z, "trajectory.helix.amp_z")
    wxy = 2.0 * math.pi / pick(period_xy, "trajectory.helix.period_xy")
    wz = 2.0 * math.pi / pick(period_z, "trajectory.helix.period_z")
    ya = pick(yaw_amp, "trajectory.helix.yaw_amp")

    def f(t: float) -> TrajectoryPoint:
        xs = _sin_derivs(ax, wxy, t)
        ys = _sin_derivs(ay, wxy, t, -math.pi / 2.0)     # quarter-period lag
        zs = _sin_derivs(az, wz, t)
        ps = _sin_derivs(ya, wz, t)
        d = [(xs[k], ys[k], zs[k]) for k in range(DERIVATIVE_ORDER + 1)]
        d[0] = _add(d[0], center)
        return TrajectoryPoint.from_derivs(d, ps[:YAW_DERIVATIVE_ORDER + 1])
    f.period = 2.0 * math.pi / wxy
    return f


def figure8(center: Optional[Vec3] = None, amp_x=None, amp_y=None, amp_z=None, period=None,
            yaw_amp=None, scale: Optional[float] = None) -> Callable[[float], TrajectoryPoint]:
    """3D lemniscate (Lissajous 1:2) - project extension (config trajectory.figure8)."""
    if center is None:
        h = _hover_point()
        center = (h[0], h[1], h[2] + cfg("trajectory.figure8.height_offset"))
    sc = pick(scale, "trajectory.scale")
    ax = pick(amp_x, "trajectory.figure8.amp_x") * sc
    ay = pick(amp_y, "trajectory.figure8.amp_y") * sc
    az = pick(amp_z, "trajectory.figure8.amp_z")
    w = 2.0 * math.pi / pick(period, "trajectory.figure8.period")
    ya = pick(yaw_amp, "trajectory.figure8.yaw_amp")

    def f(t: float) -> TrajectoryPoint:
        xs = _sin_derivs(ax, w, t)
        ys = _sin_derivs(ay, 2.0 * w, t)
        zs = _sin_derivs(az, w, t, math.pi / 2.0)         # cosine
        ps = _sin_derivs(ya, w, t)
        d = [(xs[k], ys[k], zs[k]) for k in range(DERIVATIVE_ORDER + 1)]
        d[0] = _add(d[0], tuple(center))
        return TrajectoryPoint.from_derivs(d, ps[:YAW_DERIVATIVE_ORDER + 1])
    f.period = 2.0 * math.pi / w
    return f


# ---------------------------------------------------------------------------
# Smoothstep of order DERIVATIVE_ORDER (all derivatives up to 4 vanish at 0 and 1)
# ---------------------------------------------------------------------------

def _smoothstep_coefficients(n: int) -> List[float]:
    """Coefficients c_k of S(u) = sum c_k u^k, the polynomial of degree 2n+1 with
    S(0)=0, S(1)=1 and derivatives 1..n zero at both ends."""
    c = [0.0] * (2 * n + 2)
    for k in range(n + 1):
        c[n + 1 + k] = (-1) ** k * math.comb(n + k, k) * math.comb(2 * n + 1, n + 1 + k)
    return c


_S_COEF = _smoothstep_coefficients(DERIVATIVE_ORDER)
_S_INT = [0.0] + [c / (k + 1) for k, c in enumerate(_S_COEF)]       # coefficients of int_0^u S


def smoothstep9(u: float) -> List[float]:
    """S(u) and its first 4 derivatives w.r.t. u (degree-9 smoothstep)."""
    if u <= 0.0:
        return [0.0] * (DERIVATIVE_ORDER + 1)
    if u >= 1.0:
        return [1.0] + [0.0] * DERIVATIVE_ORDER
    out = []
    for n in range(DERIVATIVE_ORDER + 1):
        val = 0.0
        for k in range(n, len(_S_COEF)):
            if _S_COEF[k] == 0:
                continue
            val += _S_COEF[k] * math.perm(k, n) * u ** (k - n)
        out.append(val)
    return out


def _smoothstep_integral(u: float) -> float:
    if u <= 0.0:
        return 0.0
    if u >= 1.0:
        return sum(_S_INT) + (u - 1.0)
    return sum(c * u ** k for k, c in enumerate(_S_INT) if c)


def _ramp(t: float, start: float, length: float) -> List[float]:
    s = smoothstep9((t - start) / length)
    return [s[k] / length ** k for k in range(DERIVATIVE_ORDER + 1)]


# ---------------------------------------------------------------------------
# C^4 blending of overlapping segments
# ---------------------------------------------------------------------------

@dataclass
class Segment:
    start: float
    end: float
    traj: Callable[[float], TrajectoryPoint]
    local_time: bool = True
    name: str = ""


class MissionTrajectory:
    """Segments joined with C^4 blends: w_i = S((t-a_i)/Tb) - S((t-b_i)/Tb), sum w_i = 1,
    r = sum w_i r_i differentiated with the Leibniz rule."""

    def __init__(self, segments: List[Segment], blend: float):
        self.segments = segments
        self.blend = blend
        self.tol = cfg("trajectory.zero_weight_tolerance")

    @property
    def duration(self) -> float:
        return self.segments[-1].end

    def phase_name(self, t: float) -> str:
        for s in self.segments:
            if s.start <= t < s.end:
                return s.name
        return self.segments[-1].name

    def __call__(self, t: float) -> TrajectoryPoint:
        n = len(self.segments)
        N = DERIVATIVE_ORDER + 1
        pos_d = [[0.0, 0.0, 0.0] for _ in range(N)]
        yaw_d = [0.0] * (YAW_DERIVATIVE_ORDER + 1)
        for i, seg in enumerate(self.segments):
            rise = [1.0] + [0.0] * DERIVATIVE_ORDER if i == 0 else _ramp(t, seg.start, self.blend)
            fall = [0.0] * N if i == n - 1 else _ramp(t, seg.end, self.blend)
            w = [rise[k] - fall[k] for k in range(N)]
            if all(abs(x) < self.tol for x in w):
                continue
            p = seg.traj(t - seg.start if seg.local_time else t)
            pd, yd = p.derivs(), p.yaw_derivs()
            for order in range(N):
                for k in range(order + 1):
                    c = math.comb(order, k) * w[k]
                    if c:
                        for ax in range(3):
                            pos_d[order][ax] += c * pd[order - k][ax]
            for order in range(YAW_DERIVATIVE_ORDER + 1):
                for k in range(order + 1):
                    yaw_d[order] += math.comb(order, k) * w[k] * yd[order - k]
        return TrajectoryPoint.from_derivs([tuple(v) for v in pos_d], yaw_d)


# ---------------------------------------------------------------------------
# Rest-to-rest moves, speed ramps, sequences, missions
# ---------------------------------------------------------------------------

def takeoff(start: Vec3, goal: Vec3, duration: float) -> Callable[[float], TrajectoryPoint]:
    """C^4 rest-to-rest move from start to goal (also used for transfers)."""
    delta = (goal[0] - start[0], goal[1] - start[1], goal[2] - start[2])

    def f(t: float) -> TrajectoryPoint:
        s = smoothstep9(t / duration)
        d = [_scale(delta, s[k] / duration ** k) for k in range(DERIVATIVE_ORDER + 1)]
        d[0] = _add(d[0], tuple(start))
        return TrajectoryPoint.from_derivs(d, [0.0] * (YAW_DERIVATIVE_ORDER + 1))
    return f


rest_move = takeoff


def ramped(traj: Callable[[float], TrajectoryPoint], length: float, ramp: Optional[float] = None,
           ramp_out: bool = True) -> Callable[[float], TrajectoryPoint]:
    """Time-warped r(tau(t)): d(tau)/dt rises 0 -> 1 over ``ramp`` (C^4 smoothstep) and,
    with ramp_out, falls back to 0 so the trajectory ends at rest.  Covers ``length``
    seconds of trajectory time in length + ramp seconds.  Derivatives by Faa di Bruno."""
    ramp = pick(ramp, "trajectory.mission.speed_ramp_time")
    total = length + ramp

    def tau_derivs(t: float) -> List[float]:
        up = smoothstep9(t / ramp)
        tau = ramp * _smoothstep_integral(t / ramp)
        d = [up[0]] + [up[k] / ramp ** k for k in range(1, DERIVATIVE_ORDER)]
        if ramp_out:
            u2 = (t - (total - ramp)) / ramp
            if u2 > 0.0:
                dn = smoothstep9(u2)
                tau -= ramp * _smoothstep_integral(u2)
                d = [d[0] - dn[0]] + [d[k] - dn[k] / ramp ** k for k in range(1, DERIVATIVE_ORDER)]
        return [tau] + d

    def f(t: float) -> TrajectoryPoint:
        if ramp_out:
            t = min(max(t, 0.0), total)
        tau, t1, t2, t3, t4 = tau_derivs(max(t, 0.0))
        p = traj(tau)
        r1, r2, r3, r4 = p.vel, p.acc, p.jerk, p.snap
        y0, y1, y2 = p.yaw_derivs()
        out = [p.pos,
               _scale(r1, t1),
               _add(_scale(r2, t1 ** 2), _scale(r1, t2)),
               _add(_add(_scale(r3, t1 ** 3), _scale(r2, 3 * t1 * t2)), _scale(r1, t3)),
               _add(_add(_scale(r4, t1 ** 4), _scale(r3, 6 * t1 ** 2 * t2)),
                    _add(_scale(r2, 3 * t2 ** 2 + 4 * t1 * t3), _scale(r1, t4)))]
        return TrajectoryPoint.from_derivs(out, [y0, y1 * t1, y2 * t1 ** 2 + y1 * t2])

    f.duration = total
    return f


class Sequence:
    """Piecewise trajectory of segments that start and end at rest (C^4 joints)."""

    def __init__(self, parts):
        self.parts = parts
        self.starts = []
        t = 0.0
        for _, dur, _ in parts:
            self.starts.append(t)
            t += dur
        self.duration = t

    def _find(self, t: float) -> int:
        for i in range(len(self.parts) - 1, -1, -1):
            if t >= self.starts[i]:
                return i
        return 0

    def __call__(self, t: float) -> TrajectoryPoint:
        i = self._find(t)
        _, dur, fn = self.parts[i]
        return fn(min(t - self.starts[i], dur))

    def phase_name(self, t: float) -> str:
        return self.parts[self._find(t)][0]


class Repeating:
    """Repeat a trajectory that starts and ends in the same hover point."""

    def __init__(self, mission):
        self.m = mission

    def __call__(self, t: float) -> TrajectoryPoint:
        return self.m(t % self.m.duration)

    def phase_name(self, t: float) -> str:
        return self.m.phase_name(t % self.m.duration)


def transfer_time(a: Vec3, b: Vec3) -> float:
    return max(cfg("trajectory.transfer_min_time"), cfg("trajectory.transfer_time_per_metre") * math.dist(a, b))


def _cycle_parts(hover_point: Optional[Vec3] = None, scale: Optional[float] = None):
    h = tuple(hover_point) if hover_point is not None else _hover_point()
    m = "trajectory.mission."
    helix = paper_helix(center=h, scale=scale)
    fig8 = figure8(center=(h[0], h[1], h[2] + cfg("trajectory.figure8.height_offset")), scale=scale)
    helix_start, fig8_start = helix(0.0).pos, fig8(0.0).pos
    helix_r = ramped(helix, cfg(m + "helix_periods") * helix.period)
    fig8_r = ramped(fig8, cfg(m + "figure8_periods") * fig8.period)
    tt = cfg(m + "transfer_time")
    return [
        ("HOVER", cfg(m + "hover_time"), hover(h)),
        ("TRANSFER", tt, rest_move(h, helix_start, tt)),
        ("HELIX (paper Fig. 4)", helix_r.duration, helix_r),
        ("TRANSFER", tt, rest_move(helix_start, fig8_start, tt)),
        ("FIGURE-8", fig8_r.duration, fig8_r),
        ("TRANSFER", tt, rest_move(fig8_start, h, tt)),
    ]


class LoopingMission:
    """Takeoff once, then repeat hover -> paper helix (Fig. 4) -> figure-8 forever.
    Every manoeuvre starts and ends at rest, so the reference is C^4 everywhere."""

    def __init__(self, start: Optional[Vec3] = None, hover_point: Optional[Vec3] = None,
                 scale: Optional[float] = None):
        start = tuple(start) if start is not None else tuple(cfg("python_sim.start"))
        h = tuple(hover_point) if hover_point is not None else _hover_point()
        tk = cfg("trajectory.mission.takeoff_time")
        self.first = Sequence([("TAKEOFF", tk, takeoff(start, h, tk))])
        self.cycle = Sequence(_cycle_parts(h, scale))
        self.t0 = self.first.duration

    def _resolve(self, t: float):
        if t < self.t0:
            return self.first, t
        return self.cycle, (t - self.t0) % self.cycle.duration

    def __call__(self, t: float) -> TrajectoryPoint:
        m, tt = self._resolve(t)
        return m(tt)

    def phase_name(self, t: float) -> str:
        m, tt = self._resolve(t)
        return m.phase_name(tt)


def standard_mission(start: Optional[Vec3] = None, hover_point: Optional[Vec3] = None,
                     scale: Optional[float] = None) -> Sequence:
    """One pass: takeoff -> hover -> helix -> figure-8 -> back to hover."""
    lm = LoopingMission(start, hover_point, scale)
    h = tuple(hover_point) if hover_point is not None else _hover_point()
    return Sequence(lm.first.parts + lm.cycle.parts + [("HOVER", math.inf, hover(h))])


def single_trajectory(kind: str, hover_point: Optional[Vec3] = None, scale: Optional[float] = None,
                      start: Optional[Vec3] = None):
    """Modes 'spiral' / 'figure8' / 'hover': move (rest-to-rest) from ``start`` to the
    trajectory's first point, then fly it forever with a smooth speed ramp-in."""
    h = tuple(hover_point) if hover_point is not None else _hover_point()
    if kind == "spiral":
        core = paper_helix(center=h, scale=scale)
    elif kind == "figure8":
        core = figure8(center=(h[0], h[1], h[2] + cfg("trajectory.figure8.height_offset")), scale=scale)
    else:
        if start is None:
            return hover(h)
        d = transfer_time(start, h)
        return Sequence([("MOVE", d, rest_move(tuple(start), h, d)), ("HOVER", math.inf, hover(h))])
    p0 = core(0.0).pos
    run = ramped(core, math.inf, ramp_out=False)
    if start is None:
        return run
    d = transfer_time(start, p0)
    return Sequence([("MOVE", d, rest_move(tuple(start), p0, d)), (kind.upper(), math.inf, run)])


# ---------------------------------------------------------------------------
# Reference models (set points -> smooth reference)
# ---------------------------------------------------------------------------

class _ChainFilter:
    """n-th order critically damped filter  X/Xc = 1/(tau s + 1)^n , RK4 integrated."""

    def __init__(self, order: int, tau: float, x0: float = 0.0):
        self.n = order
        self.tau = tau
        self.state = [x0] + [0.0] * (order - 1)   # x, x', ..., x^(n-1)
        self.cmd = x0

    def _coeffs(self):
        a = 1.0 / self.tau
        return [math.comb(self.n, k) * a ** (self.n - k) for k in range(self.n + 1)]

    def _f(self, st):
        c = self._coeffs()
        top = c[0] * (self.cmd - st[0])
        for k in range(1, self.n):
            top -= c[k] * st[k]
        return st[1:] + [top / c[self.n]]

    def highest(self) -> float:
        return self._f(self.state)[-1]

    def step(self, dt: float) -> None:
        s = self.state
        k1 = self._f(s)
        k2 = self._f([s[i] + 0.5 * dt * k1[i] for i in range(self.n)])
        k3 = self._f([s[i] + 0.5 * dt * k2[i] for i in range(self.n)])
        k4 = self._f([s[i] + dt * k3[i] for i in range(self.n)])
        self.state = [s[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(self.n)]

    def derivs(self) -> List[float]:
        return self.state + [self.highest()]


def _peak_speed_factor(n: int) -> float:
    """Peak of the impulse response of 1/(tau s+1)^n times tau: (n-1)^(n-1) e^-(n-1) / (n-1)!."""
    return (n - 1) ** (n - 1) * math.exp(-(n - 1)) / math.factorial(n - 1)


class SetpointReferenceModel:
    """Set points -> smooth reference (paper Sec. IV, Fig. 3): position filter of order
    reference_model.position_order (5 -> C^4), heading filter of order yaw_order (3 -> C^2).
    The time constant grows with the step size so the peak speed stays below v_max."""

    def __init__(self, initial_pos: Optional[Vec3] = None, initial_yaw: float = 0.0,
                 tau_min: Optional[float] = None, v_max: Optional[float] = None,
                 yaw_tau: Optional[float] = None):
        initial_pos = tuple(initial_pos) if initial_pos is not None else tuple(cfg("python_sim.start"))
        self.order = int(cfg("reference_model.position_order"))
        yaw_order = int(cfg("reference_model.yaw_order"))
        if self.order < DERIVATIVE_ORDER + 1 or yaw_order < YAW_DERIVATIVE_ORDER + 1:
            raise ValueError("reference_model orders must be >= 5 (position) and >= 3 (heading)")
        self.tau_min = pick(tau_min, "reference_model.tau_min")
        self.v_max = pick(v_max, "reference_model.v_max")
        self.peak_factor = _peak_speed_factor(self.order)
        self.axes = [_ChainFilter(self.order, self.tau_min, p) for p in initial_pos]
        self.yaw = _ChainFilter(yaw_order, pick(yaw_tau, "reference_model.yaw_tau"), initial_yaw)
        self.target: Vec3 = tuple(initial_pos)
        self.target_yaw = initial_yaw

    def reset(self, pos: Vec3, yaw: float = 0.0, vel: Vec3 = ZERO3) -> None:
        for i, ax in enumerate(self.axes):
            ax.state = [pos[i], vel[i]] + [0.0] * (ax.n - 2)
            ax.cmd = pos[i]
        self.yaw.state = [yaw] + [0.0] * (self.yaw.n - 1)
        self.yaw.cmd = yaw
        self.target = tuple(pos)
        self.target_yaw = yaw

    def set_target(self, pos: Vec3, yaw: Optional[float] = None) -> None:
        pos = tuple(float(p) for p in pos)
        if pos != tuple(self.target):
            cur = [ax.state[0] for ax in self.axes]
            tau = max(self.tau_min, self.peak_factor * math.dist(pos, cur) / self.v_max)
            for i, ax in enumerate(self.axes):
                ax.tau = tau
                ax.cmd = pos[i]
            self.target = pos
        if yaw is not None:
            cur = self.yaw.state[0]
            self.yaw.cmd = cur + ((yaw - cur + math.pi) % (2.0 * math.pi) - math.pi)   # short way
            self.target_yaw = yaw

    def step(self, dt: float) -> TrajectoryPoint:
        for ax in self.axes:
            ax.step(dt)
        self.yaw.step(dt)
        return self.current()

    def current(self) -> TrajectoryPoint:
        d = [ax.derivs() for ax in self.axes]
        derivs = [(d[0][k], d[1][k], d[2][k]) for k in range(DERIVATIVE_ORDER + 1)]
        return TrajectoryPoint.from_derivs(derivs, self.yaw.derivs()[:YAW_DERIVATIVE_ORDER + 1])


class SmoothOffset:
    """C^4 smooth 3D offset (obstacle-avoidance detours on trajectories)."""

    def __init__(self, tau: Optional[float] = None):
        order = int(cfg("reference_model.position_order"))
        self.axes = [_ChainFilter(order, pick(tau, "guidance.offset_tau"), 0.0) for _ in range(3)]
        self.cmd = [0.0, 0.0, 0.0]

    def set_command(self, cmd: Vec3) -> None:
        self.cmd = list(cmd)
        for i, ax in enumerate(self.axes):
            ax.cmd = cmd[i]

    def step(self, dt: float) -> TrajectoryPoint:
        for ax in self.axes:
            ax.step(dt)
        d = [ax.derivs() for ax in self.axes]
        return TrajectoryPoint.from_derivs([(d[0][k], d[1][k], d[2][k]) for k in range(DERIVATIVE_ORDER + 1)],
                                           [0.0] * (YAW_DERIVATIVE_ORDER + 1))
