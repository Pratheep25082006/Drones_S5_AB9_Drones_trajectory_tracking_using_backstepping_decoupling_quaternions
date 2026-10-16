"""
Python flight core used by the Gazebo runners and the web telemetry bridge.

Gazebo's quadrotor model (models/quadrotor.world) is kinematic: Gazebo only
renders the pose.  The motion itself is produced here by integrating the rigid
body model (paper eqs. (25)-(28)) at 500 Hz with the controller's thrust and
torques, with the same sensor noise, Kalman filter and guidance as the MuJoCo
and PyBullet simulations.  Obstacles: a LiDAR ray-cast on the generated city
(world_map.WorldMap - the same city Gazebo shows), the reactive avoider and the
global path planner (routes around blocks, targets inside buildings are moved out).
"""

from __future__ import annotations

import math
from typing import Dict, Optional

from .config import cfg, pick
from .units import CM_PER_M
from .paper_controller import heading_from_quaternion
from .dynamics import RigidBodyQuadrotor
from .guidance import ReferenceManager
from .kalman_filter import QuadrotorStateEstimator, SensorNoiseModel
from .obstacle_avoidance import ReactiveObstacleAvoider
from .paper_controller import PaperBacksteppingController
from .path_planner import GridPlanner
from .world_map import WorldMap


class PythonFlightSim:
    """Arguments left as None come from config.yaml (python_sim.*, vehicle.*, noise.*, estimator.*)."""

    def __init__(self, target_path: Optional[str] = None, physics_dt: Optional[float] = None,
                 mass: Optional[float] = None, inertia=None, start=None, noise: Optional[bool] = None,
                 use_filter: Optional[bool] = None, seed: Optional[int] = None,
                 initial_command: Optional[str] = None, avoid: Optional[bool] = None):
        self.dt = pick(physics_dt, "python_sim.physics_dt")
        start = tuple(pick(start, "python_sim.start"))
        self.plant = RigidBodyQuadrotor(mass=mass, inertia=inertia, position=start,
                                        ground_z=cfg("python_sim.ground_z"))
        self.ctl = PaperBacksteppingController(mass=mass, inertia=inertia)
        self.noise = SensorNoiseModel(enabled=noise, seed=seed)
        self.use_filter = pick(use_filter, "estimator.enabled")
        self.est = QuadrotorStateEstimator(dt=self.dt, initial_pos=start)
        ps = "python_sim."
        self.avoid = pick(avoid, ps + "avoid")
        self.world = WorldMap.from_city() if self.avoid else None
        self.lidar_range, self.lidar_period = cfg(ps + "lidar_range"), cfg(ps + "lidar_period")
        self.beams = cfg("avoidance.lidar_beams")
        self.scan = None
        self._next_scan = 0.0
        self.ref = ReferenceManager(
            start_pos=start, target_path=target_path,
            avoider=ReactiveObstacleAvoider(**cfg(ps + "avoidance")) if self.avoid else None,
            planner=GridPlanner(self.world) if self.avoid and cfg("planner.enabled") else None,
            avoid_gain=cfg(ps + "avoid_gain"), setpoint_speed=cfg(ps + "setpoint_speed"))
        self.meas_pos = start
        self.t = 0.0
        self.thrust = self.ctl.mass * self.ctl.g
        self.torque = (0.0, 0.0, 0.0)
        self.last_ref = None
        self.est_pos = start
        if initial_command:
            self.command(initial_command)

    def command(self, text: str) -> None:
        """Inject a set_target.py style command ("X Y Z [deg]" or "MODE name")."""
        self.ref.command(text, self.t)

    def advance(self, duration: float) -> None:
        n = max(1, int(round(duration / self.dt)))
        for _ in range(n):
            true = self.plant.state()
            if self.world is not None and self.t >= self._next_scan:
                self._next_scan = self.t + self.lidar_period
                self.scan = self.world.lidar(true.position, heading_from_quaternion(true.quaternion),
                                             self.beams, self.lidar_range)
            ref = self.ref.update(self.t, self.dt, self.scan, heading_from_quaternion(true.quaternion))
            meas = self.noise.apply_noise(true)
            fb = (self.est.estimate(meas, dt=self.dt, acc_control=self.ctl.model_acceleration(meas.quaternion))
                  if self.use_filter else meas)
            self.thrust, self.torque = self.ctl.compute(fb, ref, self.dt)
            self.plant.step(self.thrust, self.torque, self.dt)
            self.t += self.dt
            self.last_ref = ref
            self.est_pos = fb.position
            self.meas_pos = meas.position

    def telemetry(self) -> Dict:
        p = self.plant
        ref = self.last_ref
        rp = ref.pos if ref is not None else tuple(p.x)
        return {
            "time": round(self.t, 2),
            "mode": self.ref.mode,
            "phase": self.ref.phase(self.t),
            "pos": [round(float(v), 3) for v in p.x],
            "ref": [round(float(v), 3) for v in rp],
            "vel": [round(float(v), 3) for v in p.v],
            "quat": [round(float(v), 4) for v in p.q],
            "rates": [round(float(v), 3) for v in p.w],
            "thrust": round(float(self.thrust), 3),
            "torques": [round(float(v), 4) for v in self.torque],
            "error": round(math.dist(tuple(p.x), rp), 4),
            "est_error": round(math.dist(tuple(p.x), self.est_pos), 4),
            "noise": round(math.dist(tuple(p.x), self.meas_pos), 4),
            "lidar": (self.ref.avoid_status if self.ref.avoiding else
                      (f"clear {self.scan.min_horizontal_dist:.1f}m" if self.scan is not None else "off")),
            "goal": [round(float(v), 3) for v in (self.ref.final_goal or rp)],
            "waypoints_left": len(self.ref.waypoints),
            "lyapunov": round(float(self.ctl.last.get("lyapunov", 0.0)), 5),
        }

    def marker_position(self):
        """Target marker: the commanded (possibly moved) goal for set points, else the reference."""
        if self.ref.mode in ("coords", "land") and self.ref.final_goal is not None:
            return tuple(self.ref.final_goal)
        return tuple(self.last_ref.pos) if self.last_ref is not None else tuple(self.plant.x)

    def status_line(self) -> str:
        tm = self.telemetry()
        p, r = tm["pos"], tm["ref"]
        lid = f"[LIDAR {tm['lidar']}]" if self.ref.avoiding else f"LiDAR {tm['lidar']}"
        route = f" | route {tm['waypoints_left']} wp" if tm["waypoints_left"] else ""
        return (f"[{tm['phase']:20s} t={tm['time']:6.1f}s] Pos ({p[0]:5.2f},{p[1]:5.2f},{p[2]:5.2f}) "
                f"Ref ({r[0]:5.2f},{r[1]:5.2f},{r[2]:5.2f}) err {tm['error'] * CM_PER_M:5.1f}cm | T {tm['thrust']:.2f}N | "
                f"KF {tm['est_error'] * CM_PER_M:4.1f}cm (noise {tm['noise'] * CM_PER_M:4.1f}cm) | {lid}{route}")
