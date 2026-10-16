"""
Guidance layer shared by every simulator.

* Reads the target file (config paths.target_file, written by set_target.py):
      "X Y Z [heading_deg]"  -> fly to a set point (reference model, C^4)
      "MODE auto|mission"    -> takeoff, then hover -> paper helix -> figure-8, looping
      "MODE spiral|figure8|hover|land"
* Switches between modes smoothly (rest-to-rest transfers / reference-model hand-over).
* Obstacle avoidance from the LiDAR scan: carrot steering for set points, smooth offset
  for trajectories.

Parameters: config.yaml -> guidance, trajectory, reference_model (arguments override).
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from .config import cfg, pick
from .obstacle_avoidance import LidarScan, ReactiveObstacleAvoider
from .trajectory import (LoopingMission, Repeating, Sequence, SetpointReferenceModel, SmoothOffset,
                         TrajectoryPoint, rest_move, single_trajectory, transfer_time)

Vec3 = Tuple[float, float, float]
MODE_ALIASES = {"mission": "auto", "racetrack": "figure8", "gates": "figure8"}
MODES = {"auto", "spiral", "figure8", "hover", "land"} | set(MODE_ALIASES)


def parse_target_text(content: str):
    """('coords', (x, y, z), heading_rad) | ('mode', name, 0.0) | (None, None, 0.0)."""
    content = (content or "").strip()
    if not content or content.startswith("#"):
        return None, None, 0.0
    parts = content.split()
    try:
        if parts[0].upper() == "MODE" and len(parts) >= 2:
            return "mode", parts[1].lower(), 0.0
        if parts[0].lower() in MODES:
            return "mode", parts[0].lower(), 0.0
        if len(parts) >= 3:
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            heading = math.radians(float(parts[3])) if len(parts) >= 4 else 0.0
            return "coords", (x, y, z), heading
    except ValueError:
        pass
    return None, None, 0.0


class ReferenceManager:
    def __init__(self, start_pos: Vec3, target_path: Optional[str] = None,
                 hover_point: Optional[Vec3] = None, scale: Optional[float] = None,
                 min_z: Optional[float] = None, land_point: Optional[Vec3] = None,
                 avoider: Optional[ReactiveObstacleAvoider] = None, avoid_gain: Optional[float] = None,
                 max_offset: Optional[float] = None, poll_period: Optional[float] = None,
                 initial_mode: str = "auto", setpoint_speed: Optional[float] = None, planner=None):
        g = "guidance."
        self.start = tuple(start_pos)
        self.target_path = target_path
        self.hover_point = tuple(hover_point) if hover_point is not None else tuple(cfg("trajectory.hover_point"))
        self.scale = pick(scale, "trajectory.scale")
        self.min_z = min(pick(min_z, g + "min_z"), self.start[2])     # never clamp above the take-off point
        land = list(land_point) if land_point is not None else list(cfg(g + "land_point"))
        land[2] = max(land[2], self.min_z)
        self.land_point = tuple(land)
        self.avoider = avoider
        self.planner = planner                       # path_planner.GridPlanner (needs the avoider / carrot)
        self.waypoint_radius = cfg("planner.waypoint_radius")
        self.waypoints = []
        self.final_goal = None
        self.route = []
        self.route_id = 0
        self.avoid_gain = pick(avoid_gain, g + "avoid_gain")
        self.max_offset = pick(max_offset, g + "max_offset")
        self.poll_period = pick(poll_period, g + "poll_period")
        self.setpoint_speed = pick(setpoint_speed, g + "setpoint_speed")
        self.offset_decay_time = cfg(g + "offset_decay_time")
        self.carrot_tau = cfg(g + "carrot_tau")
        self.leash = cfg(g + "carrot_leash")
        self.approach_gain = cfg(g + "carrot_approach_gain")
        self._last_poll = -math.inf
        self._last_text = None
        self.offset = SmoothOffset()
        self._offset_cmd = [0.0, 0.0, 0.0]
        self.avoiding = False
        self.avoid_status = "PATH CLEAR"
        self.mode = None
        self.mode_t0 = 0.0
        self.traj = None
        self.setpoint = SetpointReferenceModel(self.start, v_max=self.setpoint_speed)
        self.base_ref = TrajectoryPoint(pos=self.start)
        self.description = ""
        self.carrot = None
        self.goal = self.start
        self._activate(initial_mode, 0.0, None)

    # ------------------------------------------------------------------
    def _activate(self, mode: str, t: float, data) -> None:
        cur = self.base_ref
        mode = MODE_ALIASES.get(mode, mode)
        if self.mode is not None and mode == self.mode and mode != "coords":
            return                                   # already flying this mode
        if mode == "auto":
            if self.mode is None:
                self.traj = LoopingMission(start=self.start, hover_point=self.hover_point, scale=self.scale)
            else:
                d = transfer_time(cur.pos, self.hover_point)
                cycle = Repeating(LoopingMission(start=cur.pos, hover_point=self.hover_point, scale=self.scale).cycle)
                self.traj = Sequence([("MOVE", d, rest_move(tuple(cur.pos), self.hover_point, d)),
                                      ("AUTO", math.inf, cycle)])
            self.mode, self.description = "auto", "AUTO MISSION"
        elif mode in ("spiral", "figure8", "hover"):
            self.traj = single_trajectory(mode, self.hover_point, self.scale, start=cur.pos)
            self.mode, self.description = mode, mode.upper()
        elif mode in ("coords", "land"):
            if mode == "land":
                target, heading = self.land_point, 0.0
                self.description = "LANDING"
            else:
                target, heading = data
                target = (target[0], target[1], max(self.min_z, target[2]))
                self.description = f"SETPOINT ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})"
            if self.mode not in ("coords", "land"):
                # hand the current reference (all derivatives) to the reference model
                d = cur.derivs()
                for i, ax in enumerate(self.setpoint.axes):
                    ax.state = [d[k][i] if k < len(d) else 0.0 for k in range(ax.n)]
                    ax.cmd = d[0][i]
                yd = cur.yaw_derivs()
                self.setpoint.yaw.state = [yd[k] if k < len(yd) else 0.0 for k in range(self.setpoint.yaw.n)]
                self.setpoint.yaw.cmd = cur.yaw
                self.setpoint.target = tuple(cur.pos)
            self.traj = None
            self.mode = mode
            self.waypoints = []
            self.route = []
            if self.planner is not None and self.avoider is not None:
                start = tuple(cur.pos)
                route = self.planner.plan(start, tuple(target))
                if self.planner.note:
                    print(f"[PLANNER] {self.planner.note}")
                target = route[-1]
                self.waypoints = list(route)
                self.route = [start] + list(route)          # full planned curve (for displays)
                self.route_id += 1
                if mode != "land":
                    self.description = f"SETPOINT ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})"
            self.final_goal = tuple(target)
            self.goal = tuple(self.waypoints.pop(0)) if self.waypoints else tuple(target)
            if self.avoider is not None:
                self.carrot = list(self.setpoint.target)
                for ax in self.setpoint.axes:
                    ax.tau = self.carrot_tau
                self.setpoint.set_target(self.setpoint.target, heading)
            else:
                self.carrot = None
                self.setpoint.set_target(target, heading)
        else:
            return
        if mode not in ("coords", "land") and self.route:
            self.route = []                                  # trajectories: no planned route to show
            self.route_id += 1
        self.mode_t0 = t

    def command(self, text: str, t: float) -> None:
        """Apply a set_target.py style command directly (without the file)."""
        kind, data, heading = parse_target_text(text)
        if kind == "coords":
            self._activate("coords", t, (data, heading))
        elif kind == "mode":
            self._activate(data, t, None)

    def poll(self, t: float) -> None:
        if self.target_path is None or t - self._last_poll < self.poll_period:
            return
        self._last_poll = t
        try:
            with open(self.target_path, "r") as f:
                text = f.read().strip()
        except OSError:
            return
        if text == self._last_text:
            return
        self._last_text = text
        self.command(text, t)

    # ------------------------------------------------------------------
    def update(self, t: float, dt: float, scan: Optional[LidarScan] = None,
               heading: float = 0.0) -> TrajectoryPoint:
        """Reference for time t (call once per control step)."""
        self.poll(t)
        self.avoiding = False
        self.avoid_status = "PATH CLEAR"
        if self.traj is None and self.carrot is not None:
            return self._carrot_update(dt, scan, heading)
        base = self.traj(t - self.mode_t0) if self.traj is not None else self.setpoint.step(dt)
        self.base_ref = base

        if scan is not None and self.avoider is not None:
            safe_v, self.avoiding, self.avoid_status = self.avoider.compute_avoidance_velocity(
                lidar=scan, current_vel=base.vel, desired_vel=base.vel, heading=heading,
                ground_check=self.mode != "land")
            if self.avoiding:
                for i in range(3):
                    self._offset_cmd[i] += self.avoid_gain * (safe_v[i] - base.vel[i]) * dt
            else:
                decay = math.exp(-dt / self.offset_decay_time)
                self._offset_cmd = [c * decay for c in self._offset_cmd]
            n = math.sqrt(sum(c * c for c in self._offset_cmd))
            if n > self.max_offset:
                self._offset_cmd = [c * self.max_offset / n for c in self._offset_cmd]
            self.offset.set_command(tuple(self._offset_cmd))
        ref = base.shifted(self.offset.step(dt))
        if ref.pos[2] < self.min_z and self.mode != "land":
            ref = TrajectoryPoint((ref.pos[0], ref.pos[1], self.min_z), ref.vel, ref.acc, ref.jerk,
                                  ref.snap, ref.yaw, ref.yaw_rate, ref.yaw_acc)
        return ref

    def _carrot_update(self, dt: float, scan, heading: float) -> TrajectoryPoint:
        """Set point with avoidance: a lead point moves toward the goal at setpoint_speed,
        is steered by the avoider and kept within carrot_leash of the reference; the
        reference model follows it (C^4)."""
        c = self.carrot
        to_goal = [self.goal[i] - c[i] for i in range(3)]
        dist = math.sqrt(sum(v * v for v in to_goal))
        if self.waypoints and dist < self.waypoint_radius:          # next waypoint of the planned route
            self.goal = tuple(self.waypoints.pop(0))
            to_goal = [self.goal[i] - c[i] for i in range(3)]
            dist = math.sqrt(sum(v * v for v in to_goal))
        speed = self.setpoint_speed if self.waypoints else min(self.setpoint_speed, self.approach_gain * dist)
        v_nom = tuple(to_goal[i] / dist * speed for i in range(3)) if dist > 0.0 else (0.0, 0.0, 0.0)
        v_cmd = v_nom
        if scan is not None and self.avoider is not None:
            v_cmd, self.avoiding, self.avoid_status = self.avoider.compute_avoidance_velocity(
                lidar=scan, current_vel=v_nom, desired_vel=v_nom, heading=heading,
                ground_check=self.mode != "land")
        new = [c[i] + v_cmd[i] * dt for i in range(3)]
        new[2] = max(self.land_point[2] if self.mode == "land" else self.min_z, new[2])
        ref_pos = [ax.state[0] for ax in self.setpoint.axes]
        off = [new[i] - ref_pos[i] for i in range(3)]
        n = math.sqrt(sum(v * v for v in off))
        if n > self.leash:
            new = [ref_pos[i] + off[i] * self.leash / n for i in range(3)]
        self.carrot = new
        for i, ax in enumerate(self.setpoint.axes):
            ax.cmd = new[i]
        self.setpoint.target = tuple(new)
        ref = self.setpoint.step(dt)
        self.base_ref = ref
        return ref

    def phase(self, t: float) -> str:
        if self.mode == "auto" and isinstance(self.traj, LoopingMission):
            return self.traj.phase_name(t - self.mode_t0)
        if isinstance(self.traj, Sequence):
            tt = t - self.mode_t0
            i = self.traj._find(tt)
            name, _, fn = self.traj.parts[i]
            if hasattr(fn, "phase_name"):
                return fn.phase_name(tt - self.traj.starts[i])
            return name
        return self.description
