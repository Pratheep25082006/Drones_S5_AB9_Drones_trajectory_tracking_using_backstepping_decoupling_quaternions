"""
Omnidirectional LiDAR scan + reactive obstacle avoidance.

  1. LiDAR data structure: N horizontal rays (evenly spaced, heading frame) + up/down rays.
  2. Artificial potential field (repulsion) + tangential circulation (wall following).
  3. Side memory (hysteresis) and optional "ignore obstacles behind the motion".

All parameters come from config.yaml -> avoidance (constructor arguments override them).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .config import cfg, section
from .units import DEG_PER_TURN


@dataclass
class LidarScan:
    """Ranges of the horizontal rays (index i at angle i * 360/N deg from the heading,
    counter-clockwise: forward, forward-left, left, ...) and the up/down clearances.
    Values <= 0 mean 'no hit'."""

    ranges: List[float]
    dist_up: float
    dist_down: float
    max_range: float = field(default_factory=lambda: cfg("avoidance.lidar_range"))

    @property
    def min_horizontal_dist(self) -> float:
        valid = [r for r in self.ranges if r > 0.0]
        return min(valid) if valid else self.max_range

    @property
    def closest_sector(self) -> int:
        valid_ranges = [r if r > 0.0 else self.max_range for r in self.ranges]
        return int(np.argmin(valid_ranges))


def sector_name(index: int, beams: int) -> str:
    """Readable name of a horizontal ray (8 beams: FWD, FWD-L, LEFT, ...)."""
    names8 = ["FWD", "FWD-L", "LEFT", "REAR-L", "REAR", "REAR-R", "RIGHT", "FWD-R"]
    if beams == len(names8):
        return names8[index]
    return f"{round(index * DEG_PER_TURN / beams)}deg"


class ReactiveObstacleAvoider:
    """Repulsive potential field + tangential circulation around obstacles."""

    def __init__(self, safety_distance: Optional[float] = None, detection_distance: Optional[float] = None,
                 max_repulsive_speed: Optional[float] = None, ignore_behind: Optional[bool] = None,
                 **overrides: Any) -> None:
        given = {k: v for k, v in dict(safety_distance=safety_distance, detection_distance=detection_distance,
                                        max_repulsive_speed=max_repulsive_speed,
                                        ignore_behind=ignore_behind).items() if v is not None}
        given.update(overrides)
        p: Dict[str, Any] = section("avoidance", given)
        self.params = p
        self.safety_distance = float(p["safety_distance"])
        self.detection_distance = float(p["detection_distance"])
        self.max_repulsive_speed = float(p["max_repulsive_speed"])
        self.ignore_behind = bool(p["ignore_behind"])
        self.clear_reset_steps = int(p["clear_reset_steps"])
        # Side memory: once an obstacle is being passed on one side, keep that side until the
        # path has been clear for clear_reset_steps (removes the APF local minimum / dithering).
        self._circ_sign: Optional[float] = None
        self._clear_count = 0
        self.beams = int(p["lidar_beams"])
        step = 2.0 * math.pi / self.beams
        self.ray_dirs = [(math.cos(i * step), math.sin(i * step), 0.0) for i in range(self.beams)]

    def compute_avoidance_velocity(self, lidar: LidarScan, current_vel: Tuple[float, float, float],
                                   desired_vel: Tuple[float, float, float], heading: float = 0.0,
                                   ground_check: bool = True) -> Tuple[Tuple[float, float, float], bool, str]:
        """Collision-free velocity from the nominal one.  Returns (velocity, avoiding, status).
        ground_check: push up when descending close to something below (off while landing)."""
        p = self.params
        rep_x = rep_y = rep_z = 0.0
        avoiding = False
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        speed_xy = math.hypot(desired_vel[0], desired_vel[1])
        ranges = list(lidar.ranges)
        clamp = self.safety_distance * p["safety_clamp_factor"]

        for i, dist in enumerate(lidar.ranges):
            if not (0.0 < dist < self.detection_distance):
                continue
            bdx, bdy, _ = self.ray_dirs[i]
            wdx = bdx * cos_h - bdy * sin_h
            wdy = bdx * sin_h + bdy * cos_h
            if (self.ignore_behind and dist > self.safety_distance and speed_xy > p["min_speed_for_direction"]
                    and (wdx * desired_vel[0] + wdy * desired_vel[1]) / speed_xy < p["behind_cosine"]):
                ranges[i] = lidar.max_range          # beside / behind the motion: ignore
                continue
            avoiding = True
            d = max(dist, clamp)
            strength = (self.detection_distance - d) / (self.detection_distance - clamp)
            strength = min(strength, p["max_strength"]) ** p["strength_exponent"]
            rep_x -= wdx * strength                  # push away from the ray direction
            rep_y -= wdy * strength

        if ground_check and desired_vel[2] < 0.0 and 0.0 < lidar.dist_down < p["ground_clearance"]:
            rep_z += p["ground_push"]
            avoiding = True

        if not avoiding:
            self._clear_count += 1
            if self._clear_count >= self.clear_reset_steps:
                self._circ_sign = None
            return desired_vel, False, "PATH CLEAR"
        self._clear_count = 0
        scan = LidarScan(ranges=ranges, dist_up=lidar.dist_up, dist_down=lidar.dist_down, max_range=lidar.max_range)

        mag = math.hypot(rep_x, rep_y)
        if mag > p["min_repulsion"]:
            rep_x = rep_x / mag * self.max_repulsive_speed
            rep_y = rep_y / mag * self.max_repulsive_speed
            circ_x, circ_y = -rep_y, rep_x           # tangential (wall-following) direction
            if self._circ_sign is None:
                self._circ_sign = -1.0 if circ_x * desired_vel[0] + circ_y * desired_vel[1] < 0.0 else 1.0
            rep_x += circ_x * self._circ_sign * p["circulation_weight"]
            rep_y += circ_y * self._circ_sign * p["circulation_weight"]
            tot = math.hypot(rep_x, rep_y)
            if tot > self.max_repulsive_speed:
                rep_x = rep_x / tot * self.max_repulsive_speed
                rep_y = rep_y / tot * self.max_repulsive_speed

        # blend: the closer the obstacle, the more avoidance overrides the nominal path
        min_dist = scan.min_horizontal_dist
        w_avoid = min(1.0, max(0.0, (self.detection_distance - min_dist) / (self.detection_distance - self.safety_distance)))
        w_goal = max(p["min_goal_weight"], 1.0 - w_avoid)
        vx = desired_vel[0] * w_goal + rep_x * w_avoid
        vy = desired_vel[1] * w_goal + rep_y * w_avoid
        vz = desired_vel[2] + rep_z * p["vertical_push_gain"]

        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > p["max_safe_speed"]:
            k = p["max_safe_speed"] / speed
            vx, vy, vz = vx * k, vy * k, vz * k

        status = f"AVOIDING [{sector_name(scan.closest_sector, self.beams)} {min_dist:.2f}m]"
        return (vx, vy, vz), True, status
