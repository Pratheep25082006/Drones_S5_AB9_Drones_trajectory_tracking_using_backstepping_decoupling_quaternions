"""
Obstacle map of the generated city (config world.*) for simulators without their own
collision engine (Gazebo / web bridge flight core) and for the path planner.

Every solid primitive of the city (world_builder.CityBuilder, collide=True) becomes an
axis-aligned box [lo, hi].  Ray casts are vectorised over all boxes (slab method):
  * raycast(origin, direction, max_range)       distance to the first hit (ground plane included)
  * lidar(pos, heading, beams, max_range)        LidarScan like the MuJoCo / PyBullet LiDARs
  * segment_free(a, b, clearance, vclear, ...)   straight path check for the planner
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

from .config import cfg
from .obstacle_avoidance import LidarScan

Vec3 = Tuple[float, float, float]


class WorldMap:
    def __init__(self, lo: np.ndarray, hi: np.ndarray, ground_z: float = 0.0, names: Optional[Sequence[str]] = None):
        self.lo = np.asarray(lo, dtype=float).reshape(-1, 3)
        self.hi = np.asarray(hi, dtype=float).reshape(-1, 3)
        self.ground_z = ground_z
        self.names = list(names) if names is not None else [""] * len(self.lo)
        self.eps = cfg("numerics.degenerate_tolerance")

    @classmethod
    def from_city(cls) -> "WorldMap":
        from .world_builder import CityBuilder
        lo, hi, names = [], [], []
        for p in CityBuilder().build():
            if not p.collide:
                continue
            if p.kind == "box":
                h = p.size
            elif p.kind == "cylinder":
                h = (p.size[0], p.size[0], p.size[1])
            else:
                h = (p.size[0],) * 3
            c, yaw = np.asarray(p.pos), p.yaw
            if yaw:   # rotated box -> enclosing axis-aligned box
                cy, sy = abs(math.cos(yaw)), abs(math.sin(yaw))
                h = (cy * h[0] + sy * h[1], sy * h[0] + cy * h[1], h[2])
            lo.append(c - np.asarray(h))
            hi.append(c + np.asarray(h))
            names.append(p.name)
        return cls(np.array(lo), np.array(hi), 0.0, names)

    # ------------------------------------------------------------------ queries
    def _slab(self, o, d, lo, hi):
        d = np.where(np.abs(d) < self.eps, self.eps, d)
        t1 = (lo - o) / d
        t2 = (hi - o) / d
        tmin = np.max(np.minimum(t1, t2), axis=1)
        tmax = np.min(np.maximum(t1, t2), axis=1)
        return tmin, tmax

    def raycast(self, origin, direction, max_range: float) -> float:
        o = np.asarray(origin, dtype=float)
        d = np.asarray(direction, dtype=float)
        d = d / np.linalg.norm(d)
        best = max_range
        if d[2] < -self.eps:
            best = min(best, (self.ground_z - o[2]) / d[2])
        near = np.all((self.lo - max_range <= o) & (o <= self.hi + max_range), axis=1)
        if np.any(near):
            tmin, tmax = self._slab(o, d, self.lo[near], self.hi[near])
            hit = (tmax >= tmin) & (tmin >= 0.0)          # boxes the origin is inside are ignored (like mj_ray)
            if np.any(hit):
                best = min(best, float(np.min(tmin[hit])))
        return best

    def lidar(self, pos, heading: float, beams: int, max_range: float) -> LidarScan:
        step = 2.0 * math.pi / beams
        ranges = [self.raycast(pos, (math.cos(heading + i * step), math.sin(heading + i * step), 0.0), max_range)
                  for i in range(beams)]
        return LidarScan(ranges=ranges, dist_up=self.raycast(pos, (0.0, 0.0, 1.0), max_range),
                         dist_down=self.raycast(pos, (0.0, 0.0, -1.0), max_range), max_range=max_range)

    def boxes(self, ignore_below: float = -math.inf):
        keep = self.hi[:, 2] > ignore_below
        return self.lo[keep], self.hi[keep]

    def inflated(self, clearance: float, vclear: float, ignore_below: float):
        lo, hi = self.boxes(ignore_below)
        pad = np.array([clearance, clearance, vclear])
        return lo - pad, hi + pad

    def containing(self, p, clearance: float = 0.0, vclear: float = 0.0, ignore_below: float = -math.inf):
        """Indices (into boxes(ignore_below)) of the boxes that contain p after inflation."""
        lo, hi = self.inflated(clearance, vclear, ignore_below)
        p = np.asarray(p, dtype=float)
        return np.nonzero(np.all((lo <= p) & (p <= hi), axis=1))[0]

    def segment_free(self, a, b, clearance: float, vclear: float, ignore_below: float) -> bool:
        a = np.asarray(a, dtype=float)
        d = np.asarray(b, dtype=float) - a
        if np.linalg.norm(d) < self.eps:
            return len(self.containing(a, clearance, vclear, ignore_below)) == 0
        lo, hi = self.inflated(clearance, vclear, ignore_below)
        tmin, tmax = self._slab(a, d, lo, hi)
        return not np.any((tmax >= tmin) & (tmax >= 0.0) & (tmin <= 1.0))

    def min_distance(self, p) -> float:
        """Distance from p to the nearest solid box (0 inside)."""
        p = np.asarray(p, dtype=float)
        gap = np.maximum(np.maximum(self.lo - p, p - self.hi), 0.0)
        return float(np.min(np.linalg.norm(gap, axis=1))) if len(gap) else math.inf
