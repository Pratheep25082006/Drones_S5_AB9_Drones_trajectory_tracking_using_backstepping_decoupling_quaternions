"""
Global path planner for set-point flights (all simulators).

The reactive LiDAR avoider only sees a few metres; to reach a target behind or next
to a city block the route is planned first on the city map (world_map.WorldMap):

  1. straight line free (with clearance)            -> fly straight
  2. target inside a building (+clearance)          -> moved to the nearest free point
                                                       (or above the roof: planner.inside_goal)
  3. otherwise: climb / descend vertically at the lower end to the cruise height
     max(start z, goal z), A* (8-connected) on a grid of the obstacles at that height,
     line-of-sight shortening -> waypoints
  4. no route at that height                        -> over the top of the blocking buildings

The corners of the route are then rounded (quadratic Bezier blends, as large as the
obstacles allow - checked on the map) and the curve is sampled densely, so the drone flies
one smooth curve (e.g. climbing while it turns) instead of "straight up, then across".
The carrot of ReferenceManager follows the samples; the LiDAR avoider stays active.
Parameters: config.yaml -> planner.
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from typing import List, Optional, Tuple

import numpy as np

from .config import cfg, pick
from .world_map import WorldMap

Vec3 = Tuple[float, float, float]


class GridPlanner:
    def __init__(self, world: WorldMap, resolution: Optional[float] = None, clearance: Optional[float] = None,
                 vertical_clearance: Optional[float] = None):
        g = "planner."
        self.world = world
        self.res = pick(resolution, g + "resolution")
        self.clear = pick(clearance, g + "clearance")
        self.vclear = pick(vertical_clearance, g + "vertical_clearance")
        self.ignore_below = cfg(g + "ignore_below")
        self.margin = cfg(g + "margin")
        self.column_radius = cfg(g + "column_radius")
        self.inside_goal = cfg(g + "inside_goal")
        self.los_step = cfg(g + "los_step")
        self.corner_radius = cfg(g + "corner_radius")
        self.radius_scales = cfg(g + "radius_scales")
        self.sample_step = cfg(g + "sample_step")
        self.curve_clearance = cfg(g + "curve_clearance")
        self.extent = cfg("world.extent")
        self.note = ""

    # ------------------------------------------------------------------ grid
    def _grid(self, z_lo: float, z_hi: float, pts):
        lo, hi = self.world.boxes(self.ignore_below)
        band = (hi[:, 2] + self.vclear > z_lo) & (lo[:, 2] - self.vclear < z_hi)
        lo, hi = lo[band], hi[band]
        xs = [p[0] for p in pts] + [-self.extent, self.extent]
        ys = [p[1] for p in pts] + [-self.extent, self.extent]
        x0, y0 = min(xs) - self.margin, min(ys) - self.margin
        nx = int(math.ceil((max(xs) + self.margin - x0) / self.res)) + 1
        ny = int(math.ceil((max(ys) + self.margin - y0) / self.res)) + 1
        occ = np.zeros((nx, ny), dtype=bool)
        for a, b in zip(lo, hi):
            i0 = max(0, int(math.floor((a[0] - self.clear - x0) / self.res)))
            i1 = min(nx - 1, int(math.ceil((b[0] + self.clear - x0) / self.res)))
            j0 = max(0, int(math.floor((a[1] - self.clear - y0) / self.res)))
            j1 = min(ny - 1, int(math.ceil((b[1] + self.clear - y0) / self.res)))
            occ[i0:i1 + 1, j0:j1 + 1] = True
        return occ, x0, y0, hi

    def _cell(self, p, x0, y0, occ):
        i = min(occ.shape[0] - 1, max(0, int(round((p[0] - x0) / self.res))))
        j = min(occ.shape[1] - 1, max(0, int(round((p[1] - y0) / self.res))))
        return i, j

    def _center(self, c, x0, y0):
        return x0 + c[0] * self.res, y0 + c[1] * self.res

    @staticmethod
    def _reachable(occ, seed):
        """Free cells connected to the seed cell (4-neighbourhood flood fill)."""
        nx, ny = occ.shape
        seen = np.zeros_like(occ)
        if occ[seed]:
            return seen
        seen[seed] = True
        q = deque([seed])
        while q:
            i, j = q.popleft()
            for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if 0 <= n[0] < nx and 0 <= n[1] < ny and not seen[n] and not occ[n]:
                    seen[n] = True
                    q.append(n)
        return seen

    def _nearest_free(self, occ, c, allowed=None):
        """Free (and allowed) cell with the smallest Euclidean distance to c (None if there is none)."""
        free = np.argwhere(~occ if allowed is None else (~occ & allowed))
        if len(free) == 0:
            return None
        k = int(np.argmin(np.sum((free - np.asarray(c)) ** 2, axis=1)))
        return int(free[k, 0]), int(free[k, 1])

    def _astar(self, occ, s, g):
        nx, ny = occ.shape
        diag = math.sqrt(2.0)
        moves = [(di, dj, math.hypot(di, dj)) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj]

        def h(c):
            dx, dy = abs(c[0] - g[0]), abs(c[1] - g[1])
            return (dx + dy) + (diag - 2.0) * min(dx, dy)

        openq = [(h(s), 0.0, s)]
        cost = {s: 0.0}
        parent = {s: None}
        while openq:
            _, c, cur = heapq.heappop(openq)
            if cur == g:
                path = []
                while cur is not None:
                    path.append(cur)
                    cur = parent[cur]
                return path[::-1]
            if c > cost[cur]:
                continue
            for di, dj, w in moves:
                n = (cur[0] + di, cur[1] + dj)
                if not (0 <= n[0] < nx and 0 <= n[1] < ny) or (occ[n] and n != g):
                    continue
                if di and dj and (occ[cur[0] + di, cur[1]] or occ[cur[0], cur[1] + dj]):
                    continue                                   # no corner cutting
                nc = c + w
                if nc < cost.get(n, math.inf):
                    cost[n] = nc
                    parent[n] = cur
                    heapq.heappush(openq, (nc + h(n), nc, n))
        return None

    def _los(self, occ, a, b):
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) * self.res / self.los_step)))
        for k in range(n + 1):
            u = k / n
            c = (int(round(a[0] + (b[0] - a[0]) * u)), int(round(a[1] + (b[1] - a[1]) * u)))
            if occ[c]:
                return False
        return True

    def _shorten(self, occ, path):
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._los(occ, path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    def _column_free(self, x, y, z_lo, z_hi) -> bool:
        r = self.column_radius
        return self.world.segment_free((x, y, z_lo), (x, y, z_hi), r, self.vclear, self.ignore_below)

    # ------------------------------------------------------------------ public
    def free_goal(self, goal: Vec3, start: Optional[Vec3] = None) -> Vec3:
        """Target moved out of buildings (inflated by the clearance): nearest free point that can be
        reached from start at the target height (any free point when start is None / enclosed)."""
        inside = self.world.containing(goal, self.clear, self.vclear, self.ignore_below)
        if len(inside) == 0:
            return tuple(goal)
        if self.inside_goal == "above":
            z = goal[2]
            _, hi = self.world.boxes(self.ignore_below)
            while len(inside):
                z = float(np.max(hi[inside, 2])) + self.vclear
                inside = self.world.containing((goal[0], goal[1], z), self.clear, self.vclear, self.ignore_below)
            return (goal[0], goal[1], z)
        occ, x0, y0, _ = self._grid(goal[2], goal[2], [goal] + ([start] if start is not None else []))
        allowed = None
        if start is not None:
            seed = self._nearest_free(occ, self._cell(start, x0, y0, occ))
            if seed is not None:
                allowed = self._reachable(occ, seed)
        c = self._nearest_free(occ, self._cell(goal, x0, y0, occ), allowed)
        if c is None:
            return tuple(goal)
        x, y = self._center(c, x0, y0)
        return (x, y, goal[2])

    def plan(self, start: Vec3, goal: Vec3) -> List[Vec3]:
        """Smooth route: dense points from start (excluded) to the (possibly moved) goal (last element)."""
        wps = self._route(start, goal)
        if len(wps) < 2:
            return wps                                   # straight line: the reference model is smooth already
        return self._smooth([tuple(start)] + [tuple(w) for w in wps])

    # ------------------------------------------------------------------ smoothing
    def _curve_free(self, pts) -> bool:
        return all(self.world.segment_free(pts[i], pts[i + 1], self.curve_clearance, self.curve_clearance,
                                           self.ignore_below) for i in range(len(pts) - 1))

    def _sample_line(self, a, b, out) -> None:
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / self.sample_step)))
        out.extend(tuple(float(v) for v in a + (b - a) * k / n) for k in range(1, n + 1))

    def _rounded(self, pts, scale: float):
        """Polyline with each interior corner replaced by a quadratic Bezier of radius
        scale * min(corner_radius, half of the adjacent legs)."""
        P = [np.asarray(p, dtype=float) for p in pts]
        out = []
        cur = P[0]
        for i in range(1, len(P) - 1):
            a, b = P[i] - P[i - 1], P[i + 1] - P[i]
            la, lb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
            # legs shared by two corners give each half; the first / last leg can be used fully
            la_max = la if i == 1 else la / 2.0
            lb_max = lb if i == len(P) - 2 else lb / 2.0
            r = scale * min(self.corner_radius, la_max, lb_max)
            if r <= 0.0 or la == 0.0 or lb == 0.0:
                self._sample_line(cur, P[i], out)
                cur = P[i]
                continue
            A, B = P[i] - a / la * r, P[i] + b / lb * r
            self._sample_line(cur, A, out)
            n = max(2, int(math.ceil(2.0 * r / self.sample_step)))
            for k in range(1, n + 1):
                u = k / n
                out.append(tuple(float(v) for v in (1 - u) ** 2 * A + 2 * (1 - u) * u * P[i] + u ** 2 * B))
            cur = B
        self._sample_line(cur, P[-1], out)
        return out

    def _smooth(self, pts):
        for scale in self.radius_scales:                 # largest blend that stays clear of obstacles
            curve = self._rounded(pts, scale)
            if self._curve_free([pts[0]] + curve):
                return curve
        return self._rounded(pts, 0.0)

    def _route(self, start: Vec3, goal: Vec3) -> List[Vec3]:
        """Waypoints from start (excluded) to the (possibly moved) goal (last element)."""
        self.note = ""
        g = self.free_goal(goal, start)
        if math.dist(g, goal) > self.res / 2:
            self.note = f"target inside an obstacle -> moved to ({g[0]:.1f}, {g[1]:.1f}, {g[2]:.1f})"
        if self.world.segment_free(start, g, self.clear, self.vclear, self.ignore_below):
            return [g]
        zc = max(start[2], g[2])
        occ, x0, y0, _ = self._grid(zc, zc, [start, g])
        s_cell, g_cell = self._cell(start, x0, y0, occ), self._cell(g, x0, y0, occ)
        escape = []
        if occ[s_cell]:                     # close to a wall: first move sideways out of the clearance zone
            g_seed = g_cell if not occ[g_cell] else self._nearest_free(occ, g_cell)
            toward_goal = self._reachable(occ, g_seed) if g_seed is not None else None
            free = self._nearest_free(occ, s_cell, toward_goal)
            if free is not None:
                s_cell = free
                x, y = self._center(free, x0, y0)
                escape = [(x, y, start[2])]
                start = escape[0]
        climb_ok = self._column_free(start[0], start[1], start[2], zc)
        desc_ok = self._column_free(g[0], g[1], g[2], zc)
        path = self._astar(occ, s_cell, g_cell) if climb_ok and desc_ok else None
        if path is not None:
            pts = [self._center(c, x0, y0) for c in self._shorten(occ, path)[1:-1]]
            wps = list(escape)
            if zc - start[2] > self.res:
                wps.append((start[0], start[1], zc))
            wps += [(x, y, zc) for x, y in pts]
            if zc - g[2] > self.res:
                wps.append((g[0], g[1], zc))
            wps.append(g)
            self.note = (self.note + "; " if self.note else "") + f"route around obstacles, {len(wps)} waypoints"
            return wps
        # over the top of everything between start and goal
        lo, hi = self.world.inflated(self.clear, self.vclear, self.ignore_below)
        L = math.hypot(g[0] - start[0], g[1] - start[1])
        n = max(1, int(math.ceil(L / self.los_step)))
        z_top = zc
        for k in range(n + 1):
            u = k / n
            p = np.array([start[0] + (g[0] - start[0]) * u, start[1] + (g[1] - start[1]) * u])
            under = np.all((lo[:, :2] <= p) & (p <= hi[:, :2]), axis=1)
            if np.any(under):
                z_top = max(z_top, float(np.max(hi[under, 2])))
        self.note = (self.note + "; " if self.note else "") + f"no route at {zc:.1f} m -> over the top at {z_top:.1f} m"
        return escape + [(start[0], start[1], z_top), (g[0], g[1], z_top), g]
