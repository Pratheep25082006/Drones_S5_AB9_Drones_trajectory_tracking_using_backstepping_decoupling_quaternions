"""City map, LiDAR ray cast and global path planner (shared by all simulators)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from quadrotor_controller import GridPlanner, PythonFlightSim, WorldMap  # noqa: E402
from quadrotor_controller.config import cfg  # noqa: E402

WORLD = WorldMap.from_city()


def route_is_free(start, wps, clearance=0.5, vclear=0.3):
    pts = [start] + list(wps)
    return all(WORLD.segment_free(pts[i], pts[i + 1], clearance, vclear, cfg("planner.ignore_below"))
               for i in range(len(pts) - 1))


def test_lidar_sees_the_clock_tower():
    # clock tower at (3.2, 3.2), half size 0.6 -> 45 deg beam from (0, 0, 1) hits it at ~3.68 m
    scan = WORLD.lidar((0.0, 0.0, 1.0), 0.0, 8, 8.0)
    assert abs(scan.ranges[1] - math.dist((0, 0), (2.6, 2.6))) < 0.05
    assert scan.dist_down < 1.0


def test_route_avoids_buildings_and_obstacles():
    planner = GridPlanner(WORLD)
    for start, goal in [((0, 0, 0.2), (12, 10, 9)), ((0, 0, 1), (25, -20, 4)), ((0, 0, 1), (-14, -30, 3))]:
        wps = planner.plan(start, goal)
        assert route_is_free(start, wps), (goal, wps)


def test_target_inside_a_building_is_moved_out():
    planner = GridPlanner(WORLD)
    goal = (3.2, 3.2, 1.0)                       # inside the clock tower
    wps = planner.plan((0, 0, 1), goal)
    g = wps[-1]
    assert len(WORLD.containing(g, cfg("planner.clearance") * 0.9, 0.0, cfg("planner.ignore_below"))) == 0
    assert math.dist(g, goal) < 3.0


def test_python_flight_sim_reaches_target_without_touching_obstacles():
    sim = PythonFlightSim(target_path=None, initial_command="12 10 9", seed=1)
    lo, hi = WORLD.boxes(cfg("planner.ignore_below"))
    closest = math.inf
    while sim.t < 40.0:
        sim.advance(0.1)
        x = sim.plant.x
        gap = np.maximum(np.maximum(lo - x, x - hi), 0.0)
        closest = min(closest, float(np.min(np.linalg.norm(gap, axis=1))))
    assert math.dist(sim.plant.x, (12, 10, 9)) < 0.05
    assert closest > cfg("avoidance.safety_distance") * 0.5
    assert sim.telemetry()["noise"] > 0.0                    # sensor noise is on
