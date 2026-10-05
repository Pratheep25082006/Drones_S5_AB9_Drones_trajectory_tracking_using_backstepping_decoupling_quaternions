"""
Unit Tests for LiDAR Sensing & Reactive Obstacle Avoidance
"""

import math
import os
import sys
import numpy as np
import pytest

# Ensure root directory is on path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from quadrotor_controller import (
    BacksteppingController,
    ControllerReference,
    ControllerState,
    LidarScan,
    ReactiveObstacleAvoider,
)


def test_lidar_scan_properties():
    """Verify LidarScan computes minimum horizontal distance and closest threat sector."""
    # Rays: [FWD, FWD-L, LEFT, REAR-L, REAR, REAR-R, RIGHT, FWD-R]
    ranges = [5.0, 5.0, 1.2, 5.0, 5.0, 5.0, 5.0, 5.0]
    scan = LidarScan(ranges=ranges, dist_up=5.0, dist_down=1.5, max_range=8.0)

    assert scan.min_horizontal_dist == 1.2
    assert scan.closest_sector == 2  # Index 2 is LEFT


def test_obstacle_avoider_clear_path():
    """Verify that when no obstacles are within detection range, velocity is unaltered."""
    avoider = ReactiveObstacleAvoider(safety_distance=0.8, detection_distance=2.0)
    scan = LidarScan(ranges=[6.0] * 8, dist_up=5.0, dist_down=2.0)

    desired_vel = (1.5, 0.0, 0.2)
    safe_vel, is_avoiding, status = avoider.compute_avoidance_velocity(
        lidar=scan,
        current_vel=desired_vel,
        desired_vel=desired_vel,
        heading=0.0,
    )

    assert not is_avoiding
    assert safe_vel == desired_vel
    assert "CLEAR" in status


def test_obstacle_avoider_head_on_repulsion_and_detour():
    """
    Verify that an obstacle directly in front (FWD ray = 1.0m) triggers:
    1. is_avoiding = True
    2. Negative X velocity (repulsion backward)
    3. Lateral Y velocity (tangential circulation detour around building)
    """
    avoider = ReactiveObstacleAvoider(safety_distance=0.8, detection_distance=2.0, max_repulsive_speed=2.0)
    # Obstacle at index 0 (FWD) at distance 0.9m
    ranges = [0.9, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
    scan = LidarScan(ranges=ranges, dist_up=5.0, dist_down=2.0)

    desired_vel = (1.0, 0.0, 0.0)
    safe_vel, is_avoiding, status = avoider.compute_avoidance_velocity(
        lidar=scan,
        current_vel=desired_vel,
        desired_vel=desired_vel,
        heading=0.0,
    )

    assert is_avoiding
    assert "AVOIDING" in status
    # Must steer away from front obstacle (safe_vx significantly reduced or reversed)
    assert safe_vel[0] < desired_vel[0]
    # Must add lateral detour component to circumnavigate building
    assert abs(safe_vel[1]) > 0.1


def test_obstacle_avoider_multidirectional_building_corner():
    """Verify drone steers away when passing close to a building on the left."""
    avoider = ReactiveObstacleAvoider(safety_distance=0.8, detection_distance=2.0)
    # Threat on LEFT (index 2: 0.9m)
    ranges = [4.0, 4.0, 0.9, 4.0, 4.0, 4.0, 4.0, 4.0]
    scan = LidarScan(ranges=ranges, dist_up=4.0, dist_down=1.5)

    desired_vel = (1.0, 0.5, 0.0)
    safe_vel, is_avoiding, status = avoider.compute_avoidance_velocity(
        lidar=scan,
        current_vel=desired_vel,
        desired_vel=desired_vel,
        heading=0.0,
    )

    assert is_avoiding
    # Must push away from left building (pushing right / -Y)
    assert safe_vel[1] < desired_vel[1]
    assert "LEFT" in status
