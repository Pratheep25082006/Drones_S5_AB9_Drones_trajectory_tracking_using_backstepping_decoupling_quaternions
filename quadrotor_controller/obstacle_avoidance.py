"""
Omnidirectional LiDAR Sensor & Reactive Obstacle Avoidance for Urban City Environments.
Implements:
  1. Omnidirectional LiDAR data structure (8 horizontal rays + 2 vertical rays).
  2. Artificial Potential Field (APF) + Tangential Circulation for obstacle circumnavigation.
  3. Seamless path replanning to steer drones around skyscrapers and building blocks.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np


@dataclass
class LidarScan:
    """
    3D LiDAR sensor readings from drone center.
    Angles: 0 (Forward), 45, 90 (Left), 135, 180 (Back), 225, 270 (Right), 315.
    Plus up and down range.
    Distances in meters. -1 or large value indicates no hit / out of range.
    """
    ranges: List[float]       # 8 horizontal ray distances in order [0, 45, 90, 135, 180, 225, 270, 315]
    dist_up: float            # upward clearance
    dist_down: float          # downward clearance
    max_range: float = 8.0

    @property
    def min_horizontal_dist(self) -> float:
        valid = [r for r in self.ranges if r > 0.0]
        return min(valid) if valid else self.max_range

    @property
    def closest_sector(self) -> int:
        valid_ranges = [r if r > 0.0 else self.max_range for r in self.ranges]
        return int(np.argmin(valid_ranges))


class ReactiveObstacleAvoider:
    """
    Reactive Obstacle Avoidance Engine.
    Combines repulsive potential fields with tangential circulation vectors
    to smoothly steer the drone around buildings without getting trapped in local minima.
    """

    def __init__(
        self,
        safety_distance: float = 1.0,     # Critical inner safety margin (meters)
        detection_distance: float = 2.2,  # Outer detection distance where avoidance starts (meters)
        max_repulsive_speed: float = 2.2, # Max avoidance speed in m/s
    ) -> None:
        self.safety_distance = float(safety_distance)
        self.detection_distance = float(detection_distance)
        self.max_repulsive_speed = float(max_repulsive_speed)

        # 8 Horizontal Ray Unit Vectors (in drone body / heading frame)
        self.ray_angles = [i * (math.pi / 4.0) for i in range(8)]
        self.ray_dirs = [
            (math.cos(ang), math.sin(ang), 0.0) for ang in self.ray_angles
        ]

    def compute_avoidance_velocity(
        self,
        lidar: LidarScan,
        current_vel: Tuple[float, float, float],
        desired_vel: Tuple[float, float, float],
        heading: float = 0.0,
    ) -> Tuple[Tuple[float, float, float], bool, str]:
        """
        Given the current LiDAR scan and desired nominal velocity,
        computes the modified collision-free velocity vector.
        
        Returns:
            (safe_velocity, is_avoiding_flag, status_description)
        """
        repulsive_x = 0.0
        repulsive_y = 0.0
        repulsive_z = 0.0
        is_avoiding = False
        active_threats = 0

        cos_h = math.cos(heading)
        sin_h = math.sin(heading)

        # Evaluate each horizontal ray
        for i, dist in enumerate(lidar.ranges):
            if 0.0 < dist < self.detection_distance:
                is_avoiding = True
                active_threats += 1

                # Normalize threat severity: 1.0 at safety distance, 0.0 at detection boundary
                clamped_dist = max(dist, self.safety_distance * 0.5)
                strength = (self.detection_distance - clamped_dist) / (self.detection_distance - self.safety_distance * 0.5)
                strength = min(strength, 2.5) ** 1.5

                # Unit direction of ray in world frame
                body_dx, body_dy, _ = self.ray_dirs[i]
                world_dx = body_dx * cos_h - body_dy * sin_h
                world_dy = body_dx * sin_h + body_dy * cos_h

                # Repulsive force points OPPOSITE to ray direction
                repulsive_x -= world_dx * strength
                repulsive_y -= world_dy * strength

        # Check vertical clearance
        if 0.0 < lidar.dist_down < 0.35:
            # Too close to ground/low roof: push up
            repulsive_z += 1.0
            is_avoiding = True

        if not is_avoiding:
            return desired_vel, False, "PATH CLEAR"

        # Normalize and scale repulsive vector
        rep_mag = math.sqrt(repulsive_x ** 2 + repulsive_y ** 2)
        if rep_mag > 1e-4:
            repulsive_x = (repulsive_x / rep_mag) * self.max_repulsive_speed
            repulsive_y = (repulsive_y / rep_mag) * self.max_repulsive_speed

            # Add tangential circulation to smoothly glide around building walls
            # Circulation is orthogonal to repulsive direction: (-rep_y, rep_x)
            # Choose circulation direction aligned with desired goal velocity
            circ_x = -repulsive_y
            circ_y = repulsive_x
            dot_circ_goal = circ_x * desired_vel[0] + circ_y * desired_vel[1]
            if dot_circ_goal < 0.0:
                circ_x = -circ_x
                circ_y = -circ_y

            circ_weight = 0.8
            repulsive_x += circ_x * circ_weight
            repulsive_y += circ_y * circ_weight

            # Re-scale combined horizontal avoidance speed
            tot_mag = math.sqrt(repulsive_x ** 2 + repulsive_y ** 2)
            if tot_mag > self.max_repulsive_speed:
                repulsive_x = (repulsive_x / tot_mag) * self.max_repulsive_speed
                repulsive_y = (repulsive_y / tot_mag) * self.max_repulsive_speed

        # Blend with nominal desired velocity
        # The closer the obstacle, the more the avoidance overrides nominal path
        min_dist = lidar.min_horizontal_dist
        avoid_weight = min(1.0, max(0.0, (self.detection_distance - min_dist) / (self.detection_distance - self.safety_distance)))
        goal_weight = max(0.15, 1.0 - avoid_weight)

        safe_vx = desired_vel[0] * goal_weight + repulsive_x * avoid_weight
        safe_vy = desired_vel[1] * goal_weight + repulsive_y * avoid_weight
        safe_vz = desired_vel[2] + repulsive_z * 0.5

        # Speed limit
        tot_speed = math.sqrt(safe_vx ** 2 + safe_vy ** 2 + safe_vz ** 2)
        MAX_SAFE_SPEED = 3.5
        if tot_speed > MAX_SAFE_SPEED:
            safe_vx = (safe_vx / tot_speed) * MAX_SAFE_SPEED
            safe_vy = (safe_vy / tot_speed) * MAX_SAFE_SPEED
            safe_vz = (safe_vz / tot_speed) * MAX_SAFE_SPEED

        sector_names = ["FWD", "FWD-L", "LEFT", "REAR-L", "REAR", "REAR-R", "RIGHT", "FWD-R"]
        threat_sector = sector_names[lidar.closest_sector]
        status = f"AVOIDING [{threat_sector} {min_dist:.2f}m]"

        return (safe_vx, safe_vy, safe_vz), True, status
