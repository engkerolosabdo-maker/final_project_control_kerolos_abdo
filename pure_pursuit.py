"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math
import numpy as np


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        return float(np.clip(self.kv * v + self.l_min, self.l_min, self.l_max))

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance.

        Returns (index, point) where point is the path entry (x, y, yaw).
        """
        n = len(path_points)

        # 1. Nearest waypoint to the rear axle
        nearest_idx = 0
        min_d_sq = float('inf')
        for i in range(n):
            dx = path_points[i][0] - x
            dy = path_points[i][1] - y
            d_sq = dx * dx + dy * dy
            if d_sq < min_d_sq:
                min_d_sq = d_sq
                nearest_idx = i

        # 2. Walk forward (the track is a closed loop, so wrap around) until the
        #    waypoint is at least `lookahead` meters away from the car.
        idx = nearest_idx
        for _ in range(n):
            dx = path_points[idx][0] - x
            dy = path_points[idx][1] - y
            if math.hypot(dx, dy) >= lookahead:
                break
            idx = (idx + 1) % n

        return idx, path_points[idx]

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        # 3. Direction to the target in the vehicle frame: alpha > 0 means target is on the LEFT
        dx = target_pt[0] - x
        dy = target_pt[1] - y
        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # 4. Arc law: delta = atan(2 L sin(alpha) / Ld)
        ld = max(float(lookahead), 1e-3)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), ld)

        return float(np.clip(delta, -self.max_steer_rad, self.max_steer_rad))
