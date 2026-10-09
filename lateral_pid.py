"""
High-Level Lateral Steering Controller: Reactive Lateral PID.
Steers based on instantaneous Cross-Track Error (CTE) and Heading Error.
"""

import math
import numpy as np


class LateralPIDController:
    """Lateral PID steering controller based on Cross-Track Error (CTE) and Heading Error.

    Commands front wheel steering based on instantaneous lateral offset (cross-track error)
    and orientation error relative to the nearest path waypoint.
    """

    def __init__(self, kp=0.8, ki=0.02, kd=0.15, k_yaw=0.5, dt=0.1,
                 max_steer_rad=math.radians(35.0), integral_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.k_yaw = k_yaw
        self.dt = dt
        self.max_steer_rad = max_steer_rad
        self.integral_limit = integral_limit

        self.integral_cte = 0.0
        self.prev_cte = 0.0
        self.first_call = True

    def compute_steering(self, cte, heading_err):
        """Computes front wheel steering angle delta in radians.

        Args:
            cte: Signed cross-track error in meters (positive = vehicle is left of path).
            heading_err: Heading error in radians (psi_vehicle - psi_path).

        Returns:
            delta_rad: Commanded front steering angle in radians [-max_steer_rad, max_steer_rad].
        """
        # Integral of the CTE with anti-windup clamping
        self.integral_cte += cte * self.dt
        self.integral_cte = float(np.clip(self.integral_cte,
                                          -self.integral_limit, self.integral_limit))

        # Derivative of the CTE (skipped on the first call to avoid a kick)
        if self.first_call:
            d_cte = 0.0
            self.first_call = False
        else:
            d_cte = (cte - self.prev_cte) / self.dt
        self.prev_cte = cte

        # Sign convention: car LEFT of the path (cte > 0) -> steer RIGHT (negative delta).
        # Car pointing LEFT of the path direction (heading_err > 0) -> steer RIGHT too.
        delta = -(self.kp * cte + self.ki * self.integral_cte + self.kd * d_cte)
        delta -= self.k_yaw * heading_err

        return float(np.clip(delta, -self.max_steer_rad, self.max_steer_rad))

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral_cte = 0.0
        self.prev_cte = 0.0
        self.first_call = True
