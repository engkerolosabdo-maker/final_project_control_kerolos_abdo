"""
Low-Level Powertrain Cruise Controller (Longitudinal PID).
Regulates vehicle speed via normalized throttle/braking effort.
"""

import numpy as np


class PIDLongitudinalController:
    """Low-Level Powertrain Cruise Controller / Electronic Speed Control (ESC).

    Translates high-level velocity requests into normalized throttle/brake effort.
    Because physical vehicles experience friction and speed-squared aerodynamic drag,
    a closed-loop speed regulator is required to maintain target velocity.
    """

    def __init__(self, kp=1.0, ki=0.2, kd=0.05, dt=0.1,
                 max_throttle=1.0, max_brake=1.0, integral_limit=2.0, max_rate=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit
        # Optional slew-rate limit on the output (throttle units per second); None = off
        self.max_rate = max_rate

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_meas = 0.0
        self.prev_u = 0.0
        self.first_call = True

    def compute(self, target_vel, current_vel):
        """Computes normalized throttle/braking effort in [-1.0, 1.0]."""
        error = target_vel - current_vel

        # Derivative on the MEASUREMENT (not the error): when the profiler changes the target
        # speed the error jumps, and differentiating it would kick the throttle.
        # d(error)/dt = -d(v)/dt while the target is constant.  Skipped on the first call.
        if self.first_call:
            derivative = 0.0
            self.first_call = False
        else:
            derivative = -(current_vel - self.prev_meas) / self.dt
        self.prev_meas = current_vel
        self.prev_error = error

        # Integral with anti-windup (1): hard clamp on the accumulated error
        new_integral = float(np.clip(self.integral + error * self.dt,
                                     -self.integral_limit, self.integral_limit))

        u = self.kp * error + self.ki * new_integral + self.kd * derivative

        # Anti-windup (2): conditional integration. If the actuator is saturated and the
        # error keeps pushing in the same direction, do not keep accumulating.
        saturated_high = u > self.max_throttle and error > 0.0
        saturated_low = u < -self.max_brake and error < 0.0
        if not (saturated_high or saturated_low):
            self.integral = new_integral

        u = float(np.clip(u, -self.max_brake, self.max_throttle))

        # Smooth actuator output: limit how fast the command may change per step
        if self.max_rate is not None:
            step = self.max_rate * self.dt
            u = float(np.clip(u, self.prev_u - step, self.prev_u + step))
        self.prev_u = u
        return u

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_meas = 0.0
        self.prev_u = 0.0
        self.first_call = True
