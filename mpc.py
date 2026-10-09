"""
High-Level Lateral Steering Controller: Extended Kinematic Bicycle MPC.
Solves a constrained non-linear program over prediction horizon N using SciPy,
optimizing steering angle and longitudinal acceleration (mapped to throttle).
"""

import math
import numpy as np
from scipy.optimize import minimize


def wrap_angle(a):
    """Wraps an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class KinematicBicycleMPC:
    """Nonlinear Model Predictive Control for an Extended Kinematic Bicycle Model.

    Optimizes future control sequences u = [delta_k, a_k] where steering angle delta_k
    and longitudinal acceleration a_k (mapped to throttle effort) are the control inputs,
    forward-simulating a 4-state extended kinematic bicycle model x = [x, y, theta, v]^T.
    """

    def __init__(self, wheelbase=1.25, dt=0.1, horizon=10,
                 max_steer_rad=math.radians(35.0), k_a=4.0,
                 max_accel=None, max_brake=None, c_drag=0.005, c_roll=0.05,
                 delay_steps=1):
        self.L = wheelbase
        self.dt = dt
        self.N = horizon
        self.max_steer_rad = max_steer_rad
        self.k_a = float(max_accel if max_accel is not None else k_a)
        self.max_brake = float(max_brake if max_brake is not None else self.k_a)
        # Same resistance terms as the simulator, so the prediction model matches the plant
        self.c_drag = c_drag
        self.c_roll = c_roll
        # Control delay (steps): sensor age + 10 Hz timers, so a command acts one step later
        self.delay_steps = int(delay_steps)
        self.last_cmd = (0.0, 0.0)  # (delta, a) returned by the previous solve

        # Weights: heavily penalize lateral CTE, heading error, and steering rate
        self.w_lat = 60.0
        self.w_long = 1.0
        self.w_yaw = 2.0
        self.w_v = 1.0
        self.w_steer = 0.2
        self.w_dsteer = 6.0
        self.w_accel = 0.1

        self.last_u = np.zeros(2 * self.N)  # warm-start [delta_0, a_0, delta_1, a_1, ...]

    def propagate(self, state, delta, a):
        """One forward-Euler step of the extended kinematic bicycle model."""
        x, y, th, v = state
        dt = self.dt
        x_n = x + v * math.cos(th) * dt
        y_n = y + v * math.sin(th) * dt
        th_n = th + v / self.L * math.tan(delta) * dt
        v_n = max(v + (a - self.c_drag * v * v - self.c_roll * v) * dt, 0.0)
        return [x_n, y_n, th_n, v_n]

    def solve(self, x0, ref_trajectory, current_steer=0.0):
        """Solves MPC optimization problem over horizon N.

        x0: [x, y, yaw, v]
        ref_trajectory: list of length N containing [x_ref, y_ref, yaw_ref, v_ref]
        current_steer: actual current steering angle in radians
        Returns: (steer_rad, throttle_cmd in [-1.0, 1.0])
        """
        # 0. Delay compensation: the command computed now only takes effect `delay_steps` later.
        #    So predict where the car will be by then (using the commands already on their way)
        #    and drop the same number of reference points.
        state0 = [float(c) for c in x0]
        prev_cmd_delta = float(current_steer)
        if self.delay_steps > 0 and len(ref_trajectory) > self.delay_steps + 2:
            for _ in range(self.delay_steps):
                state0 = self.propagate(state0, self.last_cmd[0], self.last_cmd[1])
            prev_cmd_delta = self.last_cmd[0]
            ref_trajectory = ref_trajectory[self.delay_steps:]

        # 1. Horizon & bounds
        N = min(self.N, len(ref_trajectory))
        if N < 2:
            return 0.0, 0.0

        bounds = [(-self.max_steer_rad, self.max_steer_rad),
                  (-self.max_brake, self.k_a)] * N

        dt = self.dt
        L = self.L
        ref = [list(map(float, r)) for r in ref_trajectory[:N]]
        px0, py0, th0, v0 = state0

        # 2. Objective: roll the model forward and add up the Frenet-frame costs
        def objective(u):
            x, y, th, v = px0, py0, th0, v0
            prev_delta = prev_cmd_delta
            cost = 0.0
            for k in range(N):
                delta = u[2 * k]
                a = u[2 * k + 1]

                # Extended kinematic bicycle, forward Euler (v is a state)
                x += v * math.cos(th) * dt
                y += v * math.sin(th) * dt
                th += v / L * math.tan(delta) * dt
                v += (a - self.c_drag * v * v - self.c_roll * v) * dt
                if v < 0.0:
                    v = 0.0

                # Error in the path-aligned (Frenet) frame of reference point k
                xr, yr, thr, vr = ref[k]
                dx = x - xr
                dy = y - yr
                e_long = math.cos(thr) * dx + math.sin(thr) * dy
                e_lat = -math.sin(thr) * dx + math.cos(thr) * dy
                e_yaw = wrap_angle(th - thr)
                e_v = v - vr

                cost += (self.w_lat * e_lat ** 2 + self.w_long * e_long ** 2
                         + self.w_yaw * e_yaw ** 2 + self.w_v * e_v ** 2
                         + self.w_steer * delta ** 2
                         + self.w_dsteer * (delta - prev_delta) ** 2
                         + self.w_accel * a ** 2)
                prev_delta = delta
            return cost

        # 3. Warm start: previous solution shifted forward by one step
        shifted = np.concatenate([self.last_u[2:], self.last_u[-2:]])
        u_init = shifted[:2 * N].copy()
        for i, (lo, hi) in enumerate(bounds):
            u_init[i] = min(max(u_init[i], lo), hi)

        # 4. Optimize
        res = minimize(objective, u_init, bounds=bounds, method='SLSQP',
                       options={'maxiter': 25, 'ftol': 1e-3})
        u_opt = res.x if np.all(np.isfinite(res.x)) else u_init

        # Save for the next warm start (pad back to the full horizon length)
        padded = np.zeros(2 * self.N)
        padded[:2 * N] = u_opt
        if N < self.N:
            padded[2 * N:] = np.tile(u_opt[-2:], self.N - N)
        self.last_u = padded

        delta_cmd = float(u_opt[0])
        self.last_cmd = (delta_cmd, float(u_opt[1]))
        throttle_cmd = float(np.clip(u_opt[1] / self.k_a, -1.0, 1.0))
        return delta_cmd, throttle_cmd
