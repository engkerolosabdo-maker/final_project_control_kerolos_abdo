"""
Teleoperation bridge node:
Subscribes to standard geometry_msgs/Twist on /cmd_vel (from teleop_twist_keyboard or joy)
and translates it to /throttle (Float32 in [-1.0, 1.0]) and /steer (Float32 in radians).

Supports two progression phases:
- Phase 1 (Milestone 3): Open-loop feedforward mapping with a safety watchdog timer.
- Phase 2 (Milestone 4): Closed-loop speed regulation using PIDLongitudinalController.
"""

import numpy as np  # noqa: F401
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from nav_msgs.msg import Odometry
from bicycle_control.longitudinal_pid import PIDLongitudinalController


class TeleopBridge(Node):
    def __init__(self):
        super().__init__('teleop_bridge')
        self.get_logger().info('Teleoperation Bridge Node Initialized')

        # Parameters
        self.declare_parameter('max_linear_vel', 5.0)     # m/s corresponding to full 1.0 throttle
        self.declare_parameter('max_angular_vel', 1.0)    # rad/s corresponding to full steering
        self.declare_parameter('max_steer_rad', 0.610865)  # radians (~35 degrees)
        self.declare_parameter('auto_zero_timeout', 0.5)  # seconds before zeroing commands
        self.declare_parameter('use_cruise_control', False)  # Enable in Milestone 4.2

        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.max_steer_rad = float(self.get_parameter('max_steer_rad').value)
        self.auto_zero_timeout = float(self.get_parameter('auto_zero_timeout').value)
        self.use_cruise_control = bool(self.get_parameter('use_cruise_control').value)

        # Publishers (10 Hz rate per assignment specification)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.current_throttle = 0.0
        self.current_steer = 0.0
        self.target_vel = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Phase 2 (Milestone 4.2): closed-loop cruise control.
        # Twist linear.x is treated as a TARGET SPEED (m/s) and the longitudinal PID turns
        # the speed error into throttle/brake, so drag and rolling resistance are rejected.
        self.current_speed = 0.0
        self.have_state = False
        self.pid = PIDLongitudinalController(kp=1.0, ki=0.2, kd=0.05, dt=0.1, max_rate=6.0)
        self.state_sub = self.create_subscription(Odometry, '/state', self.odom_callback, 10)
        if self.use_cruise_control:
            self.get_logger().info('Cruise control ENABLED: linear.x is a target speed (m/s)')

        # Publish loop at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_commands)

    def odom_callback(self, msg: Odometry):
        """Milestone 4.2: extracts the vehicle forward speed from /state odometry."""
        self.current_speed = float(msg.twist.twist.linear.x)
        self.have_state = True

    def cmd_callback(self, msg: Twist):
        """Translates Twist linear.x to throttle [-1, 1] and angular.z into steering (rad)."""
        # Milestone 3.1 — Teleoperation Command Mapping
        # This connects user inputs (keyboard/joystick) to the car's physical actuators.
        # Map the incoming Twist linear/angular commands to throttle and steering.
        # The car cannot reverse, so a negative speed target means "stop".
        self.target_vel = float(max(msg.linear.x, 0.0))

        throttle = msg.linear.x / self.max_linear_vel
        self.current_throttle = float(np.clip(throttle, -1.0, 1.0))

        steer = (msg.angular.z / self.max_angular_vel) * self.max_steer_rad
        self.current_steer = float(np.clip(steer, -self.max_steer_rad, self.max_steer_rad))

        self.last_cmd_time = self.get_clock().now()

    def publish_commands(self):
        """Periodically publishes throttle and steering commands at 10 Hz."""
        # Milestone 3.2 — Safety Watchdog & Command Publishing
        # This prevents the car from running away if the user's connection drops.
        # Publish the commands, or zero them out if the last command is too old.
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.auto_zero_timeout:
            self.current_throttle = 0.0
            self.current_steer = 0.0
            self.target_vel = 0.0

        if self.use_cruise_control:
            if self.have_state:
                # Closed loop: target 0 after the watchdog means the PID brakes the car to a stop
                self.current_throttle = self.pid.compute(self.target_vel, self.current_speed)
                if self.target_vel == 0.0 and self.current_speed < 0.05:
                    self.pid.reset()
                    self.current_throttle = 0.0
            else:
                self.current_throttle = 0.0  # no speed feedback yet: stay still

        throttle_msg = Float32()
        throttle_msg.data = float(self.current_throttle)
        steer_msg = Float32()
        steer_msg.data = float(self.current_steer)
        self.throttle_pub.publish(throttle_msg)
        self.steer_pub.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)
    bridge = TeleopBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
