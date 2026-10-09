# Bicycle Gym: Control Project

Name: Kerolos Abdo Anwar
Student ID: 2401039

## 1. Overview

In this project a simulated car drives around a race track in ROS 2 Humble. I built the
vehicle model, a keyboard bridge, a speed controller, a speed profiler, three steering
controllers (Lateral PID, Pure Pursuit and Model Predictive Control), and a lap analyzer
that measures how well each controller drives.

The main result: at the same constant speed of 4 m/s, MPC and Pure Pursuit follow the track
with an average error of about 1 to 2 centimeters (RMS), while the Lateral PID is about
ten times worse and visibly oscillates. When the speed profiler lets the car go faster (up to
7.5 m/s), Pure Pursuit stays accurate but the MPC becomes less accurate in my real runs.
Both results are reported below, together with the limits of my experiments.

## 2. System architecture (Milestone 1)

The repository has three ROS 2 Python packages:

- bicycle_sim: the vehicle model (bicycle_model.py), the simulator node, the URDF robot
  description and the RViz configuration.
- bicycle_control: the teleoperation bridge, the speed controller, the velocity profiler,
  the three steering controllers, and controller_node.py which runs the selected one.
- track_environment: the track loader, the path publisher and the lap analyzer.

Data flow, ten times per second:

    simulator --/state--> controller_node --/steer, /throttle--> simulator
    keyboard --/cmd_vel--> teleop_bridge --/steer, /throttle--> simulator
    simulator --/state--> lap_analyzer --> telemetry topics and RViz markers

Main topics (found with ros2 topic list, ros2 topic info and ros2 interface show):

| Topic | Type | Meaning and units |
|---|---|---|
| /throttle | std_msgs/Float32 | normalized throttle, -1 to 1 (negative brakes) |
| /steer | std_msgs/Float32 | front steering angle in radians, positive turns left |
| /state | nav_msgs/Odometry | rear axle position (m), yaw as a quaternion, speed in twist.linear.x (m/s) |
| /cmd_vel | geometry_msgs/Twist | keyboard command: linear.x in m/s, angular.z in rad/s |
| /telemetry/cte, /telemetry/speed | std_msgs/Float32 | cross-track error (m) and speed (m/s), published by the lap analyzer |
| /lap/visualization | visualization_msgs/MarkerArray | start gate, error line and scoreboard shown in RViz |

Vehicle parameters: wheelbase 1.25 m, track width 1.18 m, wheel radius 0.5 m, maximum steering
35 degrees, maximum speed 25 m/s, time step 0.1 s.

## 3. How each part works

### 3.1 Vehicle model (Milestone 2)

The state is [x, y, theta, v] at the rear axle. The speed v is a state, so the car
accelerates through the throttle and loses speed through drag and rolling friction:

    x'     = v cos(theta)
    y'     = v sin(theta)
    theta' = (v / L) tan(delta)
    v'     = k_a * throttle - (c_drag v^2 + c_roll v)

with L = 1.25 m, k_a = 4 m/s^2, c_drag = 0.005 and c_roll = 0.05. I integrate with forward Euler.
After every step the heading is wrapped to [-pi, pi) and the speed is clamped to [0, 25] so
braking can never make the car reverse. I checked the model in the running simulator:
with throttle 0.2 the speed settles at 8.55 m/s (theory: 8.60 m/s, where acceleration equals
resistance), and with steering 0.2 rad the measured yaw rate matches (v/L) tan(delta).

### 3.2 Teleoperation bridge (Milestone 3)

The bridge turns a Twist command into drive-by-wire signals with simple open-loop equations:
throttle = clip(linear.x / 5, -1, 1) and steering = clip(angular.z / 1.0, -1, 1) times the
maximum steering angle. It publishes at 10 Hz. A watchdog sets everything to zero if no command
arrived for 0.5 s, so the car does not keep driving if the keyboard stops.

### 3.3 Speed control and velocity profiler (Milestones 4 and 5.1)

The longitudinal PID uses the speed error e = v_target - v:

    throttle = Kp e + Ki * integral(e) + Kd * derivative

with Kp = 1.0, Ki = 0.2, Kd = 0.05. Anti-windup has two parts: the integral is clamped to
plus or minus 2, and it stops accumulating while the throttle is saturated and the error
still pushes the same way. The derivative is taken on the measured speed so there is no
kick when the target changes, and the change of the throttle per second is limited for
smooth output. With cruise control on, the teleop bridge uses this PID, and the car holds
the requested speed instead of drifting toward the drag limit.

The velocity profiler chooses a target speed from the path curvature kappa, so that the
lateral acceleration stays below a_lat = 5 m/s^2:

    v_target = sqrt(a_lat / |kappa|),  limited to [1.0, 7.5] m/s

On a straight line (|kappa| almost 0) it returns the maximum speed. If the curvature is invalid
it falls back to the default speed.

### 3.4 Lateral PID (Milestone 5.2)

It steers from two errors: the cross-track error (cte, positive when the car is left of the
path) and the heading error (heading of the car minus heading of the path):

    delta = -(Kp cte + Ki integral(cte) + Kd d(cte)/dt) - k_yaw * heading_error

The minus signs matter: a car on the left must steer right. The result is clamped to plus or
minus 35 degrees and the integral is clamped to avoid windup. Gains: Kp 0.6, Ki 0.03, Kd 0.1,
k_yaw 2.2.

### 3.5 Pure Pursuit (Milestone 5.3)

It picks a target point on the path in front of the car and steers along the circle that
reaches it. The look-ahead distance grows with speed:

    Ld = clip(0.25 v + 0.8, 0.8, 2.5)
    delta = atan2(2 L sin(alpha), Ld)

where alpha is the angle between the car heading and the direction to the target point. The
target is the first waypoint at least Ld away, searching forward from the nearest waypoint
(the track is a closed loop, so the search wraps around).

### 3.6 Extended kinematic MPC (Milestone 5.4)

At every step the MPC chooses 10 steering angles and 10 accelerations (a horizon of 1 s) by
simulating the same kinematic model, including drag, and minimizing a cost. The cost is
written in the Frenet frame of the reference path, so errors are measured along and across the
path instead of in x and y:

    cost = sum over k of  w_lat e_lat^2 + w_long e_long^2 + w_yaw e_yaw^2 + w_v e_v^2
                          + w_steer delta^2 + w_dsteer (delta - delta_previous)^2 + w_accel a^2

with w_lat 60, w_long 1, w_yaw 2, w_v 1, w_steer 0.2, w_dsteer 6, w_accel 0.1. The steering is
limited to plus or minus 35 degrees and the acceleration to plus or minus 4 m/s^2. The problem
is solved with SciPy SLSQP. The solution of the previous step, shifted by one step, is used as
the starting point (warm start). Because the command reaches the car about one step late, the
MPC first predicts the state one step ahead with the last command and drops the first
reference point (delay compensation).

In the final version the MPC only steers. The speed comes from the same velocity profiler and
longitudinal PID used by the other controllers, so the three steering controllers are compared
under exactly the same conditions. In my first version the MPC also produced the throttle, and I
changed it for this reason.

### 3.7 Lap analyzer (Milestone 5.5)

It measures the cross-track error as the distance from the car to the path, counts laps when
the car crosses the start line again, and after every lap prints the lap time, mean, RMS and
maximum error, and speeds. It saves one line per lap in a CSV file (with the name of the
controller under test), publishes the live telemetry topics, and draws a line from the car to
the path (green when the error is small, red when it grows) and a scoreboard in RViz. The first
lap starts when the car starts moving, not when the simulator was launched.

## 4. How the controllers were tuned

I used simulation-based tuning, as recommended in the course material, not Ziegler-Nichols.
First I decided what "good" means: a low RMS cross-track error while staying on the track.
Then I tried combinations of parameters in closed loop, one full lap each, using the real
controller code against a copy of the car equations, and kept the combination with the lowest error.
After that I checked everything in the real simulator.

- Lateral PID: a grid search over Kp, Ki, Kd and k_yaw. This search was done at 3 m/s with
  an earlier version of the error calculation, and I did not repeat it at 4 m/s. A later offline
  check at 4 m/s suggested better gains exist (RMS around 0.14 m instead of about 0.23 m),
  but I kept the gains that produced the measured results, so the Lateral PID is probably a little
  weaker here than it could be. It is still about ten times less accurate than the others.
- Pure Pursuit: I tried 9 combinations of the look-ahead speed gain and minimum look-ahead.
  The defaults were within 2 percent of the best, so I kept them.
- MPC: I changed the weights one at a time. The largest improvements did not come from the
  weights but from the delay compensation and from letting the PID handle the speed.
- Longitudinal PID and profiler: the default values already hold 4.00 m/s, so I did not change them.

The tuning scripts were quick exploratory scripts and are not part of this repository.

## 5. Benchmark results

All numbers come from the lap analyzer. The raw lap-by-lap data is in results/bicycle_gym_laps.csv.
The cross-track error (CTE) is the distance from the car to the path in meters. Mean CTE, best
lap and top speed are taken over the laps; the RMS is the square root of the average of the
squared per-lap RMS values. The track half-width is about 1.36 m (distance to the cones).

### 5.1 Constant speed of 4 m/s (same speed logic for every controller)

| Controller Mode | Best Lap Time (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps Completed / Status |
|---|---|---|---|---|---|---|
| Manual teleoperation | not measured | - | - | - | - | works, see section 8 |
| Lateral PID (reactive) | 113.30 | 4.00 | 0.173 | 0.866 | 0.231 | 4 laps, stays on track |
| Pure Pursuit (preview) | 111.20 | 4.04 | 0.012 | 0.124 | 0.021 | 3 laps |
| Extended kinematic MPC (optimal) | 111.20 | 4.00 | 0.006 | 0.100 | 0.012 | 5 laps |
| First MPC version (it also controlled the throttle) | 118.80 | 4.03 | 0.009 | 0.088 | 0.016 | 3 laps |

### 5.2 With the velocity profiler (speed up to 7.5 m/s)

| Controller Mode | Best Lap Time (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps Completed / Status |
|---|---|---|---|---|---|---|
| Lateral PID (reactive) | not run | - | - | - | - | - |
| Pure Pursuit (preview) | 63.04 | 7.58 | 0.040 | 0.210 | 0.055 | 4 laps |
| Extended kinematic MPC (optimal) | 64.61 | 7.64 | 0.169 | 0.679 | 0.237 | 3 laps, stays on track |

Notes:
- The first lap of every run includes the start from standstill.
- One MPC lap (176.6 s, with a normal average speed) was removed because the computer paused during it.
- The Lateral PID was not run with the profiler because in an offline test it left the track.

## 6. Discussion

At 4 m/s the ranking is clear: MPC (0.012 m RMS), then Pure Pursuit (0.021 m), then the
Lateral PID (0.231 m). MPC and Pure Pursuit take the same lap time, so the comparison is fair.
The profiler makes laps about 43 percent faster (111 s to 64 s) at the cost of a larger error.

Why does the MPC track better than the Lateral PID and Pure Pursuit?

- The Lateral PID only sees the error that already exists. It does not know about the next
  curve, and the command reaches the car with a delay, so its corrections arrive late and
  overshoot. This is the oscillation seen in RViz. Raising the gains makes it react faster but also
  makes it less stable, so there is a limit to how good it can become.
- Pure Pursuit looks ahead, which is why it is much better. But it uses a single target point,
  has no idea of limits or smoothness, and one look-ahead distance has to be a compromise:
  short means wobble, long means cutting corners.
- The MPC looks ahead along a whole horizon of 1 s, using the model of the car. Its cost
  balances lateral error, heading error, steering effort and steering rate, so it starts turning
  early and smoothly. It also respects the steering and acceleration limits and compensates the
  delay. That is why it has the smallest error at moderate speed.

At high speed my real result was the opposite: Pure Pursuit (0.055 m) was about four times more
accurate than the MPC (0.237 m). My offline simulation predicted a much smaller MPC error, so the
difference comes from something the offline test did not include. My explanation, which I did not
prove, is timing: the MPC depends on an accurate model of when the command takes effect, and at
7 m/s each 0.1 s of extra delay is 0.7 m of travel. Pure Pursuit is purely geometric and is less
sensitive to this. So the MPC is not always the best controller: it is more accurate at moderate speed,
but it is more expensive to compute and more sensitive to modeling errors.

Limits of these experiments: one track, one machine, one simulator whose model is the same
kinematic model the MPC uses (no tire slip), a few laps per controller, and the Lateral PID tuned
at a different speed.

## 7. Milestone 6: four-wheel Ackermann kinematics

The files are in milestone6/. The bicycle model has one steering angle, but a real car has two
front wheels. For the same turn the inner wheel must steer more than the outer one so that all
wheels roll around the same center. With L = 1.25 m and track T = 1.18 m:

    tan(delta_left)  = L tan(delta) / (L - (T/2) tan(delta))
    tan(delta_right) = L tan(delta) / (L + (T/2) tan(delta))

At the 35 degree limit, the inner wheel needs 46.3 degrees and the outer wheel 27.8 degrees
(18.5 degrees apart). The URDF hinge limit is 35 degrees, so the 3D model cannot show the inner
wheel fully. The minimum turning radius is 1.79 m for the rear axle center. For a turn at
3 m/s and 35 degrees, the four wheels need different speeds (rear wheels 2.0 and 4.0 m/s, front wheels
2.9 and 4.5 m/s), like a differential does. Simulating a 10 s turn with the bicycle model and with
the four-wheel model gives the same position (difference 0.00 mm), which shows that the bicycle model is a correct
description of the body motion. Ignoring Ackermann and steering both wheels at 35 degrees would
leave the inner wheel 11.3 degrees under-steered and cause tire scrub, which a kinematic model
cannot show. Run it with: python3 milestone6/ackermann_analysis.py

## 8. How to reproduce

Install the dependencies and build (the repository goes inside ~/ros2_ws/src):

    source /opt/ros/humble/setup.bash
    sudo apt install -y python3-colcon-common-extensions python3-numpy python3-scipy \
      ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-xacro \
      ros-humble-teleop-twist-keyboard
    cd ~/ros2_ws && colcon build && source install/setup.bash

Run a controller (three terminals, each with the two source commands first):

    Terminal 1:  ros2 launch bicycle_sim bicycle_sim.launch.py controller:=none
    Terminal 2:  ros2 param set /lap_analyzer label mpc
    Terminal 3:  ros2 run bicycle_control controller --ros-args -p control_mode:=mpc \
                   -p velocity_mode:=constant -p target_speed:=4.0

Use control_mode lateral_pid, pure_pursuit or mpc. Use velocity_mode curvature to turn on the
velocity profiler. The label is only a name written in the lap log. After each lap the analyzer
prints its numbers and adds a line to ~/bicycle_gym_laps.csv.

Keyboard driving with cruise control:

    Terminal 1:  ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop use_cruise_control:=true
    Terminal 3:  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Unit tests:

    cd ~/ros2_ws/src/Control_Project/bicycle_control
    python3 -m pytest test/test_longitudinal_pid.py test/test_velocity_profiler.py \
      test/test_lateral_pid.py test/test_pure_pursuit.py test/test_mpc.py

## 9. Note about the track file

The file centerline_0.csv has about one third of its neighbouring waypoints in a locally reversed
order. Summing all waypoint distances gives 528.2 m, but the smooth length of the track is about 444 m
(the car covers about 445 m per lap). Because of this, the heading of each waypoint is computed
from two waypoints on each side instead of the next one, which also fixed the sign of the
cross-track error.

## 10. Main files

- bicycle_sim/bicycle_sim/bicycle_model.py
- bicycle_control/bicycle_control/teleop_bridge.py
- bicycle_control/bicycle_control/longitudinal_pid.py
- bicycle_control/bicycle_control/velocity_profiler.py
- bicycle_control/bicycle_control/lateral_pid.py
- bicycle_control/bicycle_control/pure_pursuit.py
- bicycle_control/bicycle_control/mpc.py
- bicycle_control/bicycle_control/controller_node.py
- track_environment/track_environment/lap_analyzer.py
- milestone6/ackermann_analysis.py and ackermann_output.txt
- results/bicycle_gym_laps.csv
