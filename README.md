# Quadrotor control stack in ROS 2 and Webots

A DJI Mavic 2 Pro in Webots, flown through ROS 2 Jazzy by my own stack:
- a C++ driver plugin that publishes the state and runs the inner loops;
- a C++ position controller;
- a mission layer with services and an action.

The low-level interface (collective thrust + body rates) is the one PX4 exposes, so the position controller can later be swapped for a nonlinear MPC without touching the simulator side.

> Status: work in progress. The stack builds; flight tests and the hover identification are next.

## Architecture

```
 Webots (Windows)                         ROS 2 Jazzy (WSL2, Ubuntu 24.04)
┌────────────────────────────┐  /quad/odom, /quad/imu, TF  ┌─────────────────────────────┐
│ MavicDriver (C++ plugin)   │ ──────────────────────────► │ position_controller (C++)   │
│  GPS, inertial unit, gyro  │                             │  pos → vel → acc → attitude │
│  body-rate PI loop         │ ◄────────────────────────── │  (geometric attitude error) │
│  mixer → 4 propeller speeds│  /quad/cmd/rates_thrust     └──────────────▲──────────────┘
└────────────────────────────┘                                            │ /quad/setpoint
                                                           ┌──────────────┴──────────────┐
           ros2 service call /quad/takeoff, /quad/land ───►│ mission (Python)            │
           ros2 action send_goal /quad/go_to ─────────────►│  setpoint at bounded speed  │
           teleop_twist_keyboard  (/cmd_vel) ─────────────►│  teleop in the yaw frame    │
                                                           └─────────────────────────────┘
```

| Package | Language | Content |
|---|---|---|
| `quad_interfaces` | IDL | `RatesThrust.msg` (thrust [N] + body rates [rad/s]), `GoTo.action` |
| `quad_webots` | C++ | world, driver plugin, mixer (+ gtest), parameters, launch |
| `quad_control` | C++ | position controller, enable/disable service |
| `quad_mission` | Python | takeoff/land services, `go_to` action server, teleop, square demo (action client) |

**Rotor model.** The propeller constants come from `Mavic2Pro.proto` (Webots R2025a):
- thrust f = k_t ω², with k_t = 2.6·10⁻⁴;
- reaction torque k_q ω², with k_q = 5.2·10⁻⁶.

The rotor positions are taken relative to the body centre of mass, which sits 8.8 cm behind the geometric centre, so the rear rotors carry more of the weight at hover. The mixer inverts the 4×4 map from rotor thrusts to (thrust, torque) and clips each rotor to its speed limit.

## Build and run

Requires ROS 2 Jazzy in WSL2 (Ubuntu 24.04), `ros-jazzy-webots-ros2`, and Webots R2025a installed on Windows.

```bash
mkdir -p ~/quad_ws && cd ~/quad_ws
ln -s /mnt/d/Work/quadrotor-ros2/src src     # or clone the repo into src/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test --packages-select quad_webots && colcon test-result --verbose
source install/setup.bash
ros2 launch quad_webots sim.launch.py        # add rviz:=true for RViz
```

Run each of these in another terminal:

```bash
ros2 service call /quad/takeoff std_srvs/srv/Trigger
ros2 action send_goal --feedback /quad/go_to quad_interfaces/action/GoTo "{target: {x: 2.0, y: 1.0, z: 1.5}, yaw: 0.0, tolerance: 0.1}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run quad_mission square_demo
ros2 service call /quad/land std_srvs/srv/Trigger
```

**Safety behaviour in the driver:**
- the motors stay off until the first command arrives;
- if commands stop for 0.3 s, the drone levels itself and sinks at 90% of hover thrust;
- once it is on the ground, the motors stop.

## Roadmap
1. Hover identification (mass, thrust) and gain tuning; flight logs with rosbag2.
2. Nonlinear MPC (acados, SQP-RTI, C++ node) behind the same `RatesThrust` interface.
3. Benchmark against this cascaded controller and a geometric SE(3) controller (trajectory speed, wind, model mismatch, solve times).

## License
MIT
