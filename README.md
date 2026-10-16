# Quadrotor control in ROS 2 and Webots: nonlinear MPC vs. cascaded control

![NMPC flying a figure eight at 5 m/s in gusty wind](docs/nmpc_figure_eight.gif)

I fly a DJI Mavic 2 Pro in Webots through ROS 2 Jazzy with my own C++ stack, and compare two controllers behind the same interface (collective thrust and body rates, as PX4 exposes it).

The nonlinear MPC (`nmpc_controller`, C++) uses a quaternion model, 20 steps over 1 s, solved at 100 Hz by acados (SQP real-time iteration, HPIPM) from generated C code, with a disturbance observer for offset-free tracking. The cascaded controller (`position_controller`, C++) is PID on position and velocity, Lee's geometric attitude control, and feedforward of the reference acceleration and of the body rates from its jerk (differential flatness). The simulator side is a C++ Webots plugin (explicit rotor model, body-rate loop, mixer, wind and payload test inputs) plus a mission layer with ROS 2 services and actions.

## Results

The test is a 4 m × 2 m figure eight at 1.5 m height, top speeds of 2, 4 and 5 m/s (5 m/s needs about 60° of tilt), two laps at top speed. Error is measured against the exact path at 250 Hz. The simulator applies wind and payload, and neither controller knows about them.

| Scenario | Top speed | Cascaded PID: RMSE / max error | NMPC: RMSE / max error | NMPC solve time: median / p99 / max |
|---|---|---|---|---|
| Nominal | 2 m/s | 0.1 / 0.4 cm | 0.2 / 0.3 cm | 0.49 / 1.00 / 2.43 ms |
| Nominal | 4 m/s | 1.0 / 1.9 cm | 0.7 / 1.6 cm | 0.48 / 0.78 / 2.20 ms |
| Nominal | 5 m/s | 2.3 / 4.1 cm | 1.5 / 3.1 cm | 0.49 / 1.16 / 1.43 ms |
| Wind 1 N + gusts ±0.5 N | 2 m/s | 7.1 / 10.8 cm | 3.6 / 5.1 cm | 0.46 / 0.79 / 1.62 ms |
| Wind 1 N + gusts ±0.5 N | 4 m/s | 6.9 / 10.5 cm | 3.2 / 4.9 cm | 0.48 / 0.89 / 5.99 ms |
| Wind 1 N + gusts ±0.5 N | 5 m/s | 7.3 / 10.8 cm | 3.1 / 4.6 cm | 0.50 / 0.86 / 1.46 ms |
| Hidden payload, +20 % weight | 2 m/s | 1.4 / 2.4 cm | 0.2 / 0.3 cm | 0.49 / 0.81 / 2.12 ms |
| Hidden payload, +20 % weight | 4 m/s | 0.9 / 1.6 cm | 0.6 / 1.0 cm | 0.49 / 0.90 / 1.68 ms |
| Hidden payload, +20 % weight | 5 m/s | 1.9 / 3.6 cm | 1.1 / 2.2 cm | 0.53 / 1.02 / 2.82 ms |

![Top view and position error at 5 m/s](docs/benchmark_paths.png)

![RMSE against top speed](docs/benchmark_rmse.png)

The NMPC runs in real time on a 2016 laptop CPU (i7-6700HQ, in WSL2): 0.46 to 0.53 ms median per run, under 1.2 ms in 99 % of steps and 6.0 ms at worst, against a 10 ms period.

In gusty wind the NMPC halves the error at every speed, 3.1 to 3.6 cm against 6.9 to 7.3 cm. Its observer puts the estimated wind into the model over the whole horizon, while the cascaded controller's integrator only reacts once an error has built up.

Without disturbance, or with the hidden payload, both stay within about 2 cm up to 4 m/s. The NMPC pulls ahead near the thrust and tilt limits (5 m/s: 1.5 vs. 2.3 cm, and 1.1 vs. 1.9 cm with the payload), and the integrator lags again with the payload at low speed (2 m/s: 1.4 vs. 0.2 cm). That matches Sun et al. (T-RO 2022): with flatness feedforward, a cascaded controller stays close to an NMPC on feasible references, and the NMPC gains near the limits and against unknown disturbances.

Both controllers get the same thrust (2× weight) and rate (6/6/3 rad/s) limits, the same mass and the same reference. The cascaded gains come from a sweep of four settings at 4 m/s (I kept the stiffest stable one; the next one diverged), the NMPC weights from three. My first cascaded version had no body-rate feedforward and was about 50 cm off at 4 m/s, which would have made the NMPC look far better than it is.

The limits of all this: one simulator, one drone, no sensor noise, and a close model. The NMPC knows the mass, but not the inner rate loop, the 30 ms motor lag or the disturbance. Real flight adds delays and larger model errors, and both controllers will do worse.

### Three bugs that changed the numbers

1. The driver stamped odometry with the ROS clock, which follows `/clock` one simulation step late. A 4 ms stamp error is 2 cm at 5 m/s, and it biased every error. Measurements now carry the Webots time of their step.
2. rclpy accepted `laps=2` (a Python int) for a `float64` field and the action server received about 0, so the first runs flew no lap at top speed.
3. Stopping a background `ros2 launch` with SIGINT does nothing in a script, so one run had both controllers driving the drone. `scripts/run_benchmark.sh` now stops each stack with SIGTERM and checks that it is gone.

## Architecture

```
 Webots (Windows)                          ROS 2 Jazzy (WSL2, Ubuntu 24.04)
┌────────────────────────────┐  /quad/odom, /quad/imu, TF   ┌──────────────────────────────────┐
│ MavicDriver (C++ plugin)   │ ───────────────────────────► │ nmpc_controller (C++, acados)    │
│  GPS, inertial unit, gyro  │                              │   or                             │
│  rotor model, 250 Hz       │ ◄─────────────────────────── │ position_controller (C++)        │
│  body-rate PI loop, mixer  │  /quad/cmd/rates_thrust      └────────────────▲─────────────────┘
│  test inputs: wind force,  │                                               │ /quad/reference
│  video recording           │                              ┌────────────────┴─────────────────┐
└────────────────────────────┘    /quad/takeoff, /land ───► │ mission (Python)                 │
                                  /quad/go_to (action) ───► │  hover, go_to, teleop: a point   │
                                  /quad/follow_trajectory ► │  trajectory: next 1.5 s of path  │
                                  /cmd_vel (teleop) ──────► └──────────────────────────────────┘
```

The live graph while the NMPC flies, drawn by `scripts/graph_dot.py`:

![ROS 2 graph](docs/ros_graph.png)

| Package | Language | Content |
|---|---|---|
| `quad_interfaces` | IDL | `RatesThrust.msg`, `ReferenceTrajectory.msg` (time-stamped samples of p, v, a, yaw), `GoTo.action`, `FollowTrajectory.action` |
| `quad_webots` | C++ | world, driver plugin, mixer (+ gtest), parameters, launch (`controller:=pid` or `nmpc`) |
| `quad_control` | C++ | NMPC node and its generated solver, cascaded controller, shared reference code (+ gtest) |
| `quad_mission` | Python | takeoff/land services, `go_to` and `follow_trajectory` actions, trajectories (+ pytest), teleop |

## The nonlinear MPC

`src/quad_control/nmpc/generate_solver.py` builds the problem with CasADi and acados and writes C code to `nmpc/c_generated_code/` (committed, so building only needs the acados libraries).

State x = (p, v, q), 10 values; input u = (T, ω); parameters (m, d):

ṗ = v, v̇ = (T/m) R(q) e₃ − g e₃ + d, q̇ = ½ q ⊗ (0, ω)

The inner loop of the driver tracks the body rates. The problem has 20 shooting nodes of 50 ms, explicit RK4, linear least squares on (p, v, q, T, ω) against the reference, and bounds on thrust (0.5 N to 2× weight) and body rates. At each node the reference attitude and thrust come from the reference acceleration and yaw (differential flatness), and the quaternion is kept in the hemisphere of the previous one. Each control step runs one SQP real-time iteration (Gauss-Newton Hessian, HPIPM with partial condensing, warm start), at 100 Hz on simulation time.

The disturbance observer takes the external acceleration d as the low-pass filtered difference between the measured and predicted velocity change (0.3 s time constant) and passes it to the model as a parameter. Without it, an MPC holds a steady offset under wind or payload, which the integrator of the cascaded controller does not. The node publishes the command, solve time and disturbance estimate on `/quad/nmpc/diagnostics`, and the predicted path on `/quad/nmpc/predicted_path` for RViz.

## The cascaded controller

`position_controller` follows the same `/quad/reference`:
- velocity command = reference velocity + saturated P on position
- desired acceleration = reference acceleration + PI on velocity (the integral absorbs wind and payload)
- desired attitude from the thrust direction and yaw, attitude error e_R = ½ vee(R_dᵀR − RᵀR_d) (Lee, Leok & McClamroch 2010)
- body rates = −k e_R + RᵀR_d ω_d, where ω_d follows from the jerk of the reference (Mellinger & Kumar 2011)

The last term matters most at speed. Without it the attitude only turns once an error has built up, and the error at 4 m/s was 49 cm instead of 1 cm.

## Rotor model and identification

I don't use Webots' own propeller model. In open-loop tests (`scripts/motor_ident.py`), its thrust followed motor speed commands with a lag of 1 to 3 s: after a step to well above hover speed, the drone stayed on the ground for over 2 s before lifting off. No controller can stabilize an actuator like that, and Webots doesn't document the behavior.

So the driver models the rotors itself, as RotorS and RotorPy do. Each motor has a first-order speed response (30 ms time constant). Thrust f = k_t ω² is applied at each `Propeller.centerOfThrust`, with a reaction torque k_q ω², through the supervisor API, using the PROTO's constants k_t = 2.6·10⁻⁴ and k_q = 5.2·10⁻⁶. The Webots propeller motors stay at zero speed, so Webots adds no thrust of its own.

I measured the center of mass and the mass rather than assuming them. The center of mass is at (−6.1, 0.0, −3.6) cm in the body frame, read from the supervisor API at start-up (the bare body is at −8.8 cm; the propellers and gimbal move it forward). With it, the front and rear arms are almost equal (11.6 cm). The mass is 0.50 kg, from the steady hover thrust of the position controller (4.80–4.97 N over three flights). The mixer inverts the 4×4 map from rotor thrusts to (thrust, torque) and clips each rotor to its speed limit.

Two more simulator details: Webots puts resting bodies to sleep after 1 s, so the world sets `physicsDisableTime` high, and `/clock` comes from the `Ros2Supervisor` node, so every node runs on simulation time.

## Build and run

You need ROS 2 Jazzy in WSL2 (Ubuntu 24.04), `ros-jazzy-webots-ros2`, Webots R2025a on Windows, and for the NMPC the acados libraries (built once, no root needed):

```bash
git clone --branch v0.5.5 --depth 1 --recurse-submodules --shallow-submodules https://github.com/acados/acados.git ~/acados
cd ~/acados && mkdir build && cd build
cmake -DACADOS_WITH_QPOASES=OFF -DACADOS_INSTALL_DIR=$HOME/acados .. && make install -j4
```

Without acados everything else still builds; only `nmpc_controller` is skipped. Regenerating the solver (after changing the model or the weights) also needs Python with CasADi 3.7 and `acados_template`: `python src/quad_control/nmpc/generate_solver.py`, and `--test` closes the loop on the model itself.

```bash
mkdir -p ~/quad_ws && cd ~/quad_ws
ln -s /mnt/d/Work/quadrotor-ros2/src src     # or clone the repo into src/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test && colcon test-result --verbose  # 17 tests: mixer, reference, attitude, trajectories
source install/setup.bash
ros2 launch quad_webots sim.launch.py controller:=nmpc    # or controller:=pid; add rviz:=true for RViz
```

Then, each in another terminal:

```bash
ros2 service call /quad/takeoff std_srvs/srv/Trigger
ros2 action send_goal --feedback /quad/follow_trajectory quad_interfaces/action/FollowTrajectory "{shape: lemniscate, size: 2.0, max_speed: 4.0, laps: 2.0, start: {x: 0.0, y: 0.0, z: 1.5}}"
ros2 action send_goal --feedback /quad/go_to quad_interfaces/action/GoTo "{target: {x: 2.0, y: 1.0, z: 1.5}, yaw: 0.0, tolerance: 0.1}"
ros2 topic pub -r 20 /quad/disturbance/force geometry_msgs/msg/Vector3 "{x: 1.0}"    # 1 N of wind
ros2 service call /quad/land std_srvs/srv/Trigger
```

Write floats in actions as floats (`2.0`, not `2`); see bug 2 above.

To reproduce the benchmark (about 35 minutes per controller):

```bash
bash scripts/run_benchmark.sh C:/Users/me/flight.mp4     # both controllers, then the recorded flight
python3 scripts/plot_benchmark.py                        # figures in docs/ and the table above
python scripts/make_gif.py C:/Users/me/flight.mp4 docs/nmpc_figure_eight.gif --crop 1280:440:0:170
```

Summaries are in `results/benchmark/benchmark_{pid,nmpc}.json`. Raw 250 Hz logs go to `results/benchmark/runs/` (not committed), and `scripts/bench_metrics.py` recomputes every metric from them.

### Safety behavior

The motors stay off until the first command arrives. If commands stop for 0.3 s, the driver levels the drone and descends at 0.4 m/s on a hover thrust learned in flight, so a wrong mass parameter can't make it climb (an early version did exactly that). The motors stop once the drone is on the ground, and landing only ends when it is at its takeoff height and has stayed still for 0.5 s.

### Windows and WSL2

Webots runs on Windows and the stack in WSL2; `scripts/sim.sh` sets `WEBOTS_HOME`. The launch file passes the Windows host IP (the WSL default gateway) to the controllers, because webots_ros2 reads it from `/etc/resolv.conf`, which is wrong under systemd-resolved. The Windows firewall must let WSL reach Webots on TCP 1234, for example with a rule limited to `172.16.0.0/12` and `webots-bin.exe`.

### Other scripts

`scripts/flight_test.py` flies a scripted mission (`--square 2`) and logs it to CSV, and `scripts/plot_flight.py` plots it. `scripts/motor_ident.py` (the rotor tests) and `scripts/graph_dot.py` (the ROS graph) are described above.

## Next steps

1. The same nodes on PX4 SITL, through `VehicleRatesSetpoint` (same thrust and body-rate split).
2. Learned residual dynamics in the NMPC model (a small PyTorch network through L4CasADi), tested against unmodeled drag added to the simulator.
3. Delay compensation: predict the state over the command latency before solving.
4. Hardware: the same controller on a small quadrotor with motion capture.

## License

MIT
