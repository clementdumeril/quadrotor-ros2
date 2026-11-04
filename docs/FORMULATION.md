# Controllers: formulation and tuning

Back to the [README](../README.md). Values are copied from the files linked in each section.

## NMPC ([`generate_solver.py`](../src/quad_control/nmpc/generate_solver.py), [`nmpc_controller.cpp`](../src/quad_control/src/nmpc_controller.cpp))

World frame ENU, body frame FLU.

| | |
|---|---|
| State | $`x = [p\ (3),\ v\ (3),\ q = (q_w, q_x, q_y, q_z)]`$, 10 |
| Input | $`u = [T\ \mathrm{(N)},\ \omega\ \mathrm{(rad/s)}\ (3)]`$, 4 |
| Parameters | mass $`m`$ (kg), external acceleration $`d`$ (m/s², 3) |
| Model | $`\dot p = v,\ \dot v = (T/m) R(q) e_3 - g e_3 + d,\ \dot q = \tfrac12 q \otimes (0, \omega)`$ |
| Horizon | $`N = 20`$ steps of 50 ms (1 s), solved at 100 Hz |
| Cost | linear least squares on $`[x, u]`$ against the reference; terminal on $`x`$, scaled by 2 |
| Weights | position 500, 500, 800; velocity 20; quaternion 50 per component; thrust 0.5; body rates 0.5 |
| Bounds | $`T \in [0.5, 9.81]`$ N (thrust-to-weight 2), $`\lvert\omega\rvert \le (6, 6, 3)`$ rad/s; no tilt constraint |
| Solver | SQP real-time iteration, Gauss-Newton, HPIPM with partial condensing (5), explicit RK4, QP warm start |

At each node the reference attitude and thrust come from the reference acceleration (differential flatness). The body rates are assumed to be tracked by the inner loop of the driver, as in Kamel et al. (2017) and Sun et al. (2022).

The weights were chosen from three settings flown at 4 m/s, nominal and in wind (comment in `generate_solver.py`):

| Weights (position / velocity / quaternion / thrust / rates) | Nominal RMSE | Wind RMSE |
|---|---|---|
| 200 / 10 / 20 / 0.5 / 1 | 2.1 cm | 5.0 cm |
| 500-800 / 20 / 50 / 0.5 / 0.5 (kept) | 2.0 cm | 3.7 cm |
| 1000 / 30 / 100 / 0.5 / 0.2 | 3.2 cm (over-reacted) | not given |

The tuning flights are not in `results/`, and these numbers (from the comment) are higher than the benchmark's 0.7 cm nominal at 4 m/s, so they are not on the same basis as the README table.

Disturbance observer: $`\hat d \leftarrow \hat d + \alpha\,(a_{\mathrm{meas}} - a_{\mathrm{model}} - \hat d)`$ with $`\alpha = \Delta t / (\tau + \Delta t)`$, $`\tau = 0.3`$ s, $`a_{\mathrm{meas}}`$ from the finite difference of the measured velocity, $`a_{\mathrm{model}} = (T/m) R(q) e_3 - g e_3`$ with the last thrust, saturated at 4 m/s², active only in flight ([`nmpc_controller.yaml`](../src/quad_control/config/nmpc_controller.yaml)).

## Cascaded baseline ([`position_controller.cpp`](../src/quad_control/src/position_controller.cpp))

Position, velocity, acceleration, attitude, body rates, with feedforward of the reference velocity and acceleration; the saturations act on the feedback terms only.

1. Position to velocity: P, $`k_p = (3.5, 3.5, 4.0)`$ 1/s.
2. Velocity to acceleration: PI, $`k_p = (7, 7, 8)`$ 1/s, $`k_i = (0.3, 0.3, 1.0)`$ 1/s².
3. Acceleration to desired attitude: the thrust axis points along the desired force; tilt capped at 60°.
4. Attitude to body rates: geometric controller of Lee et al. (2010), $`e_R = \tfrac12 (R_d^\top R - R^\top R_d)^\vee`$, $`\omega = -k_{att}\, e_R + R^\top R_d\, \omega_d`$, $`k_{att} = (15, 15, 5)`$.
5. Feedforward rates $`\omega_d`$ from the reference jerk (Mellinger and Kumar 2011).

Gains from a short sweep at 4 m/s on the figure eight: the stiffest stable setting; the next step diverged ([`position_controller.yaml`](../src/quad_control/config/position_controller.yaml)). Same mass (0.50 kg), thrust limit and body-rate limits as the NMPC.

## Driver plugin ([`mavic_driver.cpp`](../src/quad_webots/src/mavic_driver.cpp), [`mavic.yaml`](../src/quad_webots/config/mavic.yaml))

Runs at the Webots step, 4 ms (250 Hz): reads GPS, inertial unit and gyro, publishes `/quad/odom` stamped with Webots time, runs a PI loop on the body rates, inverts the 4 x 4 mixer and applies the rotor forces through the supervisor API, with a first-order motor (30 ms) and thrust $`k_t \omega^2`$. Without a command for 0.3 s it levels the body and descends at 0.4 m/s. Rotor positions and thrust and torque constants come from the Mavic2Pro PROTO of Webots R2025a; the mass from the hover thrust in two flights (4.80 to 4.94 N).
