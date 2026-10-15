"""Generate the C code of the quadrotor NMPC with acados.

    ~/venvs/nmpc/bin/python src/quad_control/nmpc/generate_solver.py

Writes src/quad_control/nmpc/c_generated_code/, which is committed: building the ROS 2
package then needs only the acados libraries, not Python.

Model (world frame ENU, body frame FLU), the same interface as the cascaded controller:
    state   x = [p (3), v (3), q = (qw, qx, qy, qz)]          10
    input   u = [collective thrust T [N], body rates w [rad/s] (3)]   4
    params  [mass m [kg], external acceleration d [m/s^2] (3)]       4
    p' = v,  v' = (T/m) R(q) e3 - g e3 + d,  q' = 1/2 q (x) (0, w)
The body rates are assumed to be tracked by the inner loop of the driver, as in
Sun et al. (T-RO 2022) and Kamel et al. (2017). d comes from a disturbance observer in
the controller node; it gives offset-free tracking under wind or an unmodelled payload.

Optimal control problem: N = 20 steps of 50 ms (1 s horizon), linear least squares on
[x, u] against the reference, thrust and rate bounds, solved by SQP real-time
iteration (one Gauss-Newton QP per control step, HPIPM, partial condensing).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

HERE = Path(__file__).resolve().parent
G = 9.81
N = 20
DT = 0.05

# Weights on [p, v, q, T, w]; terminal weights on [p, v, q]. Chosen from three settings
# flown at 4 m/s (nominal and wind): 200/10/20/0.5/1 gave 2.1 and 5.0 cm RMSE, these gave
# 2.0 and 3.7 cm, and 1000/30/100/0.5/0.2 over-reacted (3.2 cm nominal).
W_P = [500.0, 500.0, 800.0]
W_V = [20.0, 20.0, 20.0]
W_Q = [50.0, 50.0, 50.0, 50.0]
W_T = [0.5]
W_W = [0.5, 0.5, 0.5]
TERMINAL_SCALE = 2.0

# Default input bounds (changed at run time by the node's parameters).
T_MIN, T_MAX = 0.5, 2.0 * 0.5 * G
W_MAX = [6.0, 6.0, 3.0]


def quadrotor_model() -> AcadosModel:
    p = ca.SX.sym("p", 3)
    v = ca.SX.sym("v", 3)
    q = ca.SX.sym("q", 4)
    thrust = ca.SX.sym("T")
    w = ca.SX.sym("w", 3)
    mass = ca.SX.sym("m")
    d = ca.SX.sym("d", 3)
    x = ca.vertcat(p, v, q)
    u = ca.vertcat(thrust, w)

    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    z_body = ca.vertcat(2 * (qw * qy + qx * qz), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy))
    q_dot = 0.5 * ca.vertcat(
        -qx * w[0] - qy * w[1] - qz * w[2],
        qw * w[0] + qy * w[2] - qz * w[1],
        qw * w[1] + qz * w[0] - qx * w[2],
        qw * w[2] + qx * w[1] - qy * w[0],
    )
    f = ca.vertcat(v, thrust / mass * z_body - ca.vertcat(0, 0, G) + d, q_dot)

    model = AcadosModel()
    model.name = "quadrotor"
    model.x, model.u, model.p = x, u, ca.vertcat(mass, d)
    model.xdot = ca.SX.sym("xdot", 10)
    model.f_expl_expr = f
    model.f_impl_expr = model.xdot - f
    return model


def build_ocp() -> AcadosOcp:
    ocp = AcadosOcp()
    ocp.model = quadrotor_model()
    nx, nu = 10, 4
    ny = nx + nu

    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * DT

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W = np.diag(W_P + W_V + W_Q + W_T + W_W)
    ocp.cost.W_e = TERMINAL_SCALE * np.diag(W_P + W_V + W_Q)
    ocp.cost.Vx = np.vstack([np.eye(nx), np.zeros((nu, nx))])
    ocp.cost.Vu = np.vstack([np.zeros((nx, nu)), np.eye(nu)])
    ocp.cost.Vx_e = np.eye(nx)
    hover = np.array([0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0.5 * G, 0, 0, 0], dtype=float)
    ocp.cost.yref = hover
    ocp.cost.yref_e = hover[:nx]

    ocp.constraints.idxbu = np.arange(nu)
    ocp.constraints.lbu = np.array([T_MIN] + [-x for x in W_MAX])
    ocp.constraints.ubu = np.array([T_MAX] + W_MAX)
    ocp.constraints.x0 = hover[:nx]
    ocp.parameter_values = np.array([0.5, 0.0, 0.0, 0.0])

    o = ocp.solver_options
    o.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    o.qp_solver_cond_N = 5
    o.hessian_approx = "GAUSS_NEWTON"
    o.integrator_type = "ERK"
    o.sim_method_num_stages = 4
    o.sim_method_num_steps = 1
    o.nlp_solver_type = "SQP_RTI"
    o.qp_solver_warm_start = 1
    return ocp


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=str(HERE / "c_generated_code"))
    ap.add_argument("--test", action="store_true", help="also build the solver and run a closed-loop check")
    args = ap.parse_args()

    ocp = build_ocp()
    ocp.code_export_directory = args.out
    json_file = str(Path(args.out).parent / "quadrotor_ocp.json")
    if not args.test:
        AcadosOcpSolver.generate(ocp, json_file=json_file)
        print("C code ->", args.out)
        return

    # Closed-loop check on the model itself: hover at (0, 0, 1) from 1 m away.
    solver = AcadosOcpSolver(ocp, json_file=json_file)
    x = np.array([1.0, -0.5, 0.5, 0, 0, 0, 1, 0, 0, 0])
    from acados_template import AcadosSim, AcadosSimSolver
    sim = AcadosSim()
    sim.model = quadrotor_model()
    sim.solver_options.T = 0.01
    sim.parameter_values = np.array([0.5, 0.0, 0.0, 0.0])
    integrator = AcadosSimSolver(sim, json_file=str(Path(args.out).parent / "quadrotor_sim.json"))
    times = []
    for _ in range(300):
        solver.set(0, "lbx", x)
        solver.set(0, "ubx", x)
        status = solver.solve()
        times.append(solver.get_stats("time_tot"))
        u = solver.get(0, "u")
        x = integrator.simulate(x=x, u=u)
        x[6:] /= np.linalg.norm(x[6:])
    print(f"status {status}, final position {np.round(x[:3], 3)}, "
          f"solve time median {1e3 * np.median(times):.2f} ms, max {1e3 * np.max(times):.2f} ms")


if __name__ == "__main__":
    main()
