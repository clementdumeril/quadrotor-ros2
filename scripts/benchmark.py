"""Trajectory-tracking benchmark: one controller, several speeds and disturbances.

    bash scripts/sim.sh controller:=nmpc        # or controller:=pid, in another terminal
    bash scripts/ros.sh python3 scripts/benchmark.py --controller nmpc

For each scenario and speed, the drone flies a figure eight (or a circle) through the
/quad/follow_trajectory action. Every odometry sample (250 Hz) is logged with the
reference at that same instant (the exact path, see bench_metrics.py), the command, and
the NMPC solve time. Errors are taken over the laps flown at top speed only. Disturbances are unknown to the controllers:
  nominal   none
  wind      1 N steady along x with gusts (0.5 N at 0.4 Hz along x, 0.5 N at 0.25 Hz along y);
            1 N is 2 m/s^2 on this 0.5 kg drone
  payload   20 % extra weight that the controllers do not know (a constant downward force)
"""
import argparse
import csv
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from quad_interfaces.action import FollowTrajectory
from quad_interfaces.msg import RatesThrust, ReferenceTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from bench_metrics import apply_exact_reference, metrics

MASS, G = 0.5, 9.81  # the drone, for the payload force
SCENARIOS = {
    "nominal": lambda t: (0.0, 0.0, 0.0),
    "wind": lambda t: (1.0 + 0.5 * math.sin(2 * math.pi * 0.4 * t), 0.5 * math.sin(2 * math.pi * 0.25 * t), 0.0),
    "payload": lambda t: (0.0, 0.0, -0.2 * MASS * G),
}
COLUMNS = ["t", "x", "y", "z", "vx", "vy", "vz", "qw", "qx", "qy", "qz", "x_ref", "y_ref", "z_ref",
           "vx_ref", "vy_ref", "vz_ref", "thrust", "wx_cmd", "wy_cmd", "wz_cmd", "solve_ms", "fx", "fy", "fz"]


def interpolate(ref, t):
    """Reference position and velocity at time t from a ReferenceTrajectory message."""
    t0 = ref.header.stamp.sec + 1e-9 * ref.header.stamp.nanosec
    n = len(ref.position)
    if n == 1 or t <= t0:
        k, f = 0, 0.0
    else:
        s = (t - t0) / ref.dt
        k = min(int(s), n - 2)
        f = min(s - k, 1.0)
    k2 = min(k + 1, n - 1)
    P = lambda a: np.array([a.x, a.y, a.z])
    p = (1 - f) * P(ref.position[k]) + f * P(ref.position[k2])
    v = (1 - f) * P(ref.velocity[k]) + f * P(ref.velocity[k2])
    return p, v


class Bench(Node):
    def __init__(self):
        super().__init__("benchmark")
        self.rows, self.logging = [], False
        self.ref = self.cmd = None
        self.solve_ms = math.nan
        self.force = (0.0, 0.0, 0.0)
        self.t_now = 0.0
        self.create_subscription(Odometry, "/quad/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(ReferenceTrajectory, "/quad/reference", lambda m: setattr(self, "ref", m), 10)
        self.create_subscription(RatesThrust, "/quad/cmd/rates_thrust", lambda m: setattr(self, "cmd", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, "/quad/nmpc/diagnostics",
                                 lambda m: setattr(self, "solve_ms", m.data[0]), 10)
        self.force_pub = self.create_publisher(Vector3, "/quad/disturbance/force", 10)
        self.record_pub = self.create_publisher(String, "/quad/record", 10)
        self.takeoff = self.create_client(Trigger, "/quad/takeoff")
        self.land = self.create_client(Trigger, "/quad/land")
        self.follow = ActionClient(self, FollowTrajectory, "/quad/follow_trajectory")

    def on_odom(self, m):
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        self.t_now = t
        if not self.logging or self.ref is None:
            return
        P, O, V = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        q = np.array([O.w, O.x, O.y, O.z])
        # body -> world velocity
        w, x, y, z = q
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        v = R @ np.array([V.x, V.y, V.z])
        p_ref, v_ref = interpolate(self.ref, t)
        c = self.cmd
        self.rows.append([t, P.x, P.y, P.z, *v, *q, *p_ref, *v_ref,
                          c.thrust if c else math.nan, c.body_rates.x if c else math.nan,
                          c.body_rates.y if c else math.nan, c.body_rates.z if c else math.nan,
                          self.solve_ms, *self.force])


def wait(future, timeout=120.0):
    t0 = time.time()
    while not future.done() and time.time() - t0 < timeout:
        time.sleep(0.05)
    return future.result() if future.done() else None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--controller", required=True, help="label of the running controller (pid or nmpc)")
    ap.add_argument("--scenarios", nargs="+", default=list(SCENARIOS), choices=list(SCENARIOS))
    ap.add_argument("--speeds", nargs="+", type=float, default=[2.0, 4.0, 5.0])
    ap.add_argument("--shape", default="lemniscate")
    ap.add_argument("--size", type=float, default=2.0)
    ap.add_argument("--laps", type=float, default=2.0)
    ap.add_argument("--height", type=float, default=1.5)
    ap.add_argument("--record", default="", help="video file on the Webots host, recorded during the first run")
    ap.add_argument("--out", default="results/benchmark", help="summary JSON here, raw logs in <out>/runs/")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Bench()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    stop = threading.Event()
    scenario = {"f": SCENARIOS["nominal"], "t0": 0.0}

    def push_force():
        while not stop.is_set():
            f = scenario["f"](node.t_now - scenario["t0"])
            node.force = f
            node.force_pub.publish(Vector3(x=float(f[0]), y=float(f[1]), z=float(f[2])))
            time.sleep(0.02)

    threading.Thread(target=push_force, daemon=True).start()

    node.takeoff.wait_for_service()
    print("takeoff:", wait(node.takeoff.call_async(Trigger.Request())).message, flush=True)
    time.sleep(6.0)
    node.follow.wait_for_server()

    summary_path = out / f"benchmark_{args.controller}.json"
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    first = True
    for name in args.scenarios:
        for speed in args.speeds:
            scenario["f"], scenario["t0"] = SCENARIOS[name], node.t_now
            node.rows, node.logging = [], True
            # float() everywhere: an int in a float64 field is not converted by rclpy (it arrives as ~0)
            goal = FollowTrajectory.Goal(shape=args.shape, size=float(args.size), max_speed=float(speed),
                                         laps=float(args.laps), start=Point(x=0.0, y=0.0, z=float(args.height)))
            recording = {"on": False}

            def on_feedback(fb, rec=recording, want=args.record and first):
                # start the video when the path starts (not during the climb to its first point)
                if want and not rec["on"] and fb.feedback.progress > 0.0:
                    node.record_pub.publish(String(data=args.record))
                    rec["on"] = True

            handle = wait(node.follow.send_goal_async(goal, feedback_callback=on_feedback))
            if not handle or not handle.accepted:
                print(f"{name} {speed} m/s: goal rejected", flush=True)
                continue
            res = wait(handle.get_result_async(), timeout=600.0)
            node.logging = False
            if recording["on"]:
                node.record_pub.publish(String(data=""))
            first = False
            if res is None or not res.result.success:
                print(f"{name} {speed} m/s: no result (timeout or failure), skipped", flush=True)
                continue
            r = res.result
            t_start = r.start_time.sec + 1e-9 * r.start_time.nanosec
            tag = f"{args.controller}_{args.shape}_{name}_{speed:g}"
            entry = {"controller": args.controller, "shape": args.shape, "size": args.size, "scenario": name,
                     "max_speed": speed, "laps": args.laps}
            # metrics against the exact path (see bench_metrics.py)
            rows = apply_exact_reference(np.array(node.rows), entry, t_start)
            entry.update(metrics(rows, entry, t_start))
            (out / "runs").mkdir(exist_ok=True)
            with open(out / "runs" / f"run_{tag}.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(COLUMNS)
                w.writerows(rows.tolist())
            summary[tag] = entry
            summary_path.write_text(json.dumps(summary, indent=2))
            print(f"{tag}: rmse {entry['rmse'] * 100:.1f} cm, max {entry['max_error'] * 100:.1f} cm, "
                  f"tilt {entry['max_tilt_deg']:.0f} deg"
                  + (f", solve {entry['solve_ms_median']:.2f} ms median" if entry["solve_ms_median"] else ""),
                  flush=True)

    scenario["f"] = SCENARIOS["nominal"]
    time.sleep(1.0)
    print("land:", wait(node.land.call_async(Trigger.Request())).message, flush=True)
    time.sleep(6.0)
    stop.set()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
