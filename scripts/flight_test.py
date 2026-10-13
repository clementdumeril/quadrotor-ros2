"""Scripted flight test with logging: takeoff, hover, go_to, land.

    bash scripts/ros.sh python3 scripts/flight_test.py [--out log.csv] [--target 2 0 1]

Logs odometry, setpoint and command at the odometry rate and prints simple metrics.
"""
import argparse
import csv
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from quad_interfaces.action import GoTo
from quad_interfaces.msg import RatesThrust
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


def rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    return roll, pitch, yaw


class FlightTest(Node):
    def __init__(self):
        super().__init__("flight_test")
        self.rows, self.sp, self.cmd, self.motors = [], None, None, [math.nan] * 4
        self.create_subscription(Odometry, "/quad/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/quad/setpoint", lambda m: setattr(self, "sp", m), 10)
        self.create_subscription(RatesThrust, "/quad/cmd/rates_thrust", lambda m: setattr(self, "cmd", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, "/quad/motor_speeds",
                                 lambda m: setattr(self, "motors", list(m.data)), 10)
        self.takeoff = self.create_client(Trigger, "/quad/takeoff")
        self.land = self.create_client(Trigger, "/quad/land")
        self.go_to = ActionClient(self, GoTo, "/quad/go_to")

    def on_odom(self, m):
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        p = m.pose.pose.position
        r, pi, y = rpy(m.pose.pose.orientation)
        sp = self.sp.pose.position if self.sp else None
        c = self.cmd
        w = m.twist.twist.angular
        self.rows.append([t, p.x, p.y, p.z, r, pi, y,
                          sp.x if sp else math.nan, sp.y if sp else math.nan, sp.z if sp else math.nan,
                          c.thrust if c else math.nan, w.x, w.y, w.z,
                          c.body_rates.x if c else math.nan, c.body_rates.y if c else math.nan,
                          c.body_rates.z if c else math.nan] + self.motors)


def wait(node, future, timeout=60.0):
    t0 = time.time()
    while not future.done() and time.time() - t0 < timeout:
        time.sleep(0.05)
    return future.result() if future.done() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="flight_log.csv")
    ap.add_argument("--target", nargs=3, type=float, default=[2.0, 0.0, 1.0])
    ap.add_argument("--hover", type=float, default=8.0)
    ap.add_argument("--square", type=float, default=0.0, help="side of a square flown at the target height [m]")
    args = ap.parse_args()

    rclpy.init()
    node = FlightTest()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    node.takeoff.wait_for_service()
    print("takeoff:", wait(node, node.takeoff.call_async(Trigger.Request())).message)
    time.sleep(args.hover)
    t_hover_end = node.rows[-1][0]

    node.go_to.wait_for_server()
    a, h = args.square, args.target[2]
    targets = [(a, 0.0, h), (a, a, h), (0.0, a, h), (0.0, 0.0, h)] if a > 0 else [tuple(args.target)]
    for tx, ty, tz in targets:
        goal = GoTo.Goal(target=Point(x=tx, y=ty, z=tz), yaw=0.0, tolerance=0.1)
        handle = wait(node, node.go_to.send_goal_async(goal))
        res = wait(node, handle.get_result_async(), timeout=60.0) if handle and handle.accepted else None
        print(f"go_to ({tx}, {ty}, {tz}):", "accepted" if handle and handle.accepted else "rejected",
              f"final distance {res.result.final_distance:.3f} m" if res else "(no result)")
    time.sleep(3.0)
    print("land:", wait(node, node.land.call_async(Trigger.Request())).message)
    time.sleep(8.0)

    rows = node.rows
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "roll", "pitch", "yaw", "sp_x", "sp_y", "sp_z", "thrust",
                    "wx", "wy", "wz", "wx_cmd", "wy_cmd", "wz_cmd", "m_fl", "m_fr", "m_rl", "m_rr"])
        w.writerows(rows)

    hover = [r for r in rows if t_hover_end - 3.0 <= r[0] <= t_hover_end]
    if hover:
        ez = [r[3] - r[9] for r in hover]
        exy = [math.hypot(r[1] - r[7], r[2] - r[8]) for r in hover]
        thr = [r[10] for r in hover if not math.isnan(r[10])]
        print(f"hover (last 3 s): z error mean {sum(ez)/len(ez):+.3f} m, max |xy| {max(exy):.3f} m, "
              f"mean thrust {sum(thr)/max(len(thr),1):.3f} N  -> mass estimate {sum(thr)/max(len(thr),1)/9.81:.3f} kg")
    print(f"max tilt {math.degrees(max(max(abs(r[4]), abs(r[5])) for r in rows)):.1f} deg, "
          f"final z {rows[-1][3]:.3f} m, {len(rows)} samples -> {args.out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
