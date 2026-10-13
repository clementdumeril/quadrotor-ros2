"""Open-loop identification on the ground: which way each rotor pair tips the drone,
and at which common speed it lifts off.

    bash scripts/ros.sh python3 scripts/motor_ident.py

Publishes /quad/cmd/motor_speeds_raw (bypasses the rate loop and mixer); signs follow the
spin directions FL +, FR -, RL -, RR +.
"""
import math
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

SPIN = [1, -1, -1, 1]


def rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class Ident(Node):
    def __init__(self):
        super().__init__("motor_ident")
        self.pub = self.create_publisher(Float64MultiArray, "/quad/cmd/motor_speeds_raw", 10)
        self.odom = None
        self.create_subscription(Odometry, "/quad/odom", lambda m: setattr(self, "odom", m), qos_profile_sensor_data)

    def state(self):
        p = self.odom.pose.pose.position
        r, pi = rpy(self.odom.pose.pose.orientation)
        return p.x, p.z, math.degrees(r), math.degrees(pi)

    def hold(self, speeds, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.pub.publish(Float64MultiArray(data=[s * d for s, d in zip(speeds, SPIN)]))
            time.sleep(0.02)


def main():
    rclpy.init()
    n = Ident()
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    while n.odom is None:
        time.sleep(0.1)

    for name, speeds in [("front pair", [130, 130, 0, 0]), ("rear pair", [0, 0, 130, 130]),
                         ("left pair", [130, 0, 130, 0]), ("right pair", [0, 130, 0, 130])]:
        trace = []
        for _ in range(8):
            n.hold(speeds, 0.1)
            trace.append(n.state())
        n.hold([0, 0, 0, 0], 3.0)
        print(f"{name:10s} roll: " + " ".join(f"{s[2]:+5.1f}" for s in trace))
        print(f"{'':10s} pitch:" + " ".join(f"{s[3]:+5.1f}" for s in trace))
        print(f"{'':10s} z:    " + " ".join(f"{s[1]:+5.2f}" for s in trace))

    lift = None
    for s in (range(60, 161, 2) if "--lift" in sys.argv else []):
        n.hold([s, s, s, s], 0.4)
        x, z, r, p = n.state()
        if z > 0.12:
            lift = s
            break
    n.hold([0, 0, 0, 0], 3.0)
    if "--lift" not in sys.argv:
        pass
    elif lift:
        thrust = 4 * 2.6e-4 * lift ** 2
        print(f"lift-off at {lift} rad/s per rotor -> total thrust {thrust:.2f} N -> mass <= {thrust / 9.81:.3f} kg")
    else:
        print("no lift-off up to 160 rad/s")
    ex.shutdown()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
