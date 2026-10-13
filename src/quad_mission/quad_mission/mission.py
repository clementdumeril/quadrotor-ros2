"""Mission layer: turns high-level requests into position setpoints.

Interfaces
- services  /quad/takeoff, /quad/land            (std_srvs/Trigger)
- action    /quad/go_to                          (quad_interfaces/GoTo)
- topic in  /cmd_vel (geometry_msgs/Twist)       keyboard teleop, body-yaw frame
- topic in  /quad/odom
- topic out /quad/setpoint (PoseStamped, 20 Hz)  read by the position controller
- client    /quad/controller/enable              (std_srvs/SetBool)

The setpoint moves at a bounded speed toward each goal, so the controller never sees a
step of several metres.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from quad_interfaces.action import GoTo
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool, Trigger


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Mission(Node):
    def __init__(self):
        super().__init__("mission")
        self.takeoff_height = self.declare_parameter("takeoff_height", 1.0).value
        self.cruise_speed = self.declare_parameter("cruise_speed", 1.0).value
        self.teleop_speed = self.declare_parameter("teleop_speed", 1.0).value
        self.rate = self.declare_parameter("rate_hz", 20.0).value

        self.group = ReentrantCallbackGroup()
        self.pos = None          # current position (np.array, world frame)
        self.yaw = 0.0
        self.sp = None           # setpoint position
        self.sp_yaw = 0.0
        self.goal = None         # point the setpoint is moving toward
        self.flying = False
        self.landing = False
        self.cmd_vel = Twist()
        self.cmd_vel_time = 0.0

        self.create_subscription(Odometry, "/quad/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.sp_pub = self.create_publisher(PoseStamped, "/quad/setpoint", 10)
        self.enable_cli = self.create_client(SetBool, "/quad/controller/enable", callback_group=self.group)
        self.create_service(Trigger, "/quad/takeoff", self.on_takeoff, callback_group=self.group)
        self.create_service(Trigger, "/quad/land", self.on_land, callback_group=self.group)
        ActionServer(self, GoTo, "/quad/go_to", execute_callback=self.execute_go_to,
                     goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=self.group)
        self.create_timer(1.0 / self.rate, self.tick)

    # ---- inputs -------------------------------------------------------------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pos = np.array([p.x, p.y, p.z])
        self.yaw = yaw_of(msg.pose.pose.orientation)

    def on_cmd_vel(self, msg):
        self.cmd_vel = msg
        self.cmd_vel_time = time.monotonic()

    # ---- services -----------------------------------------------------------
    def set_controller(self, enabled):
        if self.enable_cli.wait_for_service(timeout_sec=2.0):
            self.enable_cli.call_async(SetBool.Request(data=enabled))
        else:
            self.get_logger().error("position controller not available")

    def on_takeoff(self, _req, res):
        if self.pos is None:
            res.success, res.message = False, "no odometry yet"
            return res
        if self.flying:
            res.success, res.message = False, "already flying"
            return res
        self.sp = self.pos.copy()
        self.sp_yaw = self.yaw
        self.goal = np.array([self.pos[0], self.pos[1], self.takeoff_height])
        self.flying, self.landing = True, False
        self.set_controller(True)
        res.success, res.message = True, f"taking off to {self.takeoff_height:.1f} m"
        self.get_logger().info(res.message)
        return res

    def on_land(self, _req, res):
        if not self.flying:
            res.success, res.message = False, "not flying"
            return res
        self.goal = np.array([self.sp[0], self.sp[1], 0.0])
        self.landing = True
        res.success, res.message = True, "landing"
        self.get_logger().info(res.message)
        return res

    # ---- action -------------------------------------------------------------
    def on_goal(self, _goal):
        return GoalResponse.ACCEPT if self.flying and not self.landing else GoalResponse.REJECT

    def execute_go_to(self, handle):
        g = handle.request
        target = np.array([g.target.x, g.target.y, g.target.z])
        tol = g.tolerance if g.tolerance > 0.0 else 0.1
        self.goal, self.sp_yaw = target, g.yaw
        feedback, result = GoTo.Feedback(), GoTo.Result()
        inside_since = None
        while rclpy.ok():
            if handle.is_cancel_requested:
                self.goal = self.sp.copy()
                handle.canceled()
                result.final_distance = float(np.linalg.norm(target - self.pos))
                return result
            dist = float(np.linalg.norm(target - self.pos))
            feedback.distance = dist
            handle.publish_feedback(feedback)
            if dist < tol:
                inside_since = inside_since or time.monotonic()
                if time.monotonic() - inside_since > 1.0:   # held inside the tolerance for 1 s
                    break
            else:
                inside_since = None
            time.sleep(0.1)
        handle.succeed()
        result.success, result.final_distance = True, dist
        return result

    # ---- setpoint generation --------------------------------------------------
    def tick(self):
        if not self.flying or self.sp is None:
            return
        dt = 1.0 / self.rate
        if time.monotonic() - self.cmd_vel_time < 0.5 and not self.landing:
            # Teleop: velocity in the body-yaw frame moves the setpoint.
            c, s = math.cos(self.sp_yaw), math.sin(self.sp_yaw)
            v = self.cmd_vel.linear
            step = self.teleop_speed * dt * np.array([c * v.x - s * v.y, s * v.x + c * v.y, v.z])
            self.sp = self.sp + step
            self.sp[2] = max(self.sp[2], 0.3)
            self.sp_yaw += self.cmd_vel.angular.z * dt
            self.goal = self.sp.copy()
        elif self.goal is not None:
            d = self.goal - self.sp
            n = np.linalg.norm(d)
            max_step = self.cruise_speed * dt
            self.sp = self.goal.copy() if n <= max_step else self.sp + d * (max_step / n)

        if self.landing and self.pos is not None and self.pos[2] < 0.12:
            self.flying, self.landing = False, False
            self.set_controller(False)      # the driver then levels, sinks and stops the motors
            self.get_logger().info("landed")
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, self.sp)
        msg.pose.orientation.z = math.sin(self.sp_yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.sp_yaw / 2.0)
        self.sp_pub.publish(msg)


def main():
    rclpy.init()
    node = Mission()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
