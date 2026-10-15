"""Mission layer: turns high-level requests into the reference the controllers follow.

Interfaces
- services  /quad/takeoff, /quad/land            (std_srvs/Trigger)
- action    /quad/go_to                          (quad_interfaces/GoTo)
- action    /quad/follow_trajectory              (quad_interfaces/FollowTrajectory)
- topic in  /cmd_vel (geometry_msgs/Twist)       keyboard teleop, body-yaw frame
- topic in  /quad/odom
- topic out /quad/reference (ReferenceTrajectory, 50 Hz)  read by either controller
- topic out /quad/setpoint (PoseStamped, 50 Hz)  current reference point, for logging
- client    /quad/controller/enable              (std_srvs/SetBool)

For hover, go_to and teleop the reference is a single point that moves at a bounded
speed, so the controller never sees a step of several metres. During a trajectory it is
the next 1.5 s of the path, with velocity and acceleration, on simulation time.
"""
import math
import time

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from quad_interfaces.action import FollowTrajectory, GoTo
from quad_interfaces.msg import ReferenceTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool, Trigger

from quad_mission.trajectory import SHAPES, Trajectory

HORIZON = 1.5      # [s] of trajectory sent ahead (the NMPC looks 1 s ahead)
SAMPLE_DT = 0.02   # [s] between reference samples


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Mission(Node):
    def __init__(self):
        super().__init__("mission")
        self.takeoff_height = self.declare_parameter("takeoff_height", 1.0).value
        self.cruise_speed = self.declare_parameter("cruise_speed", 1.0).value
        self.teleop_speed = self.declare_parameter("teleop_speed", 1.0).value
        self.rate = self.declare_parameter("rate_hz", 50.0).value

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
        self.ground_z = 0.0      # resting height, read at takeoff
        self.still_since = None
        self.vz = 0.0
        self.traj = None         # Trajectory being flown, and its start time (simulation clock)
        self.traj_t0 = 0.0

        self.create_subscription(Odometry, "/quad/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.sp_pub = self.create_publisher(PoseStamped, "/quad/setpoint", 10)
        self.ref_pub = self.create_publisher(ReferenceTrajectory, "/quad/reference", 10)
        self.enable_cli = self.create_client(SetBool, "/quad/controller/enable", callback_group=self.group)
        self.create_service(Trigger, "/quad/takeoff", self.on_takeoff, callback_group=self.group)
        self.create_service(Trigger, "/quad/land", self.on_land, callback_group=self.group)
        ActionServer(self, GoTo, "/quad/go_to", execute_callback=self.execute_go_to,
                     goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=self.group)
        ActionServer(self, FollowTrajectory, "/quad/follow_trajectory", execute_callback=self.execute_trajectory,
                     goal_callback=self.on_trajectory_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=self.group)
        self.create_timer(1.0 / self.rate, self.tick)

    # ---- inputs -------------------------------------------------------------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pos = np.array([p.x, p.y, p.z])
        self.yaw = yaw_of(msg.pose.pose.orientation)
        q, v = msg.pose.pose.orientation, msg.twist.twist.linear   # twist is in the body frame
        self.vz = 2 * (q.x * q.z - q.w * q.y) * v.x + 2 * (q.y * q.z + q.w * q.x) * v.y             + (1 - 2 * (q.x * q.x + q.y * q.y)) * v.z

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
        self.ground_z = float(self.pos[2])
        self.sp = self.pos.copy()
        self.sp_yaw = self.yaw
        self.goal = np.array([self.pos[0], self.pos[1], self.ground_z + self.takeoff_height])
        self.flying, self.landing = True, False
        self.set_controller(True)
        res.success, res.message = True, f"taking off to {self.takeoff_height:.1f} m"
        self.get_logger().info(res.message)
        return res

    def on_land(self, _req, res):
        if not self.flying:
            res.success, res.message = False, "not flying"
            return res
        self.goal = np.array([self.sp[0], self.sp[1], self.ground_z - 0.05])  # press slightly into the ground
        self.landing = True
        self.still_since = None
        res.success, res.message = True, "landing"
        self.get_logger().info(res.message)
        return res

    # ---- actions ------------------------------------------------------------
    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_goal(self, _goal):
        ok = self.flying and not self.landing and self.traj is None
        return GoalResponse.ACCEPT if ok else GoalResponse.REJECT

    def on_trajectory_goal(self, goal):
        ok = (self.flying and not self.landing and self.traj is None and goal.shape in SHAPES
              and goal.size > 0.0 and goal.max_speed > 0.0)
        return GoalResponse.ACCEPT if ok else GoalResponse.REJECT

    def execute_trajectory(self, handle):
        g = handle.request
        start = np.array([g.start.x, g.start.y, g.start.z])
        traj = Trajectory(g.shape, g.size, g.max_speed, g.laps, start, yaw=self.sp_yaw)
        result = FollowTrajectory.Result()
        # 1. fly to the start point and settle there: within 10 cm for 1 s (gusts may keep the
        #    drone moving), and in any case go 15 s after the setpoint has reached the start
        self.goal = start
        inside_since, arrived = None, None
        while rclpy.ok():
            if handle.is_cancel_requested:
                self.goal = self.sp.copy()
                handle.canceled()
                return result
            now = self.sim_time()
            if np.linalg.norm(start - self.sp) < 1e-6:
                arrived = arrived or now
            if np.linalg.norm(start - self.pos) < 0.10:
                inside_since = inside_since or now
                if now - inside_since > 1.0:
                    break
            else:
                inside_since = None
            if arrived and now - arrived > 15.0:
                self.get_logger().warn("start point not held within 10 cm: starting anyway")
                break
            time.sleep(0.05)
        # 2. fly the trajectory; errors are taken against the reference at the same time
        self.traj_t0 = self.sim_time()
        self.traj = traj
        t1, t2 = traj.top_speed_window()
        errors, feedback = [], FollowTrajectory.Feedback()
        while rclpy.ok():
            t = self.sim_time() - self.traj_t0
            if handle.is_cancel_requested or t > traj.duration:
                break
            err = float(np.linalg.norm(traj.sample(t)[0] - self.pos))
            if t1 <= t <= t2:
                errors.append(err)
            feedback.progress, feedback.error = min(t / traj.duration, 1.0), err
            handle.publish_feedback(feedback)
            time.sleep(0.02)
        end = traj.sample(traj.duration)[0]
        self.traj = None
        self.sp, self.goal = end.copy(), end.copy()
        if handle.is_cancel_requested:
            handle.canceled()
            return result
        handle.succeed()
        result.success = True
        result.start_time = rclpy.time.Time(nanoseconds=int(self.traj_t0 * 1e9)).to_msg()
        result.top_speed_start, result.top_speed_end = t1, t2
        if errors:
            result.rms_error = float(np.sqrt(np.mean(np.square(errors))))
            result.max_error = float(np.max(errors))
        return result

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
        if self.traj is not None:
            self.publish_trajectory()
            return
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
            max_step = (min(self.cruise_speed, 0.5) if self.landing else self.cruise_speed) * dt
            self.sp = self.goal.copy() if n <= max_step else self.sp + d * (max_step / n)

        if self.landing and self.pos is not None:
            on_ground = self.pos[2] < self.ground_z + 0.03 and abs(self.vz) < 0.1
            self.still_since = (self.still_since or time.monotonic()) if on_ground else None
            if self.still_since and time.monotonic() - self.still_since > 0.5:
                self.flying, self.landing = False, False
                self.set_controller(False)      # the driver then stops the motors (on the ground)
                self.get_logger().info("landed")
                return

        stamp = self.get_clock().now().to_msg()
        self.publish_setpoint(stamp)
        ref = ReferenceTrajectory()
        ref.header.stamp, ref.header.frame_id = stamp, "odom"
        ref.dt = SAMPLE_DT
        ref.position = [Point(x=float(self.sp[0]), y=float(self.sp[1]), z=float(self.sp[2]))]
        ref.velocity, ref.acceleration, ref.yaw = [Vector3()], [Vector3()], [float(self.sp_yaw)]
        self.ref_pub.publish(ref)

    def publish_setpoint(self, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, self.sp)
        msg.pose.orientation.z = math.sin(self.sp_yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.sp_yaw / 2.0)
        self.sp_pub.publish(msg)

    def publish_trajectory(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9 - self.traj_t0
        ref = ReferenceTrajectory()
        ref.header.stamp, ref.header.frame_id = now.to_msg(), "odom"
        ref.dt = SAMPLE_DT
        for k in range(int(HORIZON / SAMPLE_DT) + 1):
            p, v, a = self.traj.sample(t + k * SAMPLE_DT)
            ref.position.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            ref.velocity.append(Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2])))
            ref.acceleration.append(Vector3(x=float(a[0]), y=float(a[1]), z=float(a[2])))
            ref.yaw.append(float(self.traj.yaw))
        self.sp = np.array([ref.position[0].x, ref.position[0].y, ref.position[0].z])
        self.ref_pub.publish(ref)
        self.publish_setpoint(ref.header.stamp)


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
