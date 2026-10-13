"""Action-client demo: take off, fly a 2 m square with /quad/go_to, land."""
import sys

import rclpy
from geometry_msgs.msg import Point
from quad_interfaces.action import GoTo
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


class SquareDemo(Node):
    def __init__(self):
        super().__init__("square_demo")
        self.go_to = ActionClient(self, GoTo, "/quad/go_to")
        self.takeoff = self.create_client(Trigger, "/quad/takeoff")
        self.land = self.create_client(Trigger, "/quad/land")

    def call(self, client, name):
        client.wait_for_service()
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"{name}: {future.result().message}")
        return future.result().success

    def fly_to(self, x, y, z):
        self.go_to.wait_for_server()
        goal = GoTo.Goal(target=Point(x=x, y=y, z=z), yaw=0.0, tolerance=0.15)
        sent = self.go_to.send_goal_async(
            goal, feedback_callback=lambda f: self.get_logger().info(
                f"  distance {f.feedback.distance:.2f} m", throttle_duration_sec=1.0))
        rclpy.spin_until_future_complete(self, sent)
        handle = sent.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected")
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        self.get_logger().info(f"reached ({x}, {y}, {z}), final distance {result.result().result.final_distance:.3f} m")
        return True


def main():
    rclpy.init()
    node = SquareDemo()
    ok = node.call(node.takeoff, "takeoff")
    if ok:
        node.fly_to(0.0, 0.0, 1.0)
        for x, y in [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]:
            if not node.fly_to(x, y, 1.0):
                break
        node.call(node.land, "land")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
