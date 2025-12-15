import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from time import time
import math


class SimpleTurtlesimOdometry(Node):
    def __init__(self):
        super().__init__("simple_turtlesim_odometry")

        self.vel_sub_ = self.create_subscription(Pose, "turtle1/pose", self.velCallback, 10)
        self.get_logger().info("Simple Turtlesim Node is Initialized!")
        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_time = time()

    def velCallback(self, msg: Pose):
        current_time = time()
        dt = current_time - self.last_time
        self.last_time = current_time

        v = msg.linear_velocity
        w = msg.angular_velocity

        # ---- ODOMETRY KINEMATICS ----
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        theta_heading = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # ----------------------------

        self.get_logger().info(
            f"Estimated Pose -> x: {self.x:.2f}, y: {self.y:.2f}, theta: {self.theta:.2f}, theta_heading: {theta_heading:.2f}"
        )


def main():
    rclpy.init()
    simple_turtlesim_odometry = SimpleTurtlesimOdometry()
    rclpy.spin(simple_turtlesim_odometry)
    simple_turtlesim_odometry.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()