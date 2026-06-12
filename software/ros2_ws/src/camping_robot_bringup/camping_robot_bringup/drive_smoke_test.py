import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DriveSmokeTest(Node):
    def __init__(self):
        super().__init__("drive_smoke_test")
        self.declare_parameter("linear_mps", 0.08)
        self.declare_parameter("angular_radps", 0.0)
        self.declare_parameter("run_seconds", 2.0)
        self.declare_parameter("stop_seconds", 1.0)

        self.linear_mps = float(self.get_parameter("linear_mps").value)
        self.angular_radps = float(self.get_parameter("angular_radps").value)
        self.run_seconds = float(self.get_parameter("run_seconds").value)
        self.stop_seconds = float(self.get_parameter("stop_seconds").value)
        self.pub = self.create_publisher(Twist, "cmd_vel_raw", 10)

    def run(self):
        self.get_logger().warn("Lift the wheels before running this test.")
        time.sleep(1.0)

        move = Twist()
        move.linear.x = self.linear_mps
        move.angular.z = self.angular_radps
        end_time = time.monotonic() + self.run_seconds
        self.get_logger().info(
            f"Publishing cmd_vel_raw linear={move.linear.x:.3f}, angular={move.angular.z:.3f}"
        )
        while time.monotonic() < end_time and rclpy.ok():
            self.pub.publish(move)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)

        stop = Twist()
        end_time = time.monotonic() + self.stop_seconds
        self.get_logger().info("Publishing stop command")
        while time.monotonic() < end_time and rclpy.ok():
            self.pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)


def main():
    rclpy.init()
    node = DriveSmokeTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
