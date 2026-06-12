import math
import socket

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class AckermannUdpBridge(Node):
    def __init__(self):
        super().__init__("ackermann_udp_bridge")
        self.declare_parameter("esp32_ip", "192.168.0.15")
        self.declare_parameter("command_port", 12347)
        self.declare_parameter("wheel_base", 0.20)
        self.declare_parameter("max_linear_mps", 0.40)
        self.declare_parameter("min_drive_linear_mps", 0.0)
        self.declare_parameter("max_steering_rad", 0.60)
        self.declare_parameter("command_timeout_sec", 0.5)

        self.esp32_ip = self.get_parameter("esp32_ip").value
        self.command_port = int(self.get_parameter("command_port").value)
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.max_linear = float(self.get_parameter("max_linear_mps").value)
        self.min_drive_linear = float(self.get_parameter("min_drive_linear_mps").value)
        self.max_steering = float(self.get_parameter("max_steering_rad").value)
        self.timeout_sec = float(self.get_parameter("command_timeout_sec").value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_cmd_time = self.get_clock().now()
        self.last_linear = 0.0
        self.last_steering = 0.0
        self.last_logged_linear = None
        self.last_logged_steering = None

        self.executed_pub = self.create_publisher(Twist, "cmd_vel_executed", 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(
            f"Sending Ackermann UDP commands to {self.esp32_ip}:{self.command_port}"
        )

    def on_cmd_vel(self, msg: Twist):
        linear = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        if 1e-4 < abs(linear) < self.min_drive_linear:
            linear = math.copysign(self.min_drive_linear, linear)
        if abs(linear) < 1e-4:
            steering = msg.angular.z
        else:
            steering = math.atan((msg.angular.z * self.wheel_base) / linear)
        steering = max(-self.max_steering, min(self.max_steering, steering))

        self.send_command(linear, steering)
        self.publish_executed(linear, steering)
        self.last_cmd_time = self.get_clock().now()
        self.last_linear = linear
        self.last_steering = steering

    def watchdog(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.timeout_sec and (
            abs(self.last_linear) > 1e-4 or abs(self.last_steering) > 1e-4
        ):
            self.send_command(0.0, 0.0)
            self.publish_executed(0.0, 0.0)
            self.last_linear = 0.0
            self.last_steering = 0.0

    def send_command(self, linear: float, steering: float):
        payload = f"{linear:.4f},{steering:.4f}".encode("ascii")
        try:
            self.sock.sendto(payload, (self.esp32_ip, self.command_port))
        except OSError as exc:
            self.get_logger().error(
                f"Failed to send UDP cmd to {self.esp32_ip}:{self.command_port}: {exc}",
                throttle_duration_sec=1.0,
            )
            self.publish_executed(0.0, 0.0)
            return False
        if (
            self.last_logged_linear is None
            or abs(linear - self.last_logged_linear) > 0.01
            or abs(steering - self.last_logged_steering) > 0.05
        ):
            self.get_logger().info(
                f"UDP cmd linear={linear:.3f} steering={steering:.3f}"
            )
            self.last_logged_linear = linear
            self.last_logged_steering = steering
        return True

    def publish_executed(self, linear: float, steering: float):
        msg = Twist()
        msg.linear.x = linear
        if abs(linear) > 1e-4:
            msg.angular.z = linear * math.tan(steering) / self.wheel_base
        else:
            msg.angular.z = 0.0
        self.executed_pub.publish(msg)


def main():
    rclpy.init()
    node = AckermannUdpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
