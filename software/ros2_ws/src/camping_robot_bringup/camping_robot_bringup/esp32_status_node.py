import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Esp32StatusNode(Node):
    def __init__(self):
        super().__init__("esp32_status_node")
        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("status_port", 12349)
        bind_ip = self.get_parameter("bind_ip").value
        status_port = int(self.get_parameter("status_port").value)

        self.pub = self.create_publisher(String, "esp32/status", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((bind_ip, status_port))
        self.last_status_time = None

        self.create_timer(0.05, self.poll_socket)
        self.create_timer(1.0, self.report_watchdog)
        self.get_logger().info(f"Listening for ESP32 status UDP on {bind_ip}:{status_port}")

    def poll_socket(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(512)
            except BlockingIOError:
                return

            text = data.decode("utf-8", errors="replace")
            self.last_status_time = self.get_clock().now()
            msg = String()
            msg.data = text
            self.pub.publish(msg)
            self.get_logger().info(f"ESP32 status from {addr[0]}: {text}", throttle_duration_sec=1.0)

    def report_watchdog(self):
        if self.last_status_time is None:
            self.get_logger().warn("No ESP32 status received yet")
            return
        age = (self.get_clock().now() - self.last_status_time).nanoseconds / 1e9
        if age > 2.5:
            self.get_logger().warn(f"ESP32 status stale: {age:.1f}s since last packet")


def main():
    rclpy.init()
    node = Esp32StatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
