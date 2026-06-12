import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class WarningBuzzerUdpBridge(Node):
    def __init__(self):
        super().__init__("warning_buzzer_udp_bridge")
        self.declare_parameter("esp32_ip", "192.168.0.10")
        self.declare_parameter("command_port", 12347)
        self.declare_parameter("repeat_hz", 2.0)

        self.esp32_ip = self.get_parameter("esp32_ip").value
        self.command_port = int(self.get_parameter("command_port").value)
        repeat_hz = float(self.get_parameter("repeat_hz").value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.buzzer_on = False
        self.last_sent_state = None

        self.create_subscription(Bool, "warning_buzzer", self.on_buzzer, 10)
        self.create_timer(1.0 / repeat_hz, self.repeat_state)
        self.get_logger().info(
            f"Sending buzzer UDP commands to {self.esp32_ip}:{self.command_port}"
        )
        self.send_buzzer(False)

    def on_buzzer(self, msg: Bool):
        self.buzzer_on = bool(msg.data)
        self.send_buzzer(self.buzzer_on)

    def repeat_state(self):
        self.send_buzzer(self.buzzer_on, repeat=True)

    def send_buzzer(self, enabled: bool, repeat: bool = False):
        if repeat and self.last_sent_state == enabled and not enabled:
            return

        payload = f"BUZZER,{1 if enabled else 0}".encode("ascii")
        try:
            self.sock.sendto(payload, (self.esp32_ip, self.command_port))
        except OSError as exc:
            self.get_logger().error(
                f"Failed to send buzzer UDP cmd to {self.esp32_ip}:{self.command_port}: {exc}",
                throttle_duration_sec=1.0,
            )
            return

        if self.last_sent_state != enabled:
            state = "ON" if enabled else "OFF"
            self.get_logger().info(f"Buzzer {state}")
            self.last_sent_state = enabled


def main():
    rclpy.init()
    node = WarningBuzzerUdpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.send_buzzer(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
