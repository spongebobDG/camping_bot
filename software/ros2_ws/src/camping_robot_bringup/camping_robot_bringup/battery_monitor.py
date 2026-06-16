import math
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


BATTERY_RE = re.compile(
    r"(?:battery_v|batt_v|voltage|battery)=(?P<voltage>\d+(?:\.\d+)?)"
)


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__("battery_monitor")
        self.declare_parameter("cells", 3)
        self.declare_parameter("warn_voltage", 10.8)
        self.declare_parameter("critical_voltage", 9.9)
        self.declare_parameter("stale_timeout_sec", 5.0)
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("esp32_status_topic", "/esp32/status")
        self.declare_parameter("voltage_topic", "/battery/voltage")

        self.cells = int(self.get_parameter("cells").value)
        self.warn_voltage = float(self.get_parameter("warn_voltage").value)
        self.critical_voltage = float(self.get_parameter("critical_voltage").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        self.voltage = None
        self.source = "none"
        self.last_voltage_time = None

        self.status_pub = self.create_publisher(String, "battery/status", 10)

        self.create_subscription(
            String,
            str(self.get_parameter("esp32_status_topic").value),
            self.on_esp32_status,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("voltage_topic").value),
            self.on_voltage,
            10,
        )
        self.create_timer(1.0 / publish_hz, self.report)
        self.get_logger().info("Battery monitor started")

    def on_esp32_status(self, msg):
        match = BATTERY_RE.search(msg.data)
        if not match:
            return
        self.update_voltage(float(match.group("voltage")), "esp32_status")

    def on_voltage(self, msg):
        self.update_voltage(float(msg.data), "voltage_topic")

    def update_voltage(self, voltage, source):
        if not math.isfinite(voltage) or voltage <= 0.0:
            return
        self.voltage = voltage
        self.source = source
        self.last_voltage_time = self.get_clock().now()

    def report(self):
        now = self.get_clock().now()
        age = self.age(now)
        if self.voltage is None:
            text = "UNKNOWN voltage=None cell=None source=none age=infs"
        elif age > self.stale_timeout:
            text = (
                f"STALE voltage={self.voltage:.2f}V "
                f"cell={self.cell_voltage():.2f}V source={self.source} age={age:.1f}s"
            )
        elif self.voltage <= self.critical_voltage:
            text = (
                f"CRITICAL voltage={self.voltage:.2f}V "
                f"cell={self.cell_voltage():.2f}V source={self.source} age={age:.1f}s"
            )
        elif self.voltage <= self.warn_voltage:
            text = (
                f"LOW voltage={self.voltage:.2f}V "
                f"cell={self.cell_voltage():.2f}V source={self.source} age={age:.1f}s"
            )
        else:
            text = (
                f"OK voltage={self.voltage:.2f}V "
                f"cell={self.cell_voltage():.2f}V source={self.source} age={age:.1f}s"
            )

        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def cell_voltage(self):
        if self.voltage is None or self.cells <= 0:
            return math.nan
        return self.voltage / self.cells

    def age(self, now):
        if self.last_voltage_time is None:
            return math.inf
        return (now - self.last_voltage_time).nanoseconds / 1e9


def main():
    rclpy.init()
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
