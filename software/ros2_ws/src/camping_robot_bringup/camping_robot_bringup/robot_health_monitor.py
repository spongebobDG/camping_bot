from collections import deque
import math
import re

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String


STATUS_RE = re.compile(
    r"uptime_ms=(?P<uptime>\d+).*?"
    r"rssi=(?P<rssi>-?\d+).*?"
    r"cmd_count=(?P<cmd_count>\d+).*?"
    r"last_cmd_age_ms=(?P<last_cmd_age>\d+).*?"
    r"free_heap=(?P<free_heap>\d+)"
)


class TopicRate:
    def __init__(self, maxlen=80):
        self.times = deque(maxlen=maxlen)
        self.last_time = None

    def tick(self, now):
        self.last_time = now
        self.times.append(now.nanoseconds / 1e9)

    def age(self, now):
        if self.last_time is None:
            return math.inf
        return (now - self.last_time).nanoseconds / 1e9

    def hz(self):
        if len(self.times) < 2:
            return 0.0
        elapsed = self.times[-1] - self.times[0]
        if elapsed <= 0.0:
            return 0.0
        return (len(self.times) - 1) / elapsed


class RobotHealthMonitor(Node):
    def __init__(self):
        super().__init__("robot_health_monitor")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_topic", "/cmd_vel_executed")
        self.declare_parameter("esp32_status_topic", "/esp32/status")
        self.declare_parameter("warn_after_sec", 1.0)

        self.warn_after_sec = float(self.get_parameter("warn_after_sec").value)
        self.imu_rate = TopicRate()
        self.scan_rate = TopicRate()
        self.cmd_rate = TopicRate()
        self.status_rate = TopicRate()
        self.last_cmd = Twist()
        self.last_status_text = ""
        self.last_status = {}
        self.previous_uptime = None

        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self.on_imu,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_topic").value,
            self.on_cmd,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("esp32_status_topic").value,
            self.on_status,
            10,
        )
        self.create_timer(1.0, self.report)
        self.get_logger().info("Robot health monitor started")

    def on_imu(self, _msg):
        self.imu_rate.tick(self.get_clock().now())

    def on_scan(self, _msg):
        self.scan_rate.tick(self.get_clock().now())

    def on_cmd(self, msg):
        self.last_cmd = msg
        self.cmd_rate.tick(self.get_clock().now())

    def on_status(self, msg):
        now = self.get_clock().now()
        self.status_rate.tick(now)
        self.last_status_text = msg.data
        match = STATUS_RE.search(msg.data)
        if not match:
            return

        parsed = {key: int(value) for key, value in match.groupdict().items()}
        uptime = parsed["uptime"]
        if self.previous_uptime is not None and uptime < self.previous_uptime:
            self.get_logger().warn("ESP32 uptime reset detected; possible reboot or power dip")
        self.previous_uptime = uptime
        self.last_status = parsed

    def state_label(self, age):
        if math.isinf(age):
            return "missing"
        if age > self.warn_after_sec:
            return "stale"
        return "ok"

    def report(self):
        now = self.get_clock().now()
        imu_age = self.imu_rate.age(now)
        scan_age = self.scan_rate.age(now)
        cmd_age = self.cmd_rate.age(now)
        status_age = self.status_rate.age(now)

        parts = [
            f"imu={self.state_label(imu_age)} {self.imu_rate.hz():.1f}Hz age={imu_age:.1f}s",
            f"scan={self.state_label(scan_age)} {self.scan_rate.hz():.1f}Hz age={scan_age:.1f}s",
            f"cmd={self.state_label(cmd_age)} {self.cmd_rate.hz():.1f}Hz age={cmd_age:.1f}s",
            (
                "last_cmd="
                f"linear:{self.last_cmd.linear.x:.2f}, angular:{self.last_cmd.angular.z:.2f}"
            ),
        ]

        if self.last_status:
            parts.append(
                "esp32="
                f"{self.state_label(status_age)} "
                f"rssi:{self.last_status['rssi']} "
                f"uptime:{self.last_status['uptime'] / 1000.0:.0f}s "
                f"cmd_count:{self.last_status['cmd_count']} "
                f"last_cmd_age:{self.last_status['last_cmd_age']}ms "
                f"heap:{self.last_status['free_heap']}"
            )
        else:
            parts.append(f"esp32={self.state_label(status_age)}")

        line = " | ".join(parts)
        if any(self.state_label(age) != "ok" for age in [imu_age, scan_age, status_age]):
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)


def main():
    rclpy.init()
    node = RobotHealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
