import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


RSSI_RE = re.compile(r"rssi=(?P<rssi>-?\d+)")
UPTIME_RE = re.compile(r"uptime_ms=(?P<uptime>\d+)")


class MissionSupervisor(Node):
    def __init__(self):
        super().__init__("mission_supervisor")
        self.declare_parameter("camera_timeout_sec", 5.0)
        self.declare_parameter("esp32_timeout_sec", 3.0)
        self.declare_parameter("hazard_timeout_sec", 3.0)
        self.declare_parameter("weak_rssi", -75)
        self.declare_parameter("esp32_reboot_warn_sec", 30.0)
        self.declare_parameter("publish_hz", 1.0)

        self.camera_timeout = float(self.get_parameter("camera_timeout_sec").value)
        self.esp32_timeout = float(self.get_parameter("esp32_timeout_sec").value)
        self.hazard_timeout = float(self.get_parameter("hazard_timeout_sec").value)
        self.weak_rssi = int(self.get_parameter("weak_rssi").value)
        self.esp32_reboot_warn_sec = float(
            self.get_parameter("esp32_reboot_warn_sec").value
        )
        publish_hz = float(self.get_parameter("publish_hz").value)

        self.camera_online = None
        self.camera_time = None
        self.esp32_status = ""
        self.esp32_time = None
        self.esp32_rssi = None
        self.esp32_uptime = None
        self.previous_esp32_uptime = None
        self.esp32_reboot_time = None
        self.hazard = "UNKNOWN"
        self.hazard_time = None
        self.simple_goal_status = "idle"
        self.patrol_status = "idle"
        self.last_summary = None

        self.status_pub = self.create_publisher(String, "mission/status", 10)
        self.level_pub = self.create_publisher(String, "mission/level", 10)

        self.create_subscription(Bool, "camera/online", self.on_camera_online, 10)
        self.create_subscription(String, "esp32/status", self.on_esp32_status, 10)
        self.create_subscription(String, "camping_robot/hazard", self.on_hazard, 10)
        self.create_subscription(String, "simple_goal/status", self.on_simple_goal, 10)
        self.create_subscription(String, "waypoint_patrol/status", self.on_patrol, 10)
        self.create_timer(1.0 / publish_hz, self.report)
        self.get_logger().info("Mission supervisor started")

    def on_camera_online(self, msg):
        self.camera_online = bool(msg.data)
        self.camera_time = self.get_clock().now()

    def on_esp32_status(self, msg):
        self.esp32_status = msg.data
        self.esp32_time = self.get_clock().now()

        rssi_match = RSSI_RE.search(msg.data)
        self.esp32_rssi = int(rssi_match.group("rssi")) if rssi_match else None

        uptime_match = UPTIME_RE.search(msg.data)
        self.esp32_uptime = int(uptime_match.group("uptime")) if uptime_match else None
        if (
            self.previous_esp32_uptime is not None
            and self.esp32_uptime is not None
            and self.esp32_uptime < self.previous_esp32_uptime
        ):
            self.esp32_reboot_time = self.get_clock().now()
        if self.esp32_uptime is not None:
            self.previous_esp32_uptime = self.esp32_uptime

    def on_hazard(self, msg):
        self.hazard = msg.data
        self.hazard_time = self.get_clock().now()

    def on_simple_goal(self, msg):
        self.simple_goal_status = msg.data

    def on_patrol(self, msg):
        self.patrol_status = msg.data

    def report(self):
        now = self.get_clock().now()
        issues = []
        level = "OK"

        camera_age = self.age(now, self.camera_time)
        if camera_age > self.camera_timeout:
            issues.append("CAMERA_STALE")
            level = "WARN"
        elif self.camera_online is False:
            issues.append("CAMERA_OFFLINE")
            level = "WARN"

        esp32_age = self.age(now, self.esp32_time)
        if esp32_age > self.esp32_timeout:
            issues.append("ESP32_STALE")
            level = "WARN"
        if self.esp32_rssi is not None and self.esp32_rssi <= self.weak_rssi:
            issues.append(f"ESP32_WEAK_RSSI:{self.esp32_rssi}")
            level = "WARN"
        reboot_age = self.age(now, self.esp32_reboot_time)
        if reboot_age <= self.esp32_reboot_warn_sec:
            issues.append("ESP32_REBOOT_SEEN")
            level = "WARN"

        hazard_age = self.age(now, self.hazard_time)
        if hazard_age > self.hazard_timeout:
            issues.append("HAZARD_STALE")
            level = "WARN"
        elif self.hazard != "OK":
            issues.append(self.hazard)
            if "CRITICAL" in self.hazard or "SCAN_STALE" in self.hazard:
                level = "DANGER"
            else:
                level = "WARN"

        if not issues:
            issues.append("all_systems_nominal")

        summary = (
            f"level={level}; "
            f"hazard={self.hazard}; "
            f"camera={self.camera_online}; "
            f"esp32_rssi={self.esp32_rssi}; "
            f"goal={self.simple_goal_status}; "
            f"patrol={self.patrol_status}; "
            f"issues={','.join(issues)}"
        )

        status_msg = String()
        status_msg.data = summary
        self.status_pub.publish(status_msg)

        level_msg = String()
        level_msg.data = level
        self.level_pub.publish(level_msg)

        if summary != self.last_summary:
            if level == "DANGER":
                self.get_logger().error(summary)
            elif level == "WARN":
                self.get_logger().warn(summary)
            else:
                self.get_logger().info(summary)
            self.last_summary = summary

    @staticmethod
    def age(now, last_time):
        if last_time is None:
            return 9999.0
        return (now - last_time).nanoseconds / 1e9


def main():
    rclpy.init()
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
