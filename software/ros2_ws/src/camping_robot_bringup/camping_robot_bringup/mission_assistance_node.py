import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


OBSTACLE_RE = re.compile(r"OBSTACLE_(?:CLOSE|CRITICAL)")


class MissionAssistanceNode(Node):
    def __init__(self):
        super().__init__("mission_assistance_node")
        self.declare_parameter("blocked_after_sec", 3.0)
        self.declare_parameter("clear_after_sec", 1.5)
        self.declare_parameter("publish_hz", 1.0)

        self.blocked_after_sec = float(self.get_parameter("blocked_after_sec").value)
        self.clear_after_sec = float(self.get_parameter("clear_after_sec").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        self.hazard = "UNKNOWN"
        self.mode_status = "mode=idle"
        self.task_status = "task=idle"
        self.obstacle_since = None
        self.clear_since = None
        self.request_active = False
        self.last_request = "none"
        self.last_task_command = "patrol"

        self.request_pub = self.create_publisher(String, "mission/assistance_request", 10)
        self.command_pub = self.create_publisher(String, "mission/command", 10)

        self.create_subscription(String, "camping_robot/hazard", self.on_hazard, 10)
        self.create_subscription(String, "mission/mode_status", self.on_mode, 10)
        self.create_subscription(String, "mission/task_status", self.on_task, 10)
        self.create_subscription(String, "mission/decision", self.on_decision, 10)
        self.create_subscription(String, "mission/command", self.on_mission_command, 10)
        self.create_timer(1.0 / publish_hz, self.tick)
        self.get_logger().info("Mission assistance node ready")

    def on_hazard(self, msg):
        self.hazard = msg.data

    def on_mode(self, msg):
        self.mode_status = msg.data

    def on_task(self, msg):
        self.task_status = msg.data

    def on_mission_command(self, msg):
        command = msg.data.strip().lower()
        if command in ("patrol", "delivery", "guide", "evacuate", "return_home"):
            self.last_task_command = command
        if command in ("stop", "idle"):
            self.request_active = False
            self.publish_request("none")

    def on_decision(self, msg):
        decision = msg.data.strip().lower()
        if decision in ("wait", "ack"):
            self.publish_request("waiting_for_clear")
            return
        if decision == "retry":
            self.request_active = False
            self.publish_command(self.last_task_command)
            self.publish_request("retrying")
            return
        if decision in ("next", "skip"):
            self.request_active = False
            self.publish_command("next")
            self.publish_request("skipping_waypoint")
            return
        if decision in ("stop", "cancel"):
            self.request_active = False
            self.publish_command("stop")
            self.publish_request("stopping")
            return
        if decision == "alert":
            self.publish_command("alert")
            self.publish_request("alerting")
            return
        self.publish_request(f"unknown_decision:{decision}")

    def tick(self):
        now = self.get_clock().now()
        if not self.mission_active():
            self.reset_obstacle_state()
            return

        if OBSTACLE_RE.search(self.hazard):
            self.clear_since = None
            if self.obstacle_since is None:
                self.obstacle_since = now
                return
            blocked_for = (now - self.obstacle_since).nanoseconds / 1e9
            if blocked_for >= self.blocked_after_sec:
                self.request_active = True
                self.publish_request(
                    "blocked_obstacle;"
                    " choices=wait,retry,next,stop,alert;"
                    f" hazard={self.hazard}"
                )
            return

        self.obstacle_since = None
        if self.request_active:
            if self.clear_since is None:
                self.clear_since = now
                return
            clear_for = (now - self.clear_since).nanoseconds / 1e9
            if clear_for >= self.clear_after_sec:
                self.request_active = False
                self.publish_request("cleared")

    def mission_active(self):
        inactive_markers = (
            "mode=idle",
            "mode=alert",
            "task=idle",
            "complete",
            "cancelled",
            "stopped",
        )
        mode_active = not any(marker in self.mode_status for marker in inactive_markers[:2])
        task_active = not any(marker in self.task_status for marker in inactive_markers[2:])
        return mode_active or task_active

    def reset_obstacle_state(self):
        self.obstacle_since = None
        self.clear_since = None
        if self.request_active:
            self.request_active = False
            self.publish_request("none")

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def publish_request(self, text):
        if text == self.last_request:
            return
        self.last_request = text
        msg = String()
        msg.data = text
        self.request_pub.publish(msg)
        self.get_logger().info(text)


def main():
    rclpy.init()
    node = MissionAssistanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
