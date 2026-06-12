import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class MissionCommandNode(Node):
    def __init__(self):
        super().__init__("mission_command_node")
        self.declare_parameter("allow_patrol_on_warn", False)

        self.allow_patrol_on_warn = bool(
            self.get_parameter("allow_patrol_on_warn").value
        )
        self.level = "UNKNOWN"
        self.mode = "idle"
        self.last_status = None

        self.status_pub = self.create_publisher(String, "mission/mode_status", 10)
        self.patrol_control_pub = self.create_publisher(
            String, "waypoint_patrol/control", 10
        )
        self.buzzer_pub = self.create_publisher(Bool, "warning_buzzer", 10)
        self.stop_pub = self.create_publisher(Twist, "cmd_vel_raw", 10)

        self.create_subscription(String, "mission/level", self.on_level, 10)
        self.create_subscription(String, "mission/command", self.on_command, 10)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info("Mission command node ready")

    def on_level(self, msg):
        self.level = msg.data.strip().upper()
        if self.level == "DANGER" and self.mode not in ("idle", "alert"):
            self.stop_robot()
            self.pause_patrol()
            self.mode = "idle"
            self.publish_status("blocked_by_danger")

    def on_command(self, msg):
        command = msg.data.strip().lower()

        if command in ("idle", "stop", "pause"):
            self.stop_robot()
            self.pause_patrol()
            self.set_buzzer(False)
            self.mode = "idle"
            self.publish_status("stopped")
            return

        if command == "alert":
            self.stop_robot()
            self.pause_patrol()
            self.set_buzzer(True)
            self.mode = "alert"
            self.publish_status("alert_buzzer_on")
            return

        if command == "patrol":
            if self.patrol_control_pub.get_subscription_count() == 0:
                self.stop_robot()
                self.publish_status("patrol_unavailable_start_patrol_launch")
                return
            if not self.mission_allowed():
                self.stop_robot()
                self.pause_patrol()
                self.publish_status(f"patrol_blocked_level_{self.level}")
                return
            self.set_buzzer(False)
            self.publish_patrol_control("start")
            self.mode = "patrol"
            self.publish_status("patrol_started")
            return

        if command == "reset_patrol":
            if self.patrol_control_pub.get_subscription_count() == 0:
                self.publish_status("patrol_unavailable_start_patrol_launch")
                return
            self.publish_patrol_control("reset")
            self.mode = "idle"
            self.publish_status("patrol_reset")
            return

        if command in ("next", "skip", "next_waypoint"):
            if self.patrol_control_pub.get_subscription_count() == 0:
                self.publish_status("patrol_unavailable_start_patrol_launch")
                return
            self.publish_patrol_control("next")
            self.mode = "patrol"
            self.publish_status("patrol_next_waypoint")
            return

        if command in ("delivery", "guide", "evacuate"):
            self.stop_robot()
            self.pause_patrol()
            self.mode = command
            self.publish_status(f"{command}_not_ready")
            return

        self.publish_status(f"unknown_command_{command}")

    def mission_allowed(self):
        if self.level == "OK":
            return True
        return self.allow_patrol_on_warn and self.level == "WARN"

    def publish_patrol_control(self, command):
        msg = String()
        msg.data = command
        self.patrol_control_pub.publish(msg)

    def pause_patrol(self):
        self.publish_patrol_control("pause")

    def stop_robot(self):
        msg = Twist()
        self.stop_pub.publish(msg)

    def set_buzzer(self, enabled):
        msg = Bool()
        msg.data = enabled
        self.buzzer_pub.publish(msg)

    def publish_status(self, event=None):
        text = f"mode={self.mode}; level={self.level}"
        if event:
            text += f"; event={event}"
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if text != self.last_status:
            self.get_logger().info(text)
            self.last_status = text


def main():
    rclpy.init()
    node = MissionCommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop_robot()
        node.set_buzzer(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
