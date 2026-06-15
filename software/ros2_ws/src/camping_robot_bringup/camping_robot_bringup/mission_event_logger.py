import csv
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class MissionEventLogger(Node):
    def __init__(self):
        super().__init__("mission_event_logger")
        self.declare_parameter("log_dir", "~/.ros/camping_bot_logs")
        self.declare_parameter("flush_period_sec", 1.0)
        self.declare_parameter("cmd_min_interval_sec", 0.5)
        self.declare_parameter("cmd_change_epsilon", 0.01)

        self.cmd_min_interval = float(
            self.get_parameter("cmd_min_interval_sec").value
        )
        self.cmd_change_epsilon = float(
            self.get_parameter("cmd_change_epsilon").value
        )
        flush_period = float(self.get_parameter("flush_period_sec").value)

        log_dir = Path(str(self.get_parameter("log_dir").value)).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = log_dir / f"mission_events_{stamp}.csv"

        self.file = self.log_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["wall_time", "ros_time_sec", "source", "value"])

        self.last_string_values = {}
        self.last_cmd = None
        self.last_cmd_time = None

        self.create_subscription(String, "mission/status", self.on_string("mission/status"), 10)
        self.create_subscription(String, "mission/level", self.on_string("mission/level"), 10)
        self.create_subscription(String, "mission/command", self.on_string("mission/command"), 10)
        self.create_subscription(String, "mission/task_status", self.on_string("mission/task_status"), 10)
        self.create_subscription(String, "mission/assistance_request", self.on_string("mission/assistance_request"), 10)
        self.create_subscription(String, "mission/elevator_status", self.on_string("mission/elevator_status"), 10)
        self.create_subscription(String, "camping_robot/hazard", self.on_string("camping_robot/hazard"), 10)
        self.create_subscription(String, "esp32/status", self.on_string("esp32/status"), 10)
        self.create_subscription(String, "camera/status", self.on_string("camera/status"), 10)
        self.create_subscription(Twist, "cmd_vel_executed", self.on_cmd, 10)

        self.create_timer(flush_period, self.flush)
        self.get_logger().info(f"Mission event log: {self.log_path}")

    def on_string(self, source):
        def callback(msg):
            if self.last_string_values.get(source) == msg.data:
                return
            self.last_string_values[source] = msg.data
            self.write_event(source, msg.data)

        return callback

    def on_cmd(self, msg):
        now = self.get_clock().now()
        cmd = (float(msg.linear.x), float(msg.angular.z))
        if self.last_cmd is not None:
            age = (now - self.last_cmd_time).nanoseconds / 1e9
            linear_delta = abs(cmd[0] - self.last_cmd[0])
            angular_delta = abs(cmd[1] - self.last_cmd[1])
            if (
                age < self.cmd_min_interval
                and linear_delta < self.cmd_change_epsilon
                and angular_delta < self.cmd_change_epsilon
            ):
                return

        self.last_cmd = cmd
        self.last_cmd_time = now
        self.write_event("cmd_vel_executed", f"linear={cmd[0]:.3f}; angular={cmd[1]:.3f}")

    def write_event(self, source, value):
        now = self.get_clock().now()
        ros_time = now.nanoseconds / 1e9
        wall_time = datetime.now().isoformat(timespec="milliseconds")
        self.writer.writerow([wall_time, f"{ros_time:.6f}", source, value])

    def flush(self):
        self.file.flush()

    def destroy_node(self):
        try:
            self.flush()
            self.file.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = MissionEventLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
