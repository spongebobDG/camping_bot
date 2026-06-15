import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class MissionTaskManager(Node):
    def __init__(self):
        super().__init__("mission_task_manager")
        self.declare_parameter("mission_locations_file", "")
        self.declare_parameter("navigation_mode", "auto")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("allow_on_warn", False)

        self.navigation_mode = self.get_parameter("navigation_mode").value
        self.allow_on_warn = bool(self.get_parameter("allow_on_warn").value)
        self.locations = self.load_locations(
            self.get_parameter("mission_locations_file").value
        )
        self.level = "UNKNOWN"
        self.active_task = "idle"
        self.last_status = None
        self.nav2_goal_handle = None

        goal_topic = self.get_parameter("goal_topic").value
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.cancel_pub = self.create_publisher(String, "simple_goal/cancel", 10)
        self.stop_pub = self.create_publisher(Twist, "cmd_vel_raw", 10)
        self.status_pub = self.create_publisher(String, "mission/task_status", 10)
        self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(String, "mission/level", self.on_level, 10)
        self.create_subscription(String, "mission/task_command", self.on_command, 10)
        self.create_subscription(String, "simple_goal/status", self.on_simple_status, 10)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f"Mission task manager ready: mode={self.navigation_mode}, "
            f"locations={','.join(sorted(self.locations.keys()))}"
        )

    def load_locations(self, path):
        if not path:
            self.get_logger().warn("No mission_locations_file set")
            return {}
        expanded = os.path.expanduser(path)
        with open(expanded, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        locations = {}
        for name, value in (data.get("locations") or {}).items():
            locations[name] = {
                "x": float(value.get("x", 0.0)),
                "y": float(value.get("y", 0.0)),
                "yaw": float(value.get("yaw", 0.0)),
            }
        return locations

    def on_level(self, msg):
        self.level = msg.data.strip().upper()
        if self.level == "DANGER" and self.active_task != "idle":
            self.cancel_active("blocked_by_danger")

    def on_command(self, msg):
        command = msg.data.strip().lower()
        if command in ("cancel", "stop", "idle"):
            self.cancel_active("cancelled")
            return

        location_name = self.location_for_command(command)
        if location_name is None:
            self.publish_status(f"unknown_command:{command}")
            return
        if location_name not in self.locations:
            self.publish_status(f"missing_location:{location_name}")
            return
        if not self.mission_allowed():
            self.cancel_active(f"blocked_level:{self.level}")
            return

        self.send_goal(command, location_name)

    def on_simple_status(self, msg):
        if self.active_task == "idle":
            return
        if msg.data == "reached" and self.nav2_goal_handle is None:
            self.active_task = "idle"
            self.publish_status("reached")

    def location_for_command(self, command):
        mapping = {
            "delivery": "delivery_dropoff",
            "guide": "guide_destination",
            "evacuate": "evacuation_point",
            "elevator": "elevator_wait",
            "return_home": "home",
            "home": "home",
        }
        return mapping.get(command)

    def mission_allowed(self):
        if self.level == "OK":
            return True
        return self.allow_on_warn and self.level == "WARN"

    def send_goal(self, task_name, location_name):
        location = self.locations[location_name]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = location["x"]
        goal.pose.position.y = location["y"]
        goal.pose.orientation.z = math.sin(location["yaw"] * 0.5)
        goal.pose.orientation.w = math.cos(location["yaw"] * 0.5)

        self.cancel_nav2_goal()
        self.active_task = task_name

        if self.should_use_nav2():
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal
            future = self.nav2_client.send_goal_async(nav_goal)
            future.add_done_callback(self.on_nav2_goal_response)
            self.publish_status(f"nav2_goal:{task_name}->{location_name}")
            return

        if self.goal_pub.get_subscription_count() == 0:
            self.active_task = "idle"
            self.publish_status("navigation_unavailable")
            return

        self.goal_pub.publish(goal)
        self.publish_status(f"simple_goal:{task_name}->{location_name}")

    def should_use_nav2(self):
        if self.navigation_mode == "nav2":
            return self.nav2_client.server_is_ready()
        if self.navigation_mode == "simple":
            return False
        return self.nav2_client.server_is_ready()

    def on_nav2_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.active_task = "idle"
            self.publish_status("nav2_rejected")
            return
        self.nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav2_result)

    def on_nav2_result(self, future):
        self.nav2_goal_handle = None
        result = future.result()
        if result.status == 4:
            self.active_task = "idle"
            self.publish_status("reached")
        else:
            self.active_task = "idle"
            self.publish_status(f"nav2_failed:{result.status}")

    def cancel_active(self, reason):
        self.cancel_nav2_goal()
        cancel_msg = String()
        cancel_msg.data = "cancel"
        self.cancel_pub.publish(cancel_msg)
        self.stop_pub.publish(Twist())
        self.active_task = "idle"
        self.publish_status(reason)

    def cancel_nav2_goal(self):
        if self.nav2_goal_handle is not None:
            self.nav2_goal_handle.cancel_goal_async()
            self.nav2_goal_handle = None

    def publish_status(self, event=None):
        text = f"task={self.active_task}; level={self.level}"
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
    node = MissionTaskManager()
    try:
        rclpy.spin(node)
    finally:
        node.cancel_active("shutdown")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
