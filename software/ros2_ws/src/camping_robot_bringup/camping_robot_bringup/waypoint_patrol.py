import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class WaypointPatrol(Node):
    def __init__(self):
        super().__init__("waypoint_patrol")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("goal_status_topic", "/simple_goal/status")
        self.declare_parameter("navigation_mode", "simple")
        self.declare_parameter("loop", False)
        self.declare_parameter("start_paused", True)
        self.declare_parameter("dispatch_delay_sec", 1.0)
        self.declare_parameter("max_segment_warn_m", 6.0)
        self.declare_parameter("status_hz", 1.0)

        self.goal_topic = self.get_parameter("goal_topic").value
        self.navigation_mode = self.get_parameter("navigation_mode").value
        self.loop = bool(self.get_parameter("loop").value)
        self.paused = bool(self.get_parameter("start_paused").value)
        self.dispatch_delay_sec = float(
            self.get_parameter("dispatch_delay_sec").value
        )
        self.max_segment_warn_m = float(
            self.get_parameter("max_segment_warn_m").value
        )
        self.waypoints = self.load_waypoints(self.get_parameter("waypoints_file").value)
        self.warn_long_segments()
        self.index = 0
        self.waiting_for_goal = False
        self.next_dispatch_time = None
        self.status_state = "unknown"
        self.nav2_goal_handle = None

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.status_pub = self.create_publisher(String, "waypoint_patrol/status", 10)
        self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(
            String,
            self.get_parameter("goal_status_topic").value,
            self.on_goal_status,
            10,
        )
        self.create_subscription(String, "waypoint_patrol/control", self.on_control, 10)
        self.create_timer(0.2, self.tick)
        status_hz = float(self.get_parameter("status_hz").value)
        self.create_timer(1.0 / status_hz, self.republish_status)

        state = "paused" if self.paused else "running"
        self.get_logger().info(
            f"Waypoint patrol ready: {len(self.waypoints)} waypoints, "
            f"state={state}, mode={self.navigation_mode}"
        )
        self.publish_status(state)

    def load_waypoints(self, path):
        if not path:
            self.get_logger().warn("No waypoints_file set; patrol will stay idle")
            return []
        expanded = os.path.expanduser(path)
        with open(expanded, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        waypoints = data.get("waypoints", [])
        clean = []
        for item in waypoints:
            clean.append(
                {
                    "name": item.get("name", f"wp_{len(clean) + 1}"),
                    "x": float(item["x"]),
                    "y": float(item["y"]),
                    "yaw": float(item.get("yaw", 0.0)),
                }
            )
        return clean

    def on_control(self, msg):
        command = msg.data.strip().lower()
        if command == "start":
            self.paused = False
            self.waiting_for_goal = False
            self.next_dispatch_time = self.get_clock().now().nanoseconds
            self.publish_status("running")
            self.get_logger().info("Patrol started")
        elif command == "pause":
            self.paused = True
            self.cancel_nav2_goal()
            self.publish_status("paused")
            self.get_logger().info("Patrol paused")
        elif command == "reset":
            self.index = 0
            self.waiting_for_goal = False
            self.cancel_nav2_goal()
            self.next_dispatch_time = None
            self.publish_status("reset")
            self.get_logger().info("Patrol reset to first waypoint")
        elif command in ("next", "skip"):
            if not self.waypoints:
                self.publish_status("empty")
                return
            self.index = min(self.index + 1, len(self.waypoints) - 1)
            self.waiting_for_goal = False
            self.cancel_nav2_goal()
            self.next_dispatch_time = self.get_clock().now().nanoseconds
            self.publish_status("skipped")
            self.get_logger().info(f"Patrol skipped to waypoint {self.index + 1}")

    def on_goal_status(self, msg):
        if msg.data != "reached" or not self.waiting_for_goal:
            return
        if self.navigation_mode == "simple":
            self.advance_after_reached()

    def tick(self):
        if self.paused or self.waiting_for_goal or not self.waypoints:
            return
        now_ns = self.get_clock().now().nanoseconds
        if self.next_dispatch_time is not None and now_ns < self.next_dispatch_time:
            return
        self.dispatch_current_goal()

    def dispatch_current_goal(self):
        waypoint = self.waypoints[self.index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = waypoint["x"]
        msg.pose.position.y = waypoint["y"]
        msg.pose.orientation.z = math.sin(waypoint["yaw"] * 0.5)
        msg.pose.orientation.w = math.cos(waypoint["yaw"] * 0.5)
        if self.navigation_mode == "nav2":
            if not self.nav2_client.server_is_ready():
                self.publish_status("nav2_waiting")
                self.get_logger().warn(
                    "Nav2 navigate_to_pose action server is not ready",
                    throttle_duration_sec=2.0,
                )
                return
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = msg
            future = self.nav2_client.send_goal_async(goal_msg)
            future.add_done_callback(self.on_nav2_goal_response)
        else:
            self.goal_pub.publish(msg)
        self.waiting_for_goal = True
        self.publish_status(f"goal:{waypoint['name']}")
        self.get_logger().info(
            f"Dispatching waypoint {self.index + 1}/{len(self.waypoints)}: "
            f"{waypoint['name']} x={waypoint['x']:.2f}, "
            f"y={waypoint['y']:.2f}, yaw={waypoint['yaw']:.2f}"
        )

    def on_nav2_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.waiting_for_goal = False
            self.publish_status("nav2_rejected")
            self.get_logger().warn("Nav2 rejected waypoint goal")
            return
        self.nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav2_result)

    def on_nav2_result(self, future):
        self.nav2_goal_handle = None
        result = future.result()
        status = result.status
        if status == 4:
            self.advance_after_reached()
        else:
            self.waiting_for_goal = False
            self.paused = True
            self.publish_status(f"nav2_failed:{status}")
            self.get_logger().warn(f"Nav2 waypoint failed with status={status}")

    def advance_after_reached(self):
        if not self.waiting_for_goal:
            return
        reached = self.waypoints[self.index]
        self.get_logger().info(f"Waypoint reached: {reached['name']}")
        self.index += 1
        self.waiting_for_goal = False
        if self.index >= len(self.waypoints):
            if self.loop and self.waypoints:
                self.index = 0
            else:
                self.paused = True
                self.publish_status("complete")
                self.get_logger().info("Patrol complete")
                return
        self.next_dispatch_time = (
            self.get_clock().now().nanoseconds + int(self.dispatch_delay_sec * 1e9)
        )

    def cancel_nav2_goal(self):
        if self.nav2_goal_handle is not None:
            self.nav2_goal_handle.cancel_goal_async()
            self.nav2_goal_handle = None

    def publish_status(self, status):
        self.status_state = status
        msg = String()
        total = len(self.waypoints)
        current = min(self.index + 1, total) if total else 0
        msg.data = f"{status}; index={current}/{total}"
        self.status_pub.publish(msg)

    def republish_status(self):
        self.publish_status(self.status_state)

    def warn_long_segments(self):
        if len(self.waypoints) < 2:
            return
        for index in range(len(self.waypoints) - 1):
            start = self.waypoints[index]
            end = self.waypoints[index + 1]
            distance = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
            if distance > self.max_segment_warn_m:
                self.get_logger().warn(
                    f"Long patrol segment {start['name']} -> {end['name']}: "
                    f"{distance:.2f}m. Test this route slowly first."
                )


def main():
    rclpy.init()
    node = WaypointPatrol()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
