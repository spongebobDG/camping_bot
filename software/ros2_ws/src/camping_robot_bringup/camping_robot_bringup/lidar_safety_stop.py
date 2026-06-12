import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class LidarSafetyStop(Node):
    def __init__(self):
        super().__init__("lidar_safety_stop")
        self.declare_parameter("front_angle_deg", 45.0)
        self.declare_parameter("enable_forward_safety", True)
        self.declare_parameter("ignore_distance_below_m", 0.25)
        self.declare_parameter("stop_distance_m", 0.45)
        self.declare_parameter("slow_distance_m", 0.80)
        self.declare_parameter("reverse_allowed", True)
        self.declare_parameter("raw_cmd_timeout_sec", 0.35)
        self.declare_parameter("max_reverse_seconds", 1.2)
        self.declare_parameter("emergency_reverse_distance_m", 0.10)
        self.declare_parameter("emergency_reverse_mps", 0.18)
        self.declare_parameter("emergency_reverse_seconds", 0.45)
        self.declare_parameter("emergency_turn_radps", 0.55)

        self.front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.enable_forward_safety = bool(
            self.get_parameter("enable_forward_safety").value
        )
        self.ignore_distance_below = float(
            self.get_parameter("ignore_distance_below_m").value
        )
        self.stop_distance = float(self.get_parameter("stop_distance_m").value)
        self.slow_distance = float(self.get_parameter("slow_distance_m").value)
        self.reverse_allowed = bool(self.get_parameter("reverse_allowed").value)
        self.raw_cmd_timeout = float(self.get_parameter("raw_cmd_timeout_sec").value)
        self.max_reverse_seconds = float(self.get_parameter("max_reverse_seconds").value)
        self.emergency_reverse_distance = float(
            self.get_parameter("emergency_reverse_distance_m").value
        )
        self.emergency_reverse_mps = float(
            self.get_parameter("emergency_reverse_mps").value
        )
        self.emergency_reverse_seconds = float(
            self.get_parameter("emergency_reverse_seconds").value
        )
        self.emergency_turn = float(self.get_parameter("emergency_turn_radps").value)
        self.front_min = math.inf
        self.last_raw_cmd_time = None
        self.last_raw_nonzero = False
        self.reverse_start_time = None
        self.escape_until = None
        self.last_turn_sign = 1.0

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, sensor_qos)
        self.create_subscription(Twist, "cmd_vel_raw", self.on_cmd_vel_raw, 10)
        self.create_timer(0.05, self.watchdog)
        self.create_timer(1.0, self.report_status)
        self.get_logger().info(
            f"Safety stop active: forward_safety={self.enable_forward_safety}, "
            f"ignore<{self.ignore_distance_below:.2f}m, "
            f"stop<{self.stop_distance:.2f}m, slow<{self.slow_distance:.2f}m"
        )

    def on_scan(self, msg: LaserScan):
        front_ranges = []
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < self.ignore_distance_below:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= self.front_angle:
                front_ranges.append(distance)
        self.front_min = min(front_ranges) if front_ranges else math.inf

    def on_cmd_vel_raw(self, msg: Twist):
        now = self.get_clock().now()
        self.last_raw_cmd_time = now
        self.last_raw_nonzero = abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4

        filtered = Twist()
        filtered.linear.x = msg.linear.x
        filtered.angular.z = msg.angular.z

        if abs(msg.angular.z) > 1e-3:
            self.last_turn_sign = math.copysign(1.0, msg.angular.z)

        if self.escape_until is not None:
            if now.nanoseconds < self.escape_until:
                filtered.linear.x = -self.emergency_reverse_mps
                filtered.angular.z = -self.last_turn_sign * self.emergency_turn
                self.pub.publish(filtered)
                return
            self.escape_until = None

        moving_forward = msg.linear.x > 0.0
        if (
            self.enable_forward_safety
            and moving_forward
            and self.front_min <= self.emergency_reverse_distance
            and self.reverse_allowed
        ):
            self.escape_until = now.nanoseconds + int(
                self.emergency_reverse_seconds * 1e9
            )
            filtered.linear.x = -self.emergency_reverse_mps
            filtered.angular.z = -self.last_turn_sign * self.emergency_turn
            self.get_logger().warn(
                f"Emergency escape: front_min={self.front_min:.2f}m, backing up",
                throttle_duration_sec=1.0,
            )
            self.pub.publish(filtered)
            return

        if (
            self.enable_forward_safety
            and moving_forward
            and self.front_min <= self.stop_distance
        ):
            filtered.linear.x = 0.0
            filtered.angular.z = msg.angular.z
            self.get_logger().warn(
                f"Forward blocked: front_min={self.front_min:.2f}m "
                f"<= stop_distance={self.stop_distance:.2f}m",
                throttle_duration_sec=1.0,
            )
        elif (
            self.enable_forward_safety
            and moving_forward
            and self.front_min <= self.slow_distance
        ):
            scale = max(0.2, self.front_min / self.slow_distance)
            filtered.linear.x = msg.linear.x * scale
            self.get_logger().info(
                f"Forward slowed: front_min={self.front_min:.2f}m scale={scale:.2f}",
                throttle_duration_sec=1.0,
            )

        if not self.reverse_allowed and msg.linear.x < 0.0:
            filtered.linear.x = 0.0

        if self.max_reverse_seconds > 0.0 and msg.linear.x < -1e-4:
            if self.reverse_start_time is None:
                self.reverse_start_time = now
            reverse_elapsed = (now - self.reverse_start_time).nanoseconds / 1e9
            if reverse_elapsed > self.max_reverse_seconds:
                filtered.linear.x = 0.0
                filtered.angular.z = 0.0
                self.get_logger().warn(
                    "Reverse command time limit reached; stopping",
                    throttle_duration_sec=1.0,
                )
        else:
            self.reverse_start_time = None

        if abs(filtered.linear.x) > 1e-4 or abs(filtered.angular.z) > 1e-4:
            self.get_logger().info(
                f"cmd pass raw=({msg.linear.x:.2f}, {msg.angular.z:.2f}) "
                f"filtered=({filtered.linear.x:.2f}, {filtered.angular.z:.2f})",
                throttle_duration_sec=1.0,
            )

        self.pub.publish(filtered)

    def watchdog(self):
        if self.last_raw_cmd_time is None or not self.last_raw_nonzero:
            return

        elapsed = (self.get_clock().now() - self.last_raw_cmd_time).nanoseconds / 1e9
        if elapsed > self.raw_cmd_timeout:
            self.pub.publish(Twist())
            self.last_raw_nonzero = False
            self.reverse_start_time = None
            self.escape_until = None
            self.get_logger().warn("Raw cmd timeout; publishing stop")

    def report_status(self):
        if math.isfinite(self.front_min):
            self.get_logger().info(f"front_min={self.front_min:.2f}m")
        else:
            self.get_logger().info("front_min=inf")


def main():
    rclpy.init()
    node = LidarSafetyStop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
