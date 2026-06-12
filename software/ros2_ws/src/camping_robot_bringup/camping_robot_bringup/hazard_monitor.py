import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String


class HazardMonitor(Node):
    def __init__(self):
        super().__init__("hazard_monitor")
        self.declare_parameter("scan_topic", "/scan_viz")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("front_angle_deg", 35.0)
        self.declare_parameter("close_obstacle_m", 0.35)
        self.declare_parameter("critical_obstacle_m", 0.18)
        self.declare_parameter("buzzer_on_close_obstacle", True)
        self.declare_parameter("tilt_warn_deg", 18.0)
        self.declare_parameter("tilt_critical_deg", 28.0)
        self.declare_parameter("enable_tilt_detection", True)
        self.declare_parameter("tilt_calibration_samples", 40)
        self.declare_parameter("tilt_min_accel_mps2", 6.0)
        self.declare_parameter("tilt_max_accel_mps2", 14.0)
        self.declare_parameter("scan_timeout_sec", 1.0)
        self.declare_parameter("imu_timeout_sec", 1.0)
        self.declare_parameter("publish_hz", 2.0)

        self.front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.close_obstacle = float(self.get_parameter("close_obstacle_m").value)
        self.critical_obstacle = float(self.get_parameter("critical_obstacle_m").value)
        self.buzzer_on_close_obstacle = bool(
            self.get_parameter("buzzer_on_close_obstacle").value
        )
        self.tilt_warn = math.radians(float(self.get_parameter("tilt_warn_deg").value))
        self.tilt_critical = math.radians(
            float(self.get_parameter("tilt_critical_deg").value)
        )
        self.enable_tilt_detection = bool(
            self.get_parameter("enable_tilt_detection").value
        )
        self.tilt_calibration_samples = int(
            self.get_parameter("tilt_calibration_samples").value
        )
        self.tilt_min_accel = float(self.get_parameter("tilt_min_accel_mps2").value)
        self.tilt_max_accel = float(self.get_parameter("tilt_max_accel_mps2").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self.imu_timeout = float(self.get_parameter("imu_timeout_sec").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        self.front_min = math.inf
        self.tilt_rad = 0.0
        self.last_scan_time = None
        self.last_imu_time = None
        self.last_hazard = ""
        self.gravity_ref = None
        self.gravity_sum = [0.0, 0.0, 0.0]
        self.gravity_sample_count = 0

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.hazard_pub = self.create_publisher(String, "camping_robot/hazard", 10)
        self.buzzer_pub = self.create_publisher(Bool, "warning_buzzer", 10)
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.on_scan,
            sensor_qos,
        )
        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self.on_imu,
            10,
        )
        self.create_timer(1.0 / publish_hz, self.report)
        self.get_logger().info("Hazard monitor started")

    def on_scan(self, msg: LaserScan):
        ranges = []
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= self.front_angle:
                ranges.append(distance)
        self.front_min = min(ranges) if ranges else math.inf
        self.last_scan_time = self.get_clock().now()

    def on_imu(self, msg: Imu):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.last_imu_time = self.get_clock().now()

        if not self.enable_tilt_detection:
            self.tilt_rad = 0.0
            return

        accel_norm = self.normalize_vector((ax, ay, az))
        if accel_norm is None:
            return

        if self.gravity_ref is None:
            self.gravity_sum[0] += ax
            self.gravity_sum[1] += ay
            self.gravity_sum[2] += az
            self.gravity_sample_count += 1
            self.tilt_rad = 0.0
            if self.gravity_sample_count >= self.tilt_calibration_samples:
                self.gravity_ref = self.normalize_vector(self.gravity_sum)
                if self.gravity_ref is None:
                    self.gravity_sample_count = 0
                    self.gravity_sum = [0.0, 0.0, 0.0]
                    return
                self.get_logger().info(
                    "Tilt baseline calibrated from current IMU mounting direction"
                )
            return

        dot = sum(accel_norm[i] * self.gravity_ref[i] for i in range(3))
        dot = max(-1.0, min(1.0, dot))
        self.tilt_rad = math.acos(dot)

    def report(self):
        now = self.get_clock().now()
        hazards = []

        scan_age = self.age(now, self.last_scan_time)
        imu_age = self.age(now, self.last_imu_time)

        if scan_age > self.scan_timeout:
            hazards.append(f"SCAN_STALE age={scan_age:.1f}s")
        if imu_age > self.imu_timeout:
            hazards.append(f"IMU_STALE age={imu_age:.1f}s")

        if self.front_min <= self.critical_obstacle:
            hazards.append(f"OBSTACLE_CRITICAL front={self.front_min:.2f}m")
        elif self.front_min <= self.close_obstacle:
            hazards.append(f"OBSTACLE_CLOSE front={self.front_min:.2f}m")

        if self.enable_tilt_detection and self.gravity_ref is not None:
            if self.tilt_rad >= self.tilt_critical:
                hazards.append(
                    f"TILT_CRITICAL tilt={math.degrees(self.tilt_rad):.1f}deg"
                )
            elif self.tilt_rad >= self.tilt_warn:
                hazards.append(f"TILT_WARN tilt={math.degrees(self.tilt_rad):.1f}deg")

        text = "; ".join(hazards) if hazards else "OK"
        msg = String()
        msg.data = text
        self.hazard_pub.publish(msg)

        buzzer = Bool()
        buzzer_prefixes = ["OBSTACLE_CRITICAL", "TILT_CRITICAL", "SCAN_STALE"]
        if self.buzzer_on_close_obstacle:
            buzzer_prefixes.append("OBSTACLE_CLOSE")
        buzzer.data = any(item.startswith(tuple(buzzer_prefixes)) for item in hazards)
        self.buzzer_pub.publish(buzzer)

        if text != self.last_hazard:
            if hazards:
                self.get_logger().warn(text)
            else:
                self.get_logger().info("Hazard state OK")
            self.last_hazard = text

    @staticmethod
    def age(now, last_time):
        if last_time is None:
            return math.inf
        return (now - last_time).nanoseconds / 1e9

    def normalize_vector(self, vector):
        norm = math.sqrt(sum(value * value for value in vector))
        if norm < self.tilt_min_accel or norm > self.tilt_max_accel:
            return None
        return tuple(value / norm for value in vector)


def main():
    rclpy.init()
    node = HazardMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
