import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu


class StraightTrimTest(Node):
    def __init__(self):
        super().__init__("straight_trim_test")
        self.declare_parameter("linear_mps", 0.32)
        self.declare_parameter("run_seconds", 3.0)
        self.declare_parameter("stop_seconds", 1.0)
        self.declare_parameter("imu_topic", "/imu/data_raw")

        self.linear_mps = float(self.get_parameter("linear_mps").value)
        self.run_seconds = float(self.get_parameter("run_seconds").value)
        self.stop_seconds = float(self.get_parameter("stop_seconds").value)
        imu_topic = self.get_parameter("imu_topic").value

        self.pub = self.create_publisher(Twist, "cmd_vel_raw", 10)
        self.create_subscription(Imu, imu_topic, self.on_imu, 20)

        self.integrating = False
        self.last_imu_time = None
        self.yaw_delta = 0.0
        self.peak_abs_gyro_z = 0.0

    def on_imu(self, msg: Imu):
        now = self.get_clock().now()
        gyro_z = msg.angular_velocity.z
        self.peak_abs_gyro_z = max(self.peak_abs_gyro_z, abs(gyro_z))

        if not self.integrating:
            self.last_imu_time = now
            return

        if self.last_imu_time is not None:
            dt = (now - self.last_imu_time).nanoseconds / 1e9
            if 0.0 < dt < 0.2:
                self.yaw_delta += gyro_z * dt
        self.last_imu_time = now

    def publish_for(self, msg: Twist, seconds: float):
        end_time = time.monotonic() + seconds
        while time.monotonic() < end_time and rclpy.ok():
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)

    def run(self):
        self.get_logger().warn("Run this on the floor with a clear, short path ahead.")
        self.get_logger().warn("Keep one hand ready to stop the robot.")
        time.sleep(1.0)

        self.yaw_delta = 0.0
        self.peak_abs_gyro_z = 0.0
        self.integrating = True

        move = Twist()
        move.linear.x = self.linear_mps
        self.get_logger().info(
            f"Driving straight for {self.run_seconds:.1f}s at {self.linear_mps:.2f}m/s"
        )
        self.publish_for(move, self.run_seconds)

        self.integrating = False
        stop = Twist()
        self.get_logger().info("Sending stop command")
        self.publish_for(stop, self.stop_seconds)

        yaw_deg = math.degrees(self.yaw_delta)
        self.get_logger().info(
            f"Integrated IMU yaw drift: {yaw_deg:.2f}deg "
            f"(peak gyro z {self.peak_abs_gyro_z:.3f}rad/s)"
        )
        self.get_logger().info(
            "If the robot physically veered right, increase RIGHT_MOTOR_SCALE. "
            "If it veered left, decrease RIGHT_MOTOR_SCALE or increase LEFT_MOTOR_SCALE."
        )


def main():
    rclpy.init()
    node = StraightTrimTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
