import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class CmdVelOdom(Node):
    def __init__(self):
        super().__init__("cmd_vel_odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("min_linear_for_yaw_mps", 0.01)
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("use_imu_yaw", True)
        self.declare_parameter("imu_timeout_sec", 0.30)
        self.declare_parameter("gyro_z_sign", 1.0)
        self.declare_parameter("gyro_z_deadband_radps", 0.015)
        self.declare_parameter("gyro_bias_alpha", 0.02)
        self.declare_parameter("linear_scale", 1.0)

        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        publish_hz = float(self.get_parameter("publish_hz").value)
        self.min_linear_for_yaw = float(
            self.get_parameter("min_linear_for_yaw_mps").value
        )
        self.imu_topic = self.get_parameter("imu_topic").value
        self.use_imu_yaw = bool(self.get_parameter("use_imu_yaw").value)
        self.imu_timeout_sec = float(self.get_parameter("imu_timeout_sec").value)
        self.gyro_z_sign = float(self.get_parameter("gyro_z_sign").value)
        self.gyro_z_deadband = float(
            self.get_parameter("gyro_z_deadband_radps").value
        )
        self.gyro_bias_alpha = float(self.get_parameter("gyro_bias_alpha").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear = 0.0
        self.angular = 0.0
        self.last_gyro_z = 0.0
        self.gyro_z_bias = 0.0
        self.last_imu_time = None
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel_executed", self.on_cmd_vel, 10)
        self.create_subscription(Imu, self.imu_topic, self.on_imu, 10)
        self.create_timer(1.0 / publish_hz, self.publish_odom)
        self.get_logger().warn(
            "Using command-based odometry. This is temporary and will drift without encoders."
        )
        if self.use_imu_yaw:
            self.get_logger().info(
                f"Using IMU gyro z from {self.imu_topic} for odom yaw"
            )

    def on_cmd_vel(self, msg: Twist):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    def on_imu(self, msg: Imu):
        self.last_imu_time = self.get_clock().now()
        self.last_gyro_z = self.gyro_z_sign * msg.angular_velocity.z

        if abs(self.linear) < self.min_linear_for_yaw and abs(self.angular) < 0.02:
            self.gyro_z_bias = (
                (1.0 - self.gyro_bias_alpha) * self.gyro_z_bias
                + self.gyro_bias_alpha * self.last_gyro_z
            )

    def imu_is_fresh(self, now):
        if self.last_imu_time is None:
            return False
        age = (now - self.last_imu_time).nanoseconds / 1e9
        return age <= self.imu_timeout_sec

    def publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if self.use_imu_yaw and self.imu_is_fresh(now):
            angular_for_odom = self.last_gyro_z - self.gyro_z_bias
            if abs(angular_for_odom) < self.gyro_z_deadband:
                angular_for_odom = 0.0
        else:
            angular_for_odom = (
                self.angular if abs(self.linear) >= self.min_linear_for_yaw else 0.0
            )

        self.yaw += angular_for_odom * dt
        linear_for_odom = self.linear * self.linear_scale
        self.x += linear_for_odom * math.cos(self.yaw) * dt
        self.y += linear_for_odom * math.sin(self.yaw) * dt

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_for_odom
        odom.twist.twist.angular.z = angular_for_odom
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = CmdVelOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
