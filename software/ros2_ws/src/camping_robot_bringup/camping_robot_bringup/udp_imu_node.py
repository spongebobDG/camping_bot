import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class UdpImuNode(Node):
    def __init__(self):
        super().__init__("udp_imu_node")
        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("imu_port", 12348)
        self.declare_parameter("frame_id", "imu_link")

        bind_ip = self.get_parameter("bind_ip").value
        imu_port = int(self.get_parameter("imu_port").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((bind_ip, imu_port))

        self.create_timer(0.005, self.poll_socket)
        self.get_logger().info(f"Listening for IMU UDP on {bind_ip}:{imu_port}")

    def poll_socket(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(512)
            except BlockingIOError:
                return

            try:
                ax, ay, az, gx, gy, gz = [
                    float(part) for part in data.decode("ascii").strip().split(",")
                ]
            except ValueError:
                self.get_logger().warning(f"Bad IMU packet: {data!r}")
                continue

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            msg.orientation_covariance[0] = -1.0
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = UdpImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
