import math
import socket
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


PACKET_SIZE = 47
POINTS_PER_PACKET = 12


class Lds14UdpNode(Node):
    def __init__(self):
        super().__init__("lds14_udp_node")
        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("lidar_port", 12346)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("scan_size", 360)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("status_log_hz", 1.0)
        self.declare_parameter("mirror_scan", True)

        bind_ip = self.get_parameter("bind_ip").value
        lidar_port = int(self.get_parameter("lidar_port").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.scan_size = int(self.get_parameter("scan_size").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        status_log_hz = float(self.get_parameter("status_log_hz").value)
        self.mirror_scan = bool(self.get_parameter("mirror_scan").value)
        self.scan_time = 1.0 / publish_hz

        self.ranges = [math.inf] * self.scan_size
        self.intensities = [0.0] * self.scan_size
        self.packet_count = 0
        self.publish_count = 0
        self.last_packet_time = None

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.fast_pub = self.create_publisher(LaserScan, "scan_fast", sensor_qos)
        self.pub = self.create_publisher(LaserScan, "scan", sensor_qos)
        self.viz_pub = self.create_publisher(LaserScan, "scan_viz", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind((bind_ip, lidar_port))

        self.create_timer(0.002, self.poll_socket)
        self.create_timer(1.0 / publish_hz, self.publish_scan)
        if status_log_hz > 0.0:
            self.create_timer(1.0 / status_log_hz, self.report_status)
        self.get_logger().info(f"Listening for LDS14 UDP on {bind_ip}:{lidar_port}")

    def poll_socket(self):
        for _ in range(32):
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                return

            for offset in range(0, len(data) - PACKET_SIZE + 1, PACKET_SIZE):
                self.parse_packet(data[offset : offset + PACKET_SIZE])
                self.packet_count += 1
                self.last_packet_time = self.get_clock().now()

    def parse_packet(self, packet: bytes):
        if len(packet) != PACKET_SIZE or packet[0] != 0x54 or packet[1] != 0x2C:
            return

        start_angle = struct.unpack_from("<H", packet, 4)[0] / 100.0
        end_angle = struct.unpack_from("<H", packet, 42)[0] / 100.0
        angle_span = (end_angle - start_angle) % 360.0

        for i in range(POINTS_PER_PACKET):
            point_offset = 6 + i * 3
            distance_mm = struct.unpack_from("<H", packet, point_offset)[0]
            confidence = packet[point_offset + 2]
            angle_deg = (start_angle + angle_span * i / (POINTS_PER_PACKET - 1)) % 360.0
            index = int(round(angle_deg)) % self.scan_size
            distance_m = distance_mm / 1000.0

            if self.range_min <= distance_m <= self.range_max:
                self.ranges[index] = distance_m
                self.intensities[index] = float(confidence)
            else:
                self.ranges[index] = math.inf
                self.intensities[index] = 0.0

    def publish_scan(self):
        if self.last_packet_time is None:
            return

        packet_age = (self.get_clock().now() - self.last_packet_time).nanoseconds / 1e9
        if packet_age > 0.5:
            return

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = -math.pi
        msg.angle_increment = 2.0 * math.pi / self.scan_size
        msg.angle_max = msg.angle_min + msg.angle_increment * (self.scan_size - 1)
        msg.time_increment = 0.0
        msg.scan_time = self.scan_time
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        half_scan = self.scan_size // 2
        ordered_ranges = list(self.ranges[half_scan:]) + list(self.ranges[:half_scan])
        ordered_intensities = list(self.intensities[half_scan:]) + list(
            self.intensities[:half_scan]
        )
        if self.mirror_scan:
            ordered_ranges = [
                ordered_ranges[(self.scan_size - index) % self.scan_size]
                for index in range(self.scan_size)
            ]
            ordered_intensities = [
                ordered_intensities[(self.scan_size - index) % self.scan_size]
                for index in range(self.scan_size)
            ]
        msg.ranges = ordered_ranges
        msg.intensities = ordered_intensities
        self.fast_pub.publish(msg)
        self.pub.publish(msg)
        self.viz_pub.publish(msg)
        self.publish_count += 1

    def report_status(self):
        if self.packet_count == 0:
            self.get_logger().warn("No fresh LDS14 UDP packets received")
        self.get_logger().info(
            f"LDS14 packets={self.packet_count}, scan_published={self.publish_count}"
        )
        self.packet_count = 0
        self.publish_count = 0


def main():
    rclpy.init()
    node = Lds14UdpNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
