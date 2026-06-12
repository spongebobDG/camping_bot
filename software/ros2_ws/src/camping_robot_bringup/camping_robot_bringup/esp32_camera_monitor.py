import socket
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Esp32CameraMonitor(Node):
    def __init__(self):
        super().__init__("esp32_camera_monitor")
        self.declare_parameter("enabled", False)
        self.declare_parameter("mode", "udp")
        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("status_port", 12350)
        self.declare_parameter("stream_url", "http://192.168.0.11/stream")
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("check_period_sec", 2.0)
        self.declare_parameter("stale_sec", 8.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.mode = self.get_parameter("mode").value
        bind_ip = self.get_parameter("bind_ip").value
        status_port = int(self.get_parameter("status_port").value)
        self.stream_url = self.get_parameter("stream_url").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        check_period = float(self.get_parameter("check_period_sec").value)
        self.stale_sec = float(self.get_parameter("stale_sec").value)

        self.status_pub = self.create_publisher(String, "camera/status", 10)
        self.online_pub = self.create_publisher(Bool, "camera/online", 10)
        self.last_status = None
        self.last_udp_time = None
        self.sock = None
        if self.mode == "udp":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)
            self.sock.bind((bind_ip, status_port))
            self.create_timer(0.05, self.poll_udp)
        self.create_timer(check_period, self.check_camera)

        if self.enabled:
            self.get_logger().info(
                f"ESP32 camera monitor enabled: mode={self.mode}, "
                f"url={self.stream_url}, udp={bind_ip}:{status_port}"
            )
        else:
            self.get_logger().warn(
                "ESP32 camera monitor disabled. Set esp32_camera_monitor.enabled=true "
                "and stream_url after the camera IP is fixed."
            )

    def check_camera(self):
        if not self.enabled:
            self.publish(False, "DISABLED")
            return

        if self.mode == "udp":
            if self.last_udp_time is None:
                self.publish(False, "WAITING_FOR_UDP_HEARTBEAT")
                return
            age = (self.get_clock().now() - self.last_udp_time).nanoseconds / 1e9
            if age > self.stale_sec:
                self.publish(False, f"UDP_STALE age={age:.1f}s")
            return

        try:
            request = urllib.request.Request(
                self.stream_url,
                headers={"User-Agent": "camping-bot-camera-monitor"},
            )
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                chunk = response.read(128)
                content_type = response.headers.get("Content-Type", "")
                if response.status == 200 and chunk:
                    self.publish(True, f"OK content_type={content_type}")
                else:
                    self.publish(False, f"BAD_RESPONSE status={response.status}")
        except urllib.error.URLError as exc:
            self.publish(False, f"UNREACHABLE {exc}")
        except TimeoutError:
            self.publish(False, "TIMEOUT")
        except Exception as exc:
            self.publish(False, f"ERROR {type(exc).__name__}: {exc}")

    def poll_udp(self):
        if not self.enabled or self.sock is None:
            return

        while True:
            try:
                data, addr = self.sock.recvfrom(512)
            except BlockingIOError:
                return

            text = data.decode("utf-8", errors="replace")
            self.last_udp_time = self.get_clock().now()
            self.publish(True, f"UDP_OK from={addr[0]} {text}")

    def publish(self, online, status):
        online_msg = Bool()
        online_msg.data = online
        self.online_pub.publish(online_msg)

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

        if status != self.last_status:
            if online:
                self.get_logger().info(f"Camera status: {status}")
            else:
                self.get_logger().warn(f"Camera status: {status}")
            self.last_status = status


def main():
    rclpy.init()
    node = Esp32CameraMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
