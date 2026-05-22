# { /* Reason: 라이다를 PC USB에 직결하여 /dev/ttyUSB0 로컬 시리얼 포트로부터 230400bps 바이너리 데이터를 직접 파싱하여 정식 /scan 토픽으로 발행하는 고신뢰성 드라이버 노드입니다. */ }
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math

class LidarLocalDriver(Node):
    def __init__(self):
        super().__init__('lidar_udp_bridge') # 런치 파일 호환성을 위해 노드 이름 유지
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # 로컬 USB 시리얼 포트 개방 (본인 환경에 맞게 ttyUSB0 또는 ttyACM0 체크)
        try:
            self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=230400, timeout=0.1)
            self.get_logger().info("LDS 라이다 로컬 USB 포트(/dev/ttyUSB0)가 성공적으로 개방되었습니다.")
        except Exception as e:
            self.get_logger().error(f"라이다 포트 개방 실패: {e}")
            
        self.ranges = [float('inf')] * 360
        self.intensities = [0.0] * 360
        
        # 40Hz 주기로 시리얼 버퍼 파싱 및 발행 타이머 가동
        self.timer = self.create_timer(0.025, self.read_and_publish)

    def read_and_publish(self):
        if not hasattr(self, 'ser') or not self.ser.is_open:
            return
            
        try:
            # 시리얼 대량 버퍼 인입 처리
            if self.ser.in_waiting > 42:
                data = self.ser.read(self.ser.in_waiting)
                i = 0
                while i < len(data) - 42:
                    if data[i] == 0xFA and 0xA0 <= data[i+1] <= 0xF9:
                        packet_idx = data[i+1] - 0xA0
                        base_angle = packet_idx * 4
                        
                        for j in range(4):
                            offset = i + 4 + (j * 6)
                            distance_mm = data[offset] | (data[offset+1] << 8)
                            intensity = data[offset+2] | (data[offset+3] << 8)
                            
                            current_angle = base_angle + j
                            if 0 <= current_angle < 360:
                                if distance_mm == 0:
                                    self.ranges[current_angle] = float('inf')
                                else:
                                    self.ranges[current_angle] = distance_mm / 1000.0
                                self.intensities[current_angle] = float(intensity)
                        i += 42
                    else:
                        i += 1
                        
            self.publish_laser_scan()
        except Exception as e:
            pass

    def publish_laser_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (2.0 * math.pi) / 360.0
        scan.time_increment = 1.0 / (5.0 * 360.0)
        scan.range_min = 0.12
        scan.range_max = 3.5
        scan.ranges = self.ranges
        scan.intensities = self.intensities
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarLocalDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()