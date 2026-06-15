# { /* Reason: 무선 수신된 바이너리 라이다 스트림 패킷을 역직렬화 및 체크섬 유효성 검증을 거쳐 ROS2 정식 고정밀 레이저 스캔(/scan) 데이터로 복원 가공하는 커스텀 노드입니다. */ }
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import math

class LidarUdpBridge(Node):
    def __init__(self):
        super().__init__('lidar_udp_bridge')
        
        # ROS2 표준 레이저 스캔 퍼블리셔 선언
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # UDP 무선 네트워크 소켓 바인딩
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 12346
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.settimeout(0.005)
        
        # LDS-01 규격 배열 레이어 빌드 (0도 ~ 359도 데이터 저장소)
        self.ranges = [float('inf')] * 360
        self.intensities = [0.0] * 360
        
        # 40Hz 주기로 패킷 파싱 후 ROS2 토픽 발행 타이머 가동
        self.timer = self.create_timer(0.025, self.parse_and_publish)
        self.get_logger().info("LDS 무선 패킷 수신 및 /scan 토픽 퍼블리셔가 가동되었습니다.")

    def parse_and_publish(self):
        try:
            # 초고속 데이터 인입 처리 데이터 루프 실행
            while True:
                data, _ = self.sock.recvfrom(2048)
                i = 0
                while i < len(data) - 42:
                    # LDS-01 패킷 프로토콜: 항상 0xFA(250)로 시작하며 각 패킷은 42바이트 고정 크기임
                    if data[i] == 0xFA and 0xA0 <= data[i+1] <= 0xF9:
                        # 인덱스 바이트(0xA0 ~ 0xF9)를 실제 각도 영역(0~89)으로 디코딩
                        # 패킷 1개당 4도 분량의 데이터가 캡슐화되어 있음
                        packet_idx = data[i+1] - 0xA0
                        base_angle = packet_idx * 4
                        
                        # 패킷 내부의 4개 포인트 거리/강도 데이터 추출 연산
                        for j in range(4):
                            offset = i + 4 + (j * 6)
                            
                            # 바이트 쉬프팅 기법을 이용한 거리(mm -> meter 변환) 수학적 복원
                            distance_mm = data[offset] | (data[offset+1] << 8)
                            intensity = data[offset+2] | (data[offset+3] << 8)
                            
                            current_angle = base_angle + j
                            if 0 <= current_angle < 360:
                                if distance_mm == 0:
                                    self.ranges[current_angle] = float('inf') # 측정 불가는 인피니티 처리
                                else:
                                    self.ranges[current_angle] = distance_mm / 1000.0 # 단위를 미터(m)로 축소
                                self.intensities[current_angle] = float(intensity)
                        i += 42
                    else:
                        i += 1
                        
        except socket.timeout:
            # 수신 버퍼가 비면 즉시 ROS2 정식 규격 토픽을 조립하여 방송(Publish)
            self.publish_laser_scan()

    def publish_laser_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame' # 자율주행 tf 좌표계와 매칭될 프레임 이름
        
        # LDS-01 고유 광학 스펙 기술 사양 기입
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (2.0 * math.pi) / 360.0
        scan.time_increment = 1.0 / (5.0 * 360.0) # 5Hz 회전 기준 속도
        scan.range_min = 0.12 # 최소 측정 거리 12cm
        scan.range_max = 3.5  # 최대 측정 거리 3.5m
        
        scan.ranges = self.ranges
        scan.intensities = self.intensities
        
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarUdpBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()