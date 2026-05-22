# { /* Reason: ESP32-S3에서 무선 수신한 로우레벨 주행 거리를 ROS2 자율주행 내비게이션(Nav2) 표준 오도메트리 데이터로 변환 발행하는 커스텀 노드입니다. */ }
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import socket
import math

class Esp32OdomPublisher(Node):
    def __init__(self):
        super().__init__('esp32_odom_publisher')
        
        # ROS2 퍼블리셔 및 TF 브로드캐스터 초기화
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # UDP 소켓 서버 설정
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 12345
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.settimeout(0.01) # 타임아웃 설정을 통해 ROS2 루프가 멈추지 않도록 방지
        
        # 로봇 위치 상태 변수 초기화 (X, Y, Heading Angle)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # 바퀴 간 거리 (Track Width) 설정 - 본인 로봇에 맞게 줄자로 재서 수정 (예: 15cm = 0.15)
        self.wheel_track = 0.15 
        
        self.last_left_dist = 0.0
        self.last_right_dist = 0.0
        
        # 타이머 서브루틴 구동 (연산 주기 20Hz)
        self.timer = self.create_timer(0.05, self.update_odometry)
        self.get_logger().info("ESP32-S3 비동기 오도메트리 수신 노드가 가동되었습니다.")

    def update_odometry(self):
        try:
            # UDP 버퍼로부터 최신 데이터 패킷 수신
            data, addr = self.sock.recvfrom(1024)
            decoded = data.decode('utf-8')
            left_dist, right_dist, _ = map(float, decoded.split(','))
            
            # 이전 주기 대비 이동 변위(Delta) 계산
            d_left = left_dist - self.last_left_dist
            d_right = right_dist - self.last_right_dist
            
            # 로봇 중심점의 직선 이동 변위 및 회전각(Delta Theta) 도출
            d_center = (d_left + d_right) / 2.0
            d_th = (d_right - d_left) / self.wheel_track
            
            # 현재 누적 삼각측량 좌표 갱신
            self.x += d_center * math.cos(self.th)
            self.y += d_center * math.sin(self.th)
            self.th += d_th
            
            # 과거 데이터 백업
            self.last_left_dist = left_dist
            self.last_right_dist = right_dist
            
            current_time = self.get_clock().now().to_msg()
            
            # 1. 오도메트리 TF 좌표 변환(odom -> base_link) 발행
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            
            # 오일러 각도를 쿼터니언(Quaternion) 구조로 변환
            q = self.euler_to_quaternion(0, 0, self.th)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)
            
            # 2. 정식 /odom 토픽 메시지 빌드 및 퍼블리시
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            self.odom_pub.publish(odom)
            
        except socket.timeout:
            # 패킷이 아직 도착 안 했을 때는 에러를 내지 않고 유연하게 패스
            pass

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = Esp32OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
