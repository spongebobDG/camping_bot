import socket
import math
import pygame
import sys

# --- 네트워크 세팅 ---
UDP_IP = "0.0.0.0"
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

data_buffer = bytearray()
PACKET_SIZE = 84
HEADER_PATTERN = b'\x60\x66\x86'

# --- Pygame 디스플레이 세팅 ---
pygame.init()
WIDTH, HEIGHT = 800, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("LDS14 LiDAR Real-time 2D Radar (Fixed)")
clock = pygame.time.Clock()

cx, cy = WIDTH // 2, HEIGHT // 2
zoom = 0.12  # 마우스 휠로 조절 가능

# 360도 공간의 최신 거리를 담을 딕셔너리
lidar_scan_data = {}

print("🚀 [각도 수정 버전] 360도 전방위 매핑을 시작합니다...")

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 4: zoom = min(2.0, zoom + 0.01)
            elif event.button == 5: zoom = max(0.01, zoom - 0.01)

    try:
        packet, addr = sock.recvfrom(2048)
        data_buffer.extend(packet)
    except BlockingIOError:
        pass

    # 데이터 버퍼 동기화 및 파싱
    while len(data_buffer) >= PACKET_SIZE:
        if data_buffer[:3] != HEADER_PATTERN:
            del data_buffer[0]
            continue
            
        raw_packet = data_buffer[:PACKET_SIZE]
        del data_buffer[:PACKET_SIZE]

        # 💡 [LDS14 핵심 각도 추출 공정]
        # 인덱스 3번과 4번 바이트에 저장된 패킷의 '시작 각도'를 결합합니다.
        # 데이터가 밀렸을 가능성을 고려해 하위 바이트 결합 공식을 표준화합니다.
        raw_angle = (raw_packet[4] << 8) | raw_packet[3]
        
        # 만약 해당 비트가 각도 값이 아니라면, 패킷 내부 인덱스를 각도로 강제 환산
        if raw_angle == 0 or raw_angle > 36000:
            packet_idx = raw_packet[3]
            start_angle = (packet_idx * 40.0) % 360.0  # LDS14 표준: 패킷당 40도 매핑 분할
            angle_increment = 40.0 / 36.0
        else:
            start_angle = (raw_angle / 100.0) % 360.0  # 정밀 각도 변환 (0.01도 단위 보정)
            angle_increment = 1.0  # 샘플당 1도씩 증가

        sample_count = 36
        data_start_offset = 6  # 데이터 시작점

        for i in range(sample_count):
            offset = data_start_offset + (i * 2)
            if offset + 1 >= PACKET_SIZE:
                break
                
            raw_dist = (raw_packet[offset + 1] << 8) | raw_packet[offset]
            
            # 에러 데이터 및 허공 필터링
            if (raw_dist & 0x8000) or (raw_dist == 0) or (raw_dist > 6000):
                continue  
                
            distance_mm = raw_dist & 0x3FFF
            
            # 💡 샘플별로 각도를 360도 공간에 골고루 넓게 펼쳐줍니다.
            calculated_angle = (start_angle + (i * angle_increment)) % 360.0
            
            lidar_scan_data[int(calculated_angle)] = distance_mm

    # 렌더링 시작
    screen.fill((10, 12, 18))

    # 가이드라인 동심원 그리기
    for r_meters in [1000, 2000, 3000, 4000]:
        pygame.draw.circle(screen, (35, 40, 50), (cx, cy), int(r_meters * zoom), 1)
    pygame.draw.line(screen, (30, 35, 45), (0, cy), (WIDTH, cy), 1)
    pygame.draw.line(screen, (30, 35, 45), (cx, 0), (cx, HEIGHT), 1)

    # 투사 가동 ($x = r \cdot \cos\theta$)
    for angle, dist in list(lidar_scan_data.items()):
        # 360도 전방위 공간 매핑을 위한 삼각함수 좌표계 변환
        # 라이다 회전 방향에 따라 부호 조절
        radian = math.radians(angle) 
        
        rx = int(cx + dist * zoom * math.cos(radian))
        ry = int(cy + dist * zoom * math.sin(radian))

        if 0 <= rx < WIDTH and 0 <= ry < HEIGHT:
            pygame.draw.circle(screen, (255, 50, 50), (rx, ry), 2)

    # 중앙 로봇 마커
    pygame.draw.circle(screen, (0, 255, 120), (cx, cy), 5)

    pygame.display.flip()
    clock.tick(60)