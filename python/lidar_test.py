import socket
import math

UDP_IP = "0.0.0.0"
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

data_buffer = bytearray()
PACKET_SIZE = 84

# 🎯 분석 결과 매칭된 LDS14의 진짜 3바이트 헤더 패턴
HEADER_PATTERN = b'\x60\x66\x86'

print("🚀 [LDS14 최종 가동] 진짜 헤더 기반 각도/거리 파싱 엔진 시작...")

while True:
    packet, addr = sock.recvfrom(2048)
    data_buffer.extend(packet)

    # 버퍼에 데이터가 충분히 있을 때 순회
    while len(data_buffer) >= PACKET_SIZE:
        
        # 1. 버퍼의 맨 앞이 우리가 찾은 진짜 헤더 패턴(60 66 86)인지 검사
        if data_buffer[:3] != HEADER_PATTERN:
            # 헤더가 맞지 않으면 맞을 때까지 1바이트씩 지우며 강제 동기화(Sync)
            del data_buffer[0]
            continue
            
        # 2. 정확한 시작점을 찾았으므로 84바이트 패킷 도려내기
        raw_packet = data_buffer[:PACKET_SIZE]
        del data_buffer[:PACKET_SIZE]

        # 3. LDS14 프로토콜 분해
        # 헤더 3바이트(0~2) + 속도/인덱스 2바이트(3~4) 패스 후 5번 바이트부터 데이터 시작
        # 84바이트 중 헤더부 제외 영역에 2바이트 크기의 거리 샘플들이 촘촘히 들어있습니다.
        sample_count = 36
        data_start_offset = 6  # 프로토콜 오프셋 보정
        
        # 라이다가 360도를 돌면서 쏘기 때문에, 현재 수신된 패킷의 각도 정보를 임시 보간합니다.
        # 데이터 뭉치 안에서 연속된 제로(0000)가 아닌 유효 거리를 추출합니다.
        valid_samples = []
        
        for i in range(sample_count):
            offset = data_start_offset + (i * 2)
            if offset + 1 >= PACKET_SIZE:
                break
                
            # 2바이트 결합 (Little Endian 구조 계산)
            raw_dist = (raw_packet[offset + 1] << 8) | raw_packet[offset]
            
            # 에러 비트 스크리닝 (상위 비트 플래그 필터링 및 측정 불능 0mm 제외)
            if (raw_dist & 0x8000) or (raw_dist == 0):
                continue  
                
            distance_mm = raw_dist & 0x3FFF  # 하위 14비트가 진짜 거리 정보
            
            # 360도 전방위를 36개 샘플 공간으로 분할 매핑 계산 (인덱스 기반 각도 역산)
            calculated_angle = (i * (360.0 / sample_count)) % 360.0
            
            valid_samples.append((calculated_angle, distance_mm))
            
        # 4. 정제된 진짜 물리 거리 데이터 터미널 출력
        if valid_samples:
            print(f"🎯 패킷 정렬 성공! 유효 장애물 샘플: {len(valid_samples)}개 포착")
            # 대표로 주변에 감지된 첫 번째 물체의 방향과 거리를 보여줍니다.
            print(f"   └─ [실시간 거리 감지] 방향: {valid_samples[0][0]:5.1f}° ➔ 거리: {valid_samples[0][1]} mm")