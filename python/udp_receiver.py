import socket

# ESP32 코드에 적은 포트 번호와 일치해야 합니다.
UDP_IP = "0.0.0.0" # 모든 네트워크 인터페이스로부터 수신 가능하도록 설정
UDP_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"노트북 UDP 서버 가동 중... 포트 번호: {UDP_PORT}")

try:
    while True:
        # 패킷 수신 대기 (최대 1024 바이트)
        data, addr = sock.recvfrom(1024)
        decoded_data = data.decode('utf-8')
        
        # 받은 문자열을 쉼표 기준으로 파싱
        try:
            left, right, total = map(float, decoded_data.split(','))
            print(f"[{addr[0]}] 수신 데이터 -> 좌측: {left}m | 우측: {right}m | 전체 이동: {total}m")
        except ValueError:
            print(f"파싱 실패 데이터: {decoded_data}")
            
except KeyboardInterrupt:
    print("\n서버 종료.")
    sock.close()