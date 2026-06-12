// Camping_bot firmware snapshot: esp32_s3_lidar_v1
// Date: 2026-06-12
// Notes: ESP32-S3 LDS14 UDP lidar sender baseline used by ROS2 lds14_udp_node.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "aip2.4GHz";     
const char* password = "aip123456";
const char* pcIP     = "192.168.0.8"; 
const int lidarPort  = 12346; 

WiFiUDP lidarUdp;
HardwareSerial LidarSerial(2);

#define LIDAR_PACKET_SIZE 47

QueueHandle_t lidarQueue;

struct LidarPacket {
    uint8_t data[LIDAR_PACKET_SIZE];
};

// 프레임 동기화 상태 머신 기반의 고속 수신 타스크 (Core 1 작동)
void LidarTask(void * pvParameters) {
  uint8_t packetBuffer[LIDAR_PACKET_SIZE];
  int index = 0;
  
  enum SyncState { WAIT_H1, WAIT_H2, COLLECT };
  SyncState state = WAIT_H1;

  for(;;) {
    if (LidarSerial.available() > 0) {
      uint8_t b = LidarSerial.read();
      
      if (state == WAIT_H1) {
        if (b == 0x54) { 
          packetBuffer[0] = b;
          state = WAIT_H2;
        }
      }
      else if (state == WAIT_H2) {
        if (b == 0x2C) { 
          packetBuffer[1] = b;
          index = 2;
          state = COLLECT;
        } else {
          if (b == 0x54) { 
            packetBuffer[0] = b;
            state = WAIT_H2;
          } else {
            state = WAIT_H1;
          }
        }
      }
      else if (state == COLLECT) {
        packetBuffer[index++] = b;
        if (index >= LIDAR_PACKET_SIZE) { 
          LidarPacket packet;
          memcpy(packet.data, packetBuffer, LIDAR_PACKET_SIZE);
          xQueueSend(lidarQueue, &packet, 0);
          state = WAIT_H1; 
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- [LDS14 실시간 즉시 전송 버전] 부팅 ---");
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\n[성공] Wi-Fi 연결 완료.");
  WiFi.setSleep(false); // 무선 지연 방지
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP()); // <-- 이 IP 주소를 확인합니다 (예: 192.168.0.15)
  
  lidarUdp.begin(lidarPort);
  LidarSerial.begin(115200, SERIAL_8N1, 17, 16); 
  LidarSerial.setRxBufferSize(4096);
  
  // 큐 크기를 넉넉히 확장
  lidarQueue = xQueueCreate(100, sizeof(LidarPacket));
  xTaskCreatePinnedToCore(LidarTask, "LidarTask", 4096, NULL, 3, NULL, 1);
}
void loop() {
  #define RE_BUNDLE_COUNT 3  // 3개 묶음 유지
  static uint8_t sendBuffer[LIDAR_PACKET_SIZE * RE_BUNDLE_COUNT];
  int packetCount = 0;
  LidarPacket receivedPacket;

  // 디버깅용 카운터 변수 추가
  static unsigned long lastLogTime = 0;
  static unsigned long totalSentPackets = 0;

  while (packetCount < RE_BUNDLE_COUNT) {
    if (xQueueReceive(lidarQueue, &receivedPacket, 0) == pdTRUE) {
      memcpy(&sendBuffer[packetCount * LIDAR_PACKET_SIZE], receivedPacket.data, LIDAR_PACKET_SIZE);
      packetCount++;
    } else {
      break; 
    }
  }
  if (packetCount > 0) {
    lidarUdp.beginPacket(pcIP, lidarPort);
    lidarUdp.write(sendBuffer, LIDAR_PACKET_SIZE * packetCount);
    
    // [수정] endPacket()이 1을 리턴해야 진짜 성공입니다.
    int result = lidarUdp.endPacket();
    if (result == 1) {
      totalSentPackets += packetCount; 
    } else {
      // 0이 나오면 네트워크 레이어에서 송신 실패한 것입니다.
      Serial.printf("❌ UDP 전송 실패! 에러 코드: %d\n", result);
    }
  }

  // ★ [하드웨어 디버그 추가] 1초마다 시리얼 모니터에 나 살아있다고 외치기
  unsigned long now = millis();
  if (now - lastLogTime >= 1000) {
    lastLogTime = now;
    Serial.printf("[라이다 송신 활성화] 1초간 누적 보낸 패킷 수: %d 개 | 현재 큐에 대기중인 메시지: %d 개\n", 
                  totalSentPackets, uxQueueMessagesWaiting(lidarQueue));
    totalSentPackets = 0; // 카운터 초기화
  }

  if (uxQueueMessagesWaiting(lidarQueue) == 0) {
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}
