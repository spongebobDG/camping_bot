// Camping_bot firmware snapshot: ackerman_esp32_imu_v1
// Date: 2026-06-12
// Notes: ESP32 drive + MPU6050 IMU baseline with motor trim, command timeout,
//        and UDP status heartbeat for power/communication diagnosis.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- 네트워크 설정 ---
const char* ssid     = "aip2.4GHz";     
const char* password = "aip123456"; 
const char* pcIP     = "192.168.0.8"; 
const int cmdPort    = 12347; 
const int imuPort    = 12348; 
const int statusPort = 12349;

WiFiUDP cmdUdp;
WiFiUDP imuUdp; 

Adafruit_MPU6050 mpu; 

// --- 로봇 물리 사양 상수 및 모터 핀 ---
const double WHEEL_BASE = 0.20;  
const double TRACK_WIDTH = 0.15; 
const int ENA = 19; const int IN1 = 2;  const int IN2 = 4;   
const int IN3 = 5;  const int IN4 = 18; const int ENB = 15;  
const int SERVO_PIN = 23; 
Servo steerServo;
const int SERVO_CENTER_ANGLE = 90; 
int lastServoAngle = -1; 
const int MOTOR_MIN_PWM = 120; 
const unsigned long CMD_TIMEOUT_MS = 500;
const double LEFT_MOTOR_SCALE = 1.00;
const double RIGHT_MOTOR_SCALE = 1.28;

unsigned long lastImuTime = 0; 
unsigned long lastCmdTime = 0;
unsigned long lastDriveLogTime = 0;
unsigned long lastStatusTime = 0;
unsigned long receivedCmdCount = 0;
bool stoppedByTimeout = true;

double reqLinear = 0.0;
double reqSteering = 0.0;

// 아커만 조향 및 모터 구동 함수
void setAckermanDrive(double linear_vel, double steering_angle) {
  int servoAngle = SERVO_CENTER_ANGLE + (int)(steering_angle * 180.0 / PI);
  servoAngle = constrain(servoAngle, 50, 130); 

  if (servoAngle != lastServoAngle) {
    steerServo.write(servoAngle);
    lastServoAngle = servoAngle; 
  }

  double targetLeftVel = linear_vel * (1.0 - (TRACK_WIDTH / (2.0 * WHEEL_BASE)) * tan(steering_angle));
  double targetRightVel = linear_vel * (1.0 + (TRACK_WIDTH / (2.0 * WHEEL_BASE)) * tan(steering_angle));

  int pwmLeft = map(abs(targetLeftVel) * 1000, 0, 500, 0, 255);
  int pwmRight = map(abs(targetRightVel) * 1000, 0, 500, 0, 255);

  pwmLeft = (int)(pwmLeft * LEFT_MOTOR_SCALE);
  pwmRight = (int)(pwmRight * RIGHT_MOTOR_SCALE);

  if (linear_vel != 0) {
    if (pwmLeft > 0 && pwmLeft < MOTOR_MIN_PWM) pwmLeft = MOTOR_MIN_PWM;
    if (pwmRight > 0 && pwmRight < MOTOR_MIN_PWM) pwmRight = MOTOR_MIN_PWM;
  }

  pwmLeft = constrain(pwmLeft, 0, 255);
  pwmRight = constrain(pwmRight, 0, 255);

  if (linear_vel >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  }
  
  analogWrite(ENA, pwmLeft);
  analogWrite(ENB, pwmRight);

  unsigned long now = millis();
  if (now - lastDriveLogTime >= 500) {
    lastDriveLogTime = now;
    Serial.printf("drive linear=%.3f steering=%.3f servo=%d pwmL=%d pwmR=%d\n",
                  linear_vel, steering_angle, servoAngle, pwmLeft, pwmRight);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n--- [MCU 1] IMU 전송 + 조향 커널 부팅 (엔코더 제거 버전) ---");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\n[성공] Wi-Fi 연결 완료.");
  WiFi.setSleep(false); 
  Serial.println(WiFi.localIP());

  // Hardware I2C 초기화 및 IMU 센서 연결 검증
  Wire.begin(21, 22); // SDA=21, SCL=22
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 센서를 찾을 수 없습니다. 핀 연결을 확인하세요!");
    while (1) { delay(10); }
  }
  Serial.println("⭕ MPU6050 센서 인식 성공.");
  
  // 센서 범위 최적화
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  cmdUdp.begin(cmdPort);
  imuUdp.begin(imuPort); 
  lastCmdTime = millis();

  steerServo.attach(SERVO_PIN);
  steerServo.write(SERVO_CENTER_ANGLE); 

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
}

void loop() {
  // ① [PC -> 하드웨어] 조향 명령 수신
  while (cmdUdp.parsePacket()) {
    char packetBuffer[64];
    int len = cmdUdp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = 0;
      int parsed = sscanf(packetBuffer, "%lf,%lf", &reqLinear, &reqSteering);
      if (parsed == 2) {
        lastCmdTime = millis();
        stoppedByTimeout = false;
        receivedCmdCount++;
        setAckermanDrive(reqLinear, reqSteering);
      }
    }
  }

  unsigned long currentTime = millis();

  if (!stoppedByTimeout && currentTime - lastCmdTime > CMD_TIMEOUT_MS) {
    reqLinear = 0.0;
    reqSteering = 0.0;
    setAckermanDrive(0.0, 0.0);
    stoppedByTimeout = true;
    Serial.println("Command timeout: motors stopped.");
  }

  // ② [하드웨어 -> PC] IMU 데이터 고속 전송 (20Hz = 50ms 주기)
  if (currentTime - lastImuTime >= 50) {
    lastImuTime = currentTime;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    char imuPacket[128];
    // ROS2 표준 가속도(m/s^2) 및 자이로 각속도(rad/s) 포맷
    sprintf(imuPacket, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", 
            a.acceleration.x, a.acceleration.y, a.acceleration.z,
            g.gyro.x, g.gyro.y, g.gyro.z);
            
    imuUdp.beginPacket(pcIP, imuPort);
    imuUdp.write((uint8_t*)imuPacket, strlen(imuPacket));
    imuUdp.endPacket();
  }

  if (currentTime - lastStatusTime >= 1000) {
    lastStatusTime = currentTime;
    int rssi = WiFi.RSSI();
    char statusPacket[160];
    snprintf(statusPacket, sizeof(statusPacket),
             "uptime_ms=%lu,rssi=%d,cmd_count=%lu,last_cmd_age_ms=%lu,free_heap=%lu,ip=%s",
             currentTime,
             rssi,
             receivedCmdCount,
             currentTime - lastCmdTime,
             (unsigned long)ESP.getFreeHeap(),
             WiFi.localIP().toString().c_str());

    imuUdp.beginPacket(pcIP, statusPort);
    imuUdp.write((uint8_t*)statusPacket, strlen(statusPacket));
    imuUdp.endPacket();
    Serial.println(statusPacket);
  }
}
