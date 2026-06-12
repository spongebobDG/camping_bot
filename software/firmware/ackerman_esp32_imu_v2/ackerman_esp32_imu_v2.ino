// Camping_bot firmware snapshot: ackerman_esp32_imu_v2
// Date: 2026-06-12
// Notes: ESP32 drive + MPU6050 IMU + UDP status heartbeat + low-trigger buzzer.
//
// Buzzer wiring for MM-FMD style low-level-trigger module:
//   VCC -> ESP32 3V3 first. Use 5V only if the module input is 3.3V-safe.
//   GND -> ESP32 GND / robot common GND
//   I/O -> GPIO13

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char* ssid = "aip2.4GHz";
const char* password = "aip123456";
const char* pcIP = "192.168.0.8";

const int cmdPort = 12347;
const int imuPort = 12348;
const int statusPort = 12349;

WiFiUDP cmdUdp;
WiFiUDP imuUdp;

Adafruit_MPU6050 mpu;

const double WHEEL_BASE = 0.20;
const double TRACK_WIDTH = 0.15;

const int ENA = 19;
const int IN1 = 2;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 18;
const int ENB = 15;
const int SERVO_PIN = 23;

const int BUZZER_PIN = 13;
const bool BUZZER_LOW_TRIGGER = true;

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
bool buzzerOn = false;

double reqLinear = 0.0;
double reqSteering = 0.0;

void setBuzzer(bool enabled) {
  buzzerOn = enabled;
  if (BUZZER_LOW_TRIGGER) {
    digitalWrite(BUZZER_PIN, enabled ? LOW : HIGH);
  } else {
    digitalWrite(BUZZER_PIN, enabled ? HIGH : LOW);
  }
}

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
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  analogWrite(ENA, pwmLeft);
  analogWrite(ENB, pwmRight);

  unsigned long now = millis();
  if (now - lastDriveLogTime >= 500) {
    lastDriveLogTime = now;
    Serial.printf("drive linear=%.3f steering=%.3f servo=%d pwmL=%d pwmR=%d buzzer=%d\n",
                  linear_vel, steering_angle, servoAngle, pwmLeft, pwmRight, buzzerOn ? 1 : 0);
  }
}

void handleCommandPacket(char* packetBuffer) {
  if (strncmp(packetBuffer, "BUZZER,", 7) == 0) {
    int value = atoi(packetBuffer + 7);
    setBuzzer(value != 0);
    Serial.printf("buzzer=%d\n", buzzerOn ? 1 : 0);
    return;
  }

  double linear = 0.0;
  double steering = 0.0;
  int parsed = sscanf(packetBuffer, "%lf,%lf", &linear, &steering);
  if (parsed == 2) {
    reqLinear = linear;
    reqSteering = steering;
    lastCmdTime = millis();
    stoppedByTimeout = false;
    receivedCmdCount++;
    setAckermanDrive(reqLinear, reqSteering);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  setBuzzer(false);
  delay(1000);

  Serial.println("\n--- [MCU 1] Ackermann drive + IMU + buzzer v2 boot ---");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[OK] Wi-Fi connected.");
  WiFi.setSleep(false);
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("[ERROR] MPU6050 not found. Check SDA/SCL and power.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("[OK] MPU6050 detected.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  cmdUdp.begin(cmdPort);
  imuUdp.begin(imuPort);
  lastCmdTime = millis();

  steerServo.attach(SERVO_PIN);
  steerServo.write(SERVO_CENTER_ANGLE);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  setAckermanDrive(0.0, 0.0);
}

void loop() {
  while (cmdUdp.parsePacket()) {
    char packetBuffer[64];
    int len = cmdUdp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = 0;
      handleCommandPacket(packetBuffer);
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

  if (currentTime - lastImuTime >= 50) {
    lastImuTime = currentTime;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    char imuPacket[128];
    snprintf(imuPacket, sizeof(imuPacket), "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
             a.acceleration.x, a.acceleration.y, a.acceleration.z,
             g.gyro.x, g.gyro.y, g.gyro.z);

    imuUdp.beginPacket(pcIP, imuPort);
    imuUdp.write((uint8_t*)imuPacket, strlen(imuPacket));
    imuUdp.endPacket();
  }

  if (currentTime - lastStatusTime >= 1000) {
    lastStatusTime = currentTime;
    char statusPacket[180];
    snprintf(statusPacket, sizeof(statusPacket),
             "uptime_ms=%lu,rssi=%d,cmd_count=%lu,last_cmd_age_ms=%lu,free_heap=%lu,ip=%s,buzzer=%d",
             currentTime,
             WiFi.RSSI(),
             receivedCmdCount,
             currentTime - lastCmdTime,
             (unsigned long)ESP.getFreeHeap(),
             WiFi.localIP().toString().c_str(),
             buzzerOn ? 1 : 0);

    imuUdp.beginPacket(pcIP, statusPort);
    imuUdp.write((uint8_t*)statusPacket, strlen(statusPacket));
    imuUdp.endPacket();
    Serial.println(statusPacket);
  }
}
