#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char* ssid = "aip2.4GHz";
const char* password = "aip123456";
const char* pcIP = "192.168.0.8";
const int cmdPort = 12347;
const int imuPort = 12348;

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

const int SERVO_CENTER_ANGLE = 90;
const int SERVO_MIN_ANGLE = 50;
const int SERVO_MAX_ANGLE = 130;
const int MOTOR_MIN_PWM = 120;
const unsigned long CMD_TIMEOUT_MS = 500;

// Tune these after a wheel-in-air and floor straight test.
const double LEFT_MOTOR_SCALE = 1.00;
const double RIGHT_MOTOR_SCALE = 1.28;

Servo steerServo;
int lastServoAngle = -1;
unsigned long lastImuTime = 0;
unsigned long lastCmdLogTime = 0;
unsigned long lastCmdTime = 0;
bool stoppedByTimeout = true;

double reqLinear = 0.0;
double reqSteering = 0.0;

void setAckermanDrive(double linear_vel, double steering_angle) {
  int servoAngle = SERVO_CENTER_ANGLE + (int)(steering_angle * 180.0 / PI);
  servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

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
  if (now - lastCmdLogTime >= 500) {
    lastCmdLogTime = now;
    Serial.printf("cmd linear=%.3f steering=%.3f servo=%d pwmL=%d pwmR=%d\n",
                  linear_vel, steering_angle, servoAngle, pwmLeft, pwmRight);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n--- Camping Bot ESP32 drive + IMU ---");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected.");
  WiFi.setSleep(false);
  Serial.println(WiFi.localIP());

  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Check wiring.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 ready.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  cmdUdp.begin(cmdPort);
  imuUdp.begin(imuPort);
  lastCmdTime = millis();

  steerServo.setPeriodHertz(50);
  steerServo.attach(SERVO_PIN, 500, 2500);
  steerServo.write(SERVO_CENTER_ANGLE);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  while (cmdUdp.parsePacket()) {
    char packetBuffer[64];
    int len = cmdUdp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = 0;
      int parsed = sscanf(packetBuffer, "%lf,%lf", &reqLinear, &reqSteering);
      if (parsed == 2) {
        lastCmdTime = millis();
        stoppedByTimeout = false;
        setAckermanDrive(reqLinear, reqSteering);
      } else {
        Serial.printf("Bad cmd packet: %s\n", packetBuffer);
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

  if (currentTime - lastImuTime >= 50) {
    lastImuTime = currentTime;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    char imuPacket[128];
    sprintf(imuPacket, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
            a.acceleration.x, a.acceleration.y, a.acceleration.z,
            g.gyro.x, g.gyro.y, g.gyro.z);

    imuUdp.beginPacket(pcIP, imuPort);
    imuUdp.write((uint8_t*)imuPacket, strlen(imuPacket));
    imuUdp.endPacket();
  }
}
