// Camping_bot firmware snapshot: esp32_s3_camera_v1
// Date: 2026-06-12
// Board: Seeed Studio XIAO ESP32-S3 Sense
// Notes: Wi-Fi MJPEG camera stream for ROS2 camera monitor.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include "esp_http_server.h"

const char* ssid = "aip2.4GHz";
const char* password = "aip123456";
const char* pcIP = "192.168.0.8";
const int cameraStatusPort = 12350;

// Use DHCP by default. Prefer a router DHCP reservation for 192.168.0.11.
const bool USE_STATIC_IP = false;
IPAddress localIP(192, 168, 0, 11);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 0, 1);

// Seeed Studio XIAO ESP32-S3 Sense OV2640 pin map.
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// XIAO ESP32-S3 Sense camera expansion board power enable.
const int CAMERA_POWER_GPIO = 21;

httpd_handle_t cameraServer = nullptr;
WiFiUDP statusUdp;
unsigned long lastStatusTime = 0;

static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=frame";
static const char* STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char* STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (USE_STATIC_IP) {
    WiFi.config(localIP, gateway, subnet, dns);
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("[OK] Wi-Fi connected");
  Serial.print("Camera IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream");
}

bool initCamera() {
  pinMode(CAMERA_POWER_GPIO, OUTPUT);
  digitalWrite(CAMERA_POWER_GPIO, HIGH);
  delay(100);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[ERR] Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_vflip(sensor, 1);
    sensor->set_hmirror(sensor, 1);
  }

  Serial.println("[OK] Camera initialized");
  return true;
}

esp_err_t rootHandler(httpd_req_t* req) {
  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<title>Camping Bot Camera</title></head><body>";
  html += "<h2>Camping Bot ESP32-S3 Camera</h2>";
  html += "<p><a href='/stream'>MJPEG Stream</a></p>";
  html += "<p><a href='/capture'>Single Capture</a></p>";
  html += "<p><a href='/status'>Status</a></p>";
  html += "</body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html.c_str(), html.length());
}

esp_err_t statusHandler(httpd_req_t* req) {
  char text[192];
  snprintf(text, sizeof(text),
           "uptime_ms=%lu,rssi=%d,free_heap=%lu,ip=%s\n",
           millis(),
           WiFi.RSSI(),
           (unsigned long)ESP.getFreeHeap(),
           WiFi.localIP().toString().c_str());
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, text, strlen(text));
}

esp_err_t captureHandler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

esp_err_t streamHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);

  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[WARN] Camera frame failed");
      return ESP_FAIL;
    }

    char partHeader[64];
    size_t headerLen = snprintf(partHeader, sizeof(partHeader), STREAM_PART, fb->len);

    esp_err_t res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, partHeader, headerLen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);

    if (res != ESP_OK) {
      break;
    }
    delay(30);
  }

  return ESP_OK;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;

  if (httpd_start(&cameraServer, &config) != ESP_OK) {
    Serial.println("[ERR] Failed to start camera HTTP server");
    return;
  }

  httpd_uri_t rootUri = {
      .uri = "/", .method = HTTP_GET, .handler = rootHandler, .user_ctx = nullptr};
  httpd_uri_t statusUri = {
      .uri = "/status", .method = HTTP_GET, .handler = statusHandler, .user_ctx = nullptr};
  httpd_uri_t captureUri = {
      .uri = "/capture", .method = HTTP_GET, .handler = captureHandler, .user_ctx = nullptr};
  httpd_uri_t streamUri = {
      .uri = "/stream", .method = HTTP_GET, .handler = streamHandler, .user_ctx = nullptr};

  httpd_register_uri_handler(cameraServer, &rootUri);
  httpd_register_uri_handler(cameraServer, &statusUri);
  httpd_register_uri_handler(cameraServer, &captureUri);
  httpd_register_uri_handler(cameraServer, &streamUri);

  Serial.println("[OK] Camera HTTP server started");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("--- Camping Bot XIAO ESP32-S3 Sense Camera v1 ---");

  connectWiFi();
  statusUdp.begin(cameraStatusPort);

  if (!initCamera()) {
    Serial.println("[ERR] Camera init failed. Check camera cable/power, then press the board reset button.");
    return;
  }

  startCameraServer();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WARN] Wi-Fi disconnected. Check Wi-Fi/power, then press reset if needed.");
    delay(1000);
    return;
  }

  unsigned long now = millis();
  if (now - lastStatusTime >= 3000) {
    lastStatusTime = now;
    char statusPacket[192];
    snprintf(statusPacket, sizeof(statusPacket),
             "camera_uptime_ms=%lu,rssi=%d,free_heap=%lu,ip=%s,stream=http://%s/stream",
             now,
             WiFi.RSSI(),
             (unsigned long)ESP.getFreeHeap(),
             WiFi.localIP().toString().c_str(),
             WiFi.localIP().toString().c_str());
    statusUdp.beginPacket(pcIP, cameraStatusPort);
    statusUdp.write((uint8_t*)statusPacket, strlen(statusPacket));
    statusUdp.endPacket();
    Serial.println(statusPacket);
  }
}
