# esp32_s3_camera_v1

Camera firmware for Camping_bot.

Board:

- Seeed Studio XIAO ESP32-S3 Sense

## Upload Target

Open:

```text
esp32_s3_camera_v1.ino
```

Arduino IDE board settings:

- Board: `XIAO_ESP32S3`
- PSRAM: enabled if the menu exists
- USB CDC On Boot: enabled if Serial does not appear
- Upload Mode: default UART/USB

## Target Behavior

- Connect to Wi-Fi `aip2.4GHz`.
- Disable Wi-Fi sleep.
- Print camera IP to Serial.
- Serve MJPEG stream at `/stream`.
- Suggested IP: `192.168.0.11`.

Endpoints:

- `/`
- `/status`
- `/capture`
- `/stream`

## ROS2 Side

Update:

```text
software/ros2_ws/src/camping_robot_bringup/config/robot.yaml
```

```yaml
esp32_camera_monitor:
  ros__parameters:
    enabled: true
    stream_url: "http://192.168.0.11/status"
```
