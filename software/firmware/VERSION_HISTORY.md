# Camping_bot Firmware Version History

## 2026-06-12

### `ackerman_esp32_imu_v1`

- Source snapshot from `ackerman_esp32_imu_v5.ino`.
- ESP32 drive and MPU6050 IMU firmware.
- Includes current ROS2 UDP command format: `linear,steering`.
- Includes motor minimum PWM and right motor trim changes used during driving tests.
- Includes command timeout safety logic.
- Includes UDP/Serial status heartbeat for diagnosing ESP32 power and communication stability.

### `esp32_s3_lidar_v1`

- Source snapshot from `esp32-s3_26_06_01_v3.ino`.
- ESP32-S3 LDS14 lidar UDP sender firmware.
- Used with ROS2 `lds14_udp_node` on UDP port `12346`.

### `esp32_s3_camera_v1`

- Seeed Studio XIAO ESP32-S3 Sense MJPEG camera firmware.
- Target stream endpoint: `http://192.168.0.11/stream`.

## Versioning Rule

- Keep Arduino upload sketches in versioned folders under `software/firmware`.
- Use names like `ackerman_esp32_imu_v2`, `esp32_s3_lidar_v2`, and so on.
- When firmware changes, copy the previous version folder, increment the suffix, then edit the new version.
- Leave the old version untouched so Git can show the project history clearly.
