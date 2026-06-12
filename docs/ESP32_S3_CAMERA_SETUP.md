# ESP32-S3 Camera Setup

## Goal

Add an ESP32-S3 camera as a Wi-Fi video source before enabling higher-level
hazard and warning behaviors.

Recommended first target:

- ESP32-S3 CAM connects to Wi-Fi.
- It prints its IP address on Serial.
- It serves an MJPEG stream such as `http://<camera_ip>/stream`.
- ROS2 monitors whether the stream is alive.

## Why Start With MJPEG

MJPEG over HTTP is simple and easy to debug:

- Open it in a browser first.
- Check it from ROS2 second.
- Add computer vision later.

Do not start with object detection or ROS image processing until the basic
stream is stable.

## ROS2 Camera Monitor

The project now includes `esp32_camera_monitor`.

Configured in:

```text
software/ros2_ws/src/camping_robot_bringup/config/robot.yaml
```

Default:

```yaml
esp32_camera_monitor:
  ros__parameters:
    enabled: false
    stream_url: "http://192.168.0.11/stream"
```

After the camera IP is fixed, update it:

```yaml
esp32_camera_monitor:
  ros__parameters:
    enabled: true
    stream_url: "http://<camera_ip>/status"
```

Watch status:

```bash
ros2 topic echo /camera/status
ros2 topic echo /camera/online
```

ROS health checks use UDP heartbeat on port `12350`.

Use `/stream` for viewing video in a browser. The MJPEG stream can stay open for
a long time, so it is not ideal as a heartbeat check.

## Hardware Recommendations

- Power ESP32-S3 CAM from a stable 5 V rail.
- Do not share a thin jumper path with motor current.
- Add a local capacitor near ESP32-S3 CAM:
  - 470 uF or 1000 uF electrolytic between 5 V and GND
  - 0.1 uF ceramic between 5 V and GND
- Use short, secure 5 V and GND wires.
- Add an EN/RST reset button if the board exposes it.

## Wiring

For the Seeed Studio XIAO ESP32-S3 Sense:

```text
5 V buck converter +  -> XIAO 5V pin
5 V buck converter -  -> XIAO GND pin
Robot common GND      -> XIAO GND pin
```

Use the pins labeled `5V` and `GND` on the XIAO board silkscreen/header.

Do not power the camera board from a random GPIO pin.

If powering from USB-C during testing, do not also feed another unstable 5 V
source into the 5V pin at the same time. For robot use, use the regulated 5 V
buck converter and common ground.

Recommended physical wiring:

- soldered wire, JST connector, or screw terminal
- short 5 V/GND path
- capacitor placed physically close to the XIAO 5V/GND pins

## Firmware

Camera firmware is now prepared under:

```text
software/firmware/esp32_s3_camera_v1/
```

Target board:

- Seeed Studio XIAO ESP32-S3 Sense

Open and upload:

```text
software/firmware/esp32_s3_camera_v1/esp32_s3_camera_v1.ino
```

Expected Serial output:

```text
--- Camping Bot XIAO ESP32-S3 Sense Camera v1 ---
[OK] Wi-Fi connected
Camera IP: 192.168.0.xx
Stream URL: http://192.168.0.xx/stream
[OK] Camera initialized
[OK] Camera HTTP server started
camera_uptime_ms=...,rssi=...,free_heap=...,ip=...,stream=http://.../stream
```

## Suggested IP Plan

Current devices:

- PC Wi-Fi: `192.168.0.8`
- LDS14 ESP32-S3: `192.168.0.9`
- motor ESP32: `192.168.0.10`

Suggested camera:

- ESP32-S3 CAM: `192.168.0.11`

Use router DHCP reservation if possible.

## Reset Button vs Automatic Restart

The onboard reset button is useful and you can press it when the camera stops.
But it is not the final fix.

Recommended approach:

- Use the physical reset button for manual recovery.
- Fix power/contact so reset is rarely needed.

The camera firmware does not automatically restart itself. If Wi-Fi or camera
initialization fails, check power/contact and press the onboard reset button.

Do not add a circuit that resets the ESP32-S3 repeatedly on every robot power
noise event. That can make the camera unstable. A better hardware fix is stable
5 V, solid GND, short wires, and capacitors near the board.
