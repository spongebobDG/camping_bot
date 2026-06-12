# Buzzer Setup

This project uses the ESP32 motor/IMU board to drive the warning buzzer.

## Recommended Buzzer

Use the 3-pin `VCC / I/O / GND` MM-FMD style low-level-trigger buzzer module.

It is easier than a 2-pin buzzer because the module already has the driving
circuit. The ESP32 only needs to send an ON/OFF signal.

## Wiring

Recommended first wiring:

```text
Buzzer VCC -> ESP32 3V3
Buzzer GND -> ESP32 GND
Buzzer I/O -> ESP32 GPIO13
```

Use a common ground with the robot power system.

The module is low-level trigger, so:

- GPIO LOW means buzzer ON.
- GPIO HIGH means buzzer OFF.

If the buzzer is too quiet at 3.3V, use 5V for `VCC` only after confirming the
module input pin is safe for a 3.3V ESP32 GPIO. Some low-trigger modules pull
the input toward VCC, which can put 5V on the ESP32 pin.

## Firmware

Upload the tone-output version first:

```text
software/firmware/ackerman_esp32_imu_v3/ackerman_esp32_imu_v3.ino
```

`v2` used a simple LOW signal for low-trigger active buzzers. If the buzzer
only clicks, use `v3`; it outputs a 2400Hz tone for passive-style modules.

The firmware listens on the existing command UDP port:

```text
192.168.0.10:12347
```

Commands:

```text
BUZZER,1
BUZZER,0
```

The status heartbeat now also includes:

```text
buzzer=0
```

or:

```text
buzzer=1
```

## ROS2

The node `warning_buzzer_udp_bridge` subscribes to:

```text
/warning_buzzer
```

and sends UDP buzzer commands to the ESP32.

Test:

```bash
ros2 topic pub /warning_buzzer std_msgs/msg/Bool "{data: true}" -r 2
```

Stop the buzzer:

```bash
ros2 topic pub /warning_buzzer std_msgs/msg/Bool "{data: false}" --once
```

In normal operation, `hazard_monitor` publishes `/warning_buzzer` automatically
when a critical hazard is detected.

If `v3` still only clicks:

- The module probably needs 5V to make sound.
- Use a 5V active buzzer module that explicitly accepts 3.3V logic input.
- Keep ESP32 GND and buzzer power GND common.
