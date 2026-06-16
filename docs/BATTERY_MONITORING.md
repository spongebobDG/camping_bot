# Battery Monitoring

The robot now has a ROS-side battery monitor.

It can read voltage from either:

- `/battery/voltage` as `std_msgs/msg/Float32`
- `/esp32/status` text containing `battery_v=12.1`

The monitor publishes:

```text
/battery/status
```

Example output:

```text
OK voltage=11.85V cell=3.95V source=esp32_status age=0.2s
LOW voltage=10.65V cell=3.55V source=voltage_topic age=0.1s
CRITICAL voltage=9.70V cell=3.23V source=esp32_status age=0.1s
```

## Current Thresholds

For a 3S lithium-ion pack:

```yaml
battery_monitor:
  ros__parameters:
    cells: 3
    warn_voltage: 10.8
    critical_voltage: 9.9
```

Meaning:

- `OK`: above 10.8V
- `LOW`: 10.8V or below
- `CRITICAL`: 9.9V or below

These values are conservative. Adjust them after checking the real battery voltage under motor load.

## Quick Test Without Wiring

You can simulate battery voltage:

```bash
ros2 topic pub /battery/voltage std_msgs/msg/Float32 "{data: 11.7}" --once
ros2 topic echo /battery/status --no-daemon
```

Low-voltage test:

```bash
ros2 topic pub /battery/voltage std_msgs/msg/Float32 "{data: 10.5}" --once
ros2 topic echo /mission/status --no-daemon
```

## Recommended ESP32 Wiring

Do not connect a 3S battery directly to an ESP32 ADC pin.

Use a voltage divider:

```text
Battery + ---- R1 ----+---- ESP32 ADC pin
                      |
                     R2
                      |
Battery - / GND ------+
```

Recommended starting values:

```text
R1 = 100k ohm
R2 = 27k ohm
```

This scales 12.6V down to about 2.68V, which is safer for ESP32 ADC input.

Use an ADC1 pin on the ESP32, for example:

```text
GPIO34
```

Important:

- Battery negative, ESP32 GND, buck converter GND, and L298N GND must share common ground.
- Add a small capacitor near the ADC input if the reading jumps a lot.
- Measure with a multimeter before connecting the ADC pin.

## Future Firmware Change

The ESP32 status heartbeat should eventually include:

```text
battery_v=12.10
```

Then ROS will pick it up automatically from `/esp32/status`.
