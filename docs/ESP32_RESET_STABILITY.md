# ESP32 Reset and Startup Stability

## Current Symptom

Sometimes the robot does not move when powered from VIN. If the ESP32 is plugged
into the PC by USB and then returned to VIN, it works again.

This usually means the board needed one of these:

- reset
- cleaner 5 V
- better GND reference
- Wi-Fi reconnect
- recovery from brownout or noisy motor startup

## Recommended Hardware Fix

### Add Reset Buttons

Add a button between `EN` and `GND` for:

- motor ESP32
- LDS14 ESP32-S3
- camera ESP32-S3

Then you can reset the board without unplugging USB or power wires.

If the board already has a reset button, use that button. You do not need to
add another one unless the installed robot body makes the onboard button hard to
reach.

Automatic restart can be useful for clear software failures, but for this robot
we keep camera recovery manual while power/contact issues are being debugged.
If the board only works after repeatedly resetting, fix the 5 V/GND wiring and
add local capacitors.

### Improve Power

- Use a 5 V buck converter with enough current margin.
- Use thicker 5 V and GND wires than breadboard jumpers.
- Add capacitors near every ESP32 board.
- Keep motor current wiring away from sensor/ESP32 power wiring.
- Make all grounds common, but avoid running motor current through sensor ground.

### Firmware Stability

Motor ESP32 firmware snapshot `ackerman_esp32_imu_v1` already includes:

- command timeout
- motor stop on stale command
- Wi-Fi sleep disabled
- UDP/Serial heartbeat on port `12349`

Watch it from ROS2:

```bash
ros2 topic echo /esp32/status
```

Important fields:

- `uptime_ms`: if it resets, ESP32 rebooted
- `rssi`: Wi-Fi strength
- `cmd_count`: increases when motor commands arrive
- `last_cmd_age_ms`: small while actively commanding
- `free_heap`: memory health

## Final Startup Checklist

1. Power on robot.
2. Check lidar UDP or `/scan_viz`.
3. Check motor ESP32 heartbeat `/esp32/status`.
4. Check camera `/camera/online`.
5. Set RViz initial pose.
6. Start waypoint/goal mission.
