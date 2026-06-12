# Camping Bot Power Stability Checklist

## Symptom

The ESP32 sometimes does not drive the robot when powered through VIN, but starts
working again after connecting it to the PC by USB and then returning to VIN.

This usually means one of these happened:

- ESP32 was reset by USB serial connection.
- USB provided a more stable 5 V supply or ground reference.
- VIN or GND wiring has intermittent contact.
- Motor startup caused a voltage dip or electrical noise.
- Wi-Fi/UDP state recovered only after reboot.

## Recommended Fix Order

### 1. Add A Real Reset Method

- Add a small push button between `EN` and `GND` on the ESP32.
- When the robot gets stuck, press reset instead of plugging USB in and out.

This does not fix the root cause, but it removes the USB ritual.

### 2. Improve 5 V Supply

- Use a buck converter rated with enough current margin.
- Recommended: 5 V 3 A or higher for ESP32, ESP32-S3, MPU6050, and lidar.
- Do not power motors from the same weak 5 V rail.
- Keep L298N motor power and logic power wiring clean and secure.

### 3. Add Capacitors Near Controllers

Add capacitors close to the ESP32/ESP32-S3 5 V and GND pins:

- 470 uF to 1000 uF electrolytic capacitor
- 0.1 uF ceramic capacitor

These help absorb short voltage dips when motors or Wi-Fi current spikes happen.

### 4. Check Common Ground

All grounds must be connected:

- battery negative
- buck converter GND
- ESP32 GND
- ESP32-S3 GND
- L298N GND
- sensor GND

Bad ground can look like a communication problem even when the LED is on.

### 5. Measure Voltage While Driving

Measure ESP32 VIN-to-GND while:

- idle
- servo moving
- motors starting
- both motors driving

If VIN drops below about 4.7 V during motor startup, expect Wi-Fi or UDP trouble.

### 6. Upload Status Heartbeat Firmware

Use the firmware snapshot with ESP32 status heartbeat.

Watch:

- `uptime_ms`: resets mean ESP32 rebooted
- `rssi`: weak Wi-Fi if below about `-75`
- `cmd_count`: should increase when commands arrive
- `last_cmd_age_ms`: should stay small while driving

## Practical Recommendation

For now:

1. Add an ESP32 EN reset button.
2. Replace loose VIN/GND jumper wires with locked connectors or soldered leads.
3. Add 470 uF or larger capacitor near ESP32 5 V input.
4. Upload heartbeat firmware when convenient.
5. Then test again without USB power.

## LDS14 Lidar Power Notes

If the LDS14 disappears and returns only after unplugging/replugging the 5 V or
VIN wire, treat it as a power/contact/reset problem before debugging ROS.

Recommended changes:

- Do not use loose breadboard jumper wires for the lidar 5 V and GND lines.
- Use JST-XH, Dupont housing with locking glue, screw terminal, or soldered
  leads with heat-shrink strain relief.
- Give the lidar/ESP32-S3 a local capacitor near the board:
  - 470 uF electrolytic capacitor between 5 V and GND
  - 0.1 uF ceramic capacitor between 5 V and GND
- Add a reset switch for ESP32-S3 if the board exposes `EN`/`RST`.
- Keep lidar power wires short and separate from motor wires where possible.
- Confirm all grounds are common, but avoid long thin ground paths shared with
  motor current.

Fast diagnosis:

- If `ping 192.168.0.9` fails, ESP32-S3 Wi-Fi/power is down.
- If ping works but UDP 12346 does not arrive, lidar serial/power or ESP32-S3
  firmware path is stuck.
- If raw UDP arrives but `/scan_viz` does not publish, debug `lds14_udp_node`.
