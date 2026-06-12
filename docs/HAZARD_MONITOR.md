# Hazard Monitor

`hazard_monitor` is the first safety/event layer for Camping_bot.

It watches lidar and IMU data, then publishes:

- `/camping_robot/hazard` as `std_msgs/String`
- `/warning_buzzer` as `std_msgs/Bool`

## Hazard Types

Current events:

- `OK`
- `SCAN_STALE`
- `IMU_STALE`
- `OBSTACLE_CLOSE`
- `OBSTACLE_CRITICAL`
- `TILT_WARN`
- `TILT_CRITICAL`

## IMU Tilt Baseline

The IMU can be mounted in different directions, so tilt is not calculated by
assuming that gravity is always on the IMU `z` axis.

At startup, keep the robot still and level for about 2 seconds. The monitor
averages the first IMU samples and saves that gravity direction as the normal
upright pose. After that, tilt means "how far the current gravity direction has
moved away from the startup pose."

## Important Topics

Watch hazard state:

```bash
ros2 topic echo /camping_robot/hazard
```

Watch buzzer command:

```bash
ros2 topic echo /warning_buzzer
```

## Current Thresholds

Configured in:

```text
software/ros2_ws/src/camping_robot_bringup/config/robot.yaml
```

```yaml
hazard_monitor:
  ros__parameters:
    front_angle_deg: 35.0
    close_obstacle_m: 0.45
    critical_obstacle_m: 0.22
    buzzer_on_close_obstacle: true
    tilt_warn_deg: 18.0
    tilt_critical_deg: 28.0
    enable_tilt_detection: true
    tilt_calibration_samples: 40
    tilt_min_accel_mps2: 6.0
    tilt_max_accel_mps2: 14.0
    scan_timeout_sec: 3.0
    imu_timeout_sec: 3.0
```

## Buzzer Hardware Plan

The monitor only publishes `/warning_buzzer` for now.

Next firmware step:

- Add a buzzer pin to the ESP32 motor firmware.
- Listen for a UDP buzzer command from ROS2, or include buzzer state in the
  existing drive/status command path.
- Turn the buzzer on when `/warning_buzzer` is true.

Keep the buzzer command separate from motor stop logic so the robot can warn
even when driving is disabled.

For patrol, `OBSTACLE_CLOSE` intentionally starts earlier than the physical
stop distance. This gives the robot time to beep and slow down before the body
touches an obstacle.
