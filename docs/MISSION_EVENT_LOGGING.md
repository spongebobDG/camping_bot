# Mission Event Logging

`mission_event_logger` saves important robot events to CSV during field tests.

It is meant for debugging questions like:

- Why did the robot stop?
- Was a hazard detected?
- Did the ESP32 reboot?
- Did the camera go offline?
- What command was actually executed?

## Log Location

Default directory:

```text
~/.ros/camping_bot_logs
```

Each launch creates a new file:

```text
mission_events_YYYYMMDD_HHMMSS.csv
```

## CSV Columns

```text
wall_time,ros_time_sec,source,value
```

Example:

```text
2026-06-15T21:40:12.300,1781257000.123456,mission/level,WARN
2026-06-15T21:40:13.100,1781257000.923456,camping_robot/hazard,OBSTACLE_CLOSE front=0.31m
2026-06-15T21:40:14.020,1781257001.843456,cmd_vel_executed,linear=0.320; angular=0.245
```

## Logged Topics

String topics are saved only when the text changes:

```text
/mission/status
/mission/level
/mission/command
/mission/task_status
/mission/assistance_request
/mission/elevator_status
/camping_robot/hazard
/esp32/status
/camera/status
```

Drive commands are saved from:

```text
/cmd_vel_executed
```

Repeated drive commands are throttled so the CSV does not grow too quickly.

## Run

The logger starts automatically in:

```bash
ros2 launch camping_robot_bringup camping_robot_udp.launch.py
```

To run only the logger:

```bash
ros2 run camping_robot_bringup mission_event_logger
```

## Parameters

Configured in:

```text
software/ros2_ws/src/camping_robot_bringup/config/robot.yaml
```

```yaml
mission_event_logger:
  ros__parameters:
    log_dir: "~/.ros/camping_bot_logs"
    flush_period_sec: 1.0
    cmd_min_interval_sec: 0.5
    cmd_change_epsilon: 0.01
```

## Field Test Tip

After a test run, open the newest CSV and check the last few lines first.

```bash
ls -lt ~/.ros/camping_bot_logs | head
tail -n 30 ~/.ros/camping_bot_logs/mission_events_*.csv
```
