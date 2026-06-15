# Assistance Flow

`mission_assistance_node` handles situations where the robot is on a mission but
cannot continue because an obstacle stays in front of it.

## When It Triggers

The node watches:

- `/camping_robot/hazard`
- `/mission/mode_status`
- `/mission/task_status`

If a mission is active and `OBSTACLE_CLOSE` or `OBSTACLE_CRITICAL` remains for
more than `blocked_after_sec`, it publishes:

```text
/mission/assistance_request
```

Example:

```text
blocked_obstacle; choices=wait,retry,next,stop,alert; hazard=OBSTACLE_CLOSE front=0.35m
```

## User Decisions

Send decisions to:

```text
/mission/decision
```

Commands:

```bash
ros2 topic pub /mission/decision std_msgs/msg/String "{data: wait}" --once
ros2 topic pub /mission/decision std_msgs/msg/String "{data: retry}" --once
ros2 topic pub /mission/decision std_msgs/msg/String "{data: next}" --once
ros2 topic pub /mission/decision std_msgs/msg/String "{data: stop}" --once
ros2 topic pub /mission/decision std_msgs/msg/String "{data: alert}" --once
```

Meaning:

- `wait`: keep waiting until the obstacle clears
- `retry`: retry the last active mission command
- `next`: skip to the next patrol waypoint
- `stop`: cancel mission and stop
- `alert`: stop and turn on buzzer

## Control Panel

The local control panel shows the assistance request and provides decision
buttons.

```bash
python3 software/control_panel/camping_control_panel.py
```

Open:

```text
http://localhost:8088
```

## Parameters

Configured in `robot.yaml`:

```yaml
mission_assistance_node:
  ros__parameters:
    blocked_after_sec: 3.0
    clear_after_sec: 1.5
    publish_hz: 1.0
```
