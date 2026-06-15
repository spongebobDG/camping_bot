# Mission Tasks

`mission_task_manager` runs one-shot destination missions:

- delivery
- guide
- evacuate
- return_home

It uses the same robot stack as patrol. If Nav2 is running, it sends goals to
`NavigateToPose`. If only the simple goal follower is running, it publishes a
`/goal_pose`.

## Location File

Edit mission destination coordinates here:

```text
software/ros2_ws/src/camping_robot_bringup/config/mission_locations.yaml
```

Example:

```yaml
locations:
  home:
    x: 0.0
    y: 0.0
    yaw: 0.0
  delivery_dropoff:
    x: 2.0
    y: 1.0
    yaw: 0.0
  guide_destination:
    x: 4.0
    y: -1.0
    yaw: 1.57
  evacuation_point:
    x: -2.0
    y: 3.0
    yaw: 3.14
```

Use `waypoint_recorder` or RViz `/goal_pose` echo to collect these coordinates.

## Commands

Delivery:

```bash
ros2 topic pub /mission/command std_msgs/msg/String "{data: delivery}" --once
```

Guide:

```bash
ros2 topic pub /mission/command std_msgs/msg/String "{data: guide}" --once
```

Evacuate:

```bash
ros2 topic pub /mission/command std_msgs/msg/String "{data: evacuate}" --once
```

Return home:

```bash
ros2 topic pub /mission/command std_msgs/msg/String "{data: return_home}" --once
```

Stop/cancel:

```bash
ros2 topic pub /mission/command std_msgs/msg/String "{data: stop}" --once
```

## Watch Status

```bash
ros2 topic echo /mission/task_status --no-daemon
ros2 topic echo /mission/status --no-daemon
```

## Safety Rule

The task manager only starts movement when `/mission/level` is `OK` by default.

If `/mission/level` becomes `DANGER`, it cancels the active task, publishes a
stop command, and cancels the simple goal follower.
