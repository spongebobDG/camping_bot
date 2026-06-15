# Mission Supervisor

`mission_supervisor` combines robot health, camera, hazard, goal, and patrol
state into one mission-level status.

## Output Topics

```bash
ros2 topic echo /mission/status
ros2 topic echo /mission/level
```

Levels:

- `OK`
- `WARN`
- `DANGER`

Example:

```text
level=OK; hazard=OK; camera=True; esp32_rssi=-61; goal=reached; patrol=idle; issues=all_systems_nominal
```

## Inputs

- `/camera/online`
- `/esp32/status`
- `/camping_robot/hazard`
- `/simple_goal/status`
- `/waypoint_patrol/status`
- `/mission/task_status`

## Current Use

Use this topic before starting missions:

```bash
ros2 topic echo /mission/status
```

If nothing appears, first verify that the new package install is sourced:

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

Then check whether the executable exists:

```bash
ros2 run camping_robot_bringup mission_supervisor
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /mnt/c/Projects/Camping_bot/software/ros2_ws/install/local_setup.bash
ros2 topic echo /mission/status --no-daemon
ros2 topic echo /mission/level --no-daemon
```

`mission_supervisor` publishes even if camera, hazard, and ESP32 inputs are
missing. If no message appears, the node is not running or the terminal is using
an old workspace environment.

Useful checks:

```bash
ros2 node list --no-daemon | grep mission
ros2 topic list --no-daemon | grep mission
```

Expected nodes/topics:

```text
/mission_supervisor
/mission/status
/mission/level
```

Start patrol only when:

- `level=OK`
- camera is online
- hazard is OK
- ESP32 status is fresh
- lidar is publishing

## Mission Commands

`mission_command_node` provides one command entry point:

```bash
ros2 topic pub /mission/command std_msgs/msg/String "{data: patrol}" --once
ros2 topic pub /mission/command std_msgs/msg/String "{data: stop}" --once
ros2 topic pub /mission/command std_msgs/msg/String "{data: alert}" --once
```

Watch mode state:

```bash
ros2 topic echo /mission/mode_status --no-daemon
```

Supported command status:

- `patrol`: starts waypoint patrol only when `/mission/level` is `OK`
- `stop`: stops the robot, pauses patrol, and turns the buzzer off
- `alert`: stops the robot and turns the warning buzzer on
- `reset_patrol`: resets waypoint patrol to the first waypoint
- `delivery`, `guide`, `evacuate`: forwarded to `mission_task_manager`
- `return_home`: sends the robot to the configured home location

Destination missions are handled by `mission_task_manager`.
See:

```text
docs/MISSION_TASKS.md
```

For `patrol`, launch the patrol stack, not only the simple-goal stack:

```bash
ros2 launch camping_robot_bringup camping_robot_patrol.launch.py
```

If `/waypoint_patrol` is not running, `/mission/mode_status` reports:

```text
mode=idle; level=OK; event=patrol_unavailable_start_patrol_launch
```

Next behavior layer:

- patrol
- delivery
- guide
- evacuation
- warning
- stop

This should use `/mission/level` to decide whether a command is allowed.
