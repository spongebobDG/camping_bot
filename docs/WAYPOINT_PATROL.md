# Waypoint Patrol

This is the first mission layer for Camping_bot.

It publishes one `/goal_pose` at a time and waits for
`/simple_goal/status == reached` before sending the next waypoint.

## Edit Waypoints

File:

```text
software/ros2_ws/src/camping_robot_bringup/config/patrol_waypoints.yaml
```

Example:

```yaml
waypoints:
  - name: entrance
    x: 1.0
    y: 0.5
    yaw: 0.0
  - name: tent_zone
    x: 2.0
    y: 1.2
    yaw: 1.57
```

Coordinates are in the saved map frame.

## Record Coordinates From RViz

Start the robot with localization and RViz, then set the initial pose first.

In another terminal, run:

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run camping_robot_bringup waypoint_recorder
```

In RViz, use `2D Goal Pose` and click the patrol point you want to save.
The recorder prints YAML like:

```yaml
  - name: wp_01
    x: 1.234
    y: -0.567
    yaw: 1.570
```

Copy those printed blocks into:

```text
software/ros2_ws/src/camping_robot_bringup/config/patrol_waypoints.yaml
```

Tips:

- `x`, `y` are the position on the saved map.
- `yaw` is the direction the robot should face at that waypoint.
- Start with 2 or 3 nearby points before making a full patrol route.

## Launch

Simple patrol follows each waypoint directly. It does not plan around mapped
obstacles:

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch camping_robot_bringup camping_robot_patrol.launch.py
```

Nav2 patrol uses the saved map, global/local costmaps, and the planner to route
around mapped obstacles:

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch camping_robot_bringup camping_robot_nav2_patrol.launch.py
```

Set the initial pose in RViz first.

## Start, Pause, Reset

Start patrol:

```bash
ros2 topic pub /waypoint_patrol/control std_msgs/msg/String "{data: start}" --once
```

Pause patrol:

```bash
ros2 topic pub /waypoint_patrol/control std_msgs/msg/String "{data: pause}" --once
```

Reset to the first waypoint:

```bash
ros2 topic pub /waypoint_patrol/control std_msgs/msg/String "{data: reset}" --once
```

Skip to the next waypoint:

```bash
ros2 topic pub /waypoint_patrol/control std_msgs/msg/String "{data: next}" --once
```

The mission command node can also forward these commands:

```bash
ros2 topic pub /mission/command std_msgs/msg/String "{data: patrol}" --once
ros2 topic pub /mission/command std_msgs/msg/String "{data: next}" --once
ros2 topic pub /mission/command std_msgs/msg/String "{data: stop}" --once
```

## Watch Status

```bash
ros2 topic echo /waypoint_patrol/status
ros2 topic echo /simple_goal/status
```

Status includes the waypoint index:

```text
goal:wp_02; index=2/6
```
