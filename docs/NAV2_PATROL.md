# Nav2 Patrol

Use Nav2 patrol when the robot should route around objects already present in
the saved map.

## Why Simple Patrol Hit Obstacles

`camping_robot_patrol.launch.py` uses `simple_goal_follower`.

That node drives toward the waypoint direction. It can stop for close lidar
obstacles through `lidar_safety_stop`, but it does not read the map and compute
a path around black occupied cells.

So:

- mapped wall or object: simple patrol may still point at it
- new obstacle: simple patrol slows/stops/beeps, but does not plan a detour

## What Nav2 Adds

`camping_robot_nav2_patrol.launch.py` sends each waypoint to Nav2
`NavigateToPose`.

Nav2 uses:

- saved map for known obstacles
- global costmap for map-based planning
- local costmap for lidar obstacles
- planner for an around-the-obstacle path
- controller for following that path

## Launch

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch camping_robot_bringup camping_robot_nav2_patrol.launch.py
```

In RViz:

1. Set `Map` display durability to `Transient Local`.
2. Set the robot initial pose with `2D Pose Estimate`.
3. Confirm lidar points match the map.

Start:

```bash
ros2 topic pub /mission/command std_msgs/msg/String "{data: patrol}" --once
```

Stop:

```bash
ros2 topic pub /mission/command std_msgs/msg/String "{data: stop}" --once
```

Watch:

```bash
ros2 topic echo /waypoint_patrol/status --no-daemon
ros2 topic echo /cmd_vel_raw --no-daemon
ros2 topic echo /plan --no-daemon
```

## Notes For Ackermann Steering

The Nav2 controller is set to Regulated Pure Pursuit. This is closer to car-like
path following than a differential-drive controller that tries to rotate in
place.

If the robot still tries to cut corners too tightly:

- increase costmap `inflation_radius`
- reduce `desired_linear_vel`
- increase `lookahead_dist`
- add intermediate waypoints around sharp turns
