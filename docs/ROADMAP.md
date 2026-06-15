# Camping Bot Roadmap

## Current Baseline

- ROS2 Humble on WSL Ubuntu 22.04.
- ESP32 motor and IMU UDP bridge is working.
- ESP32-S3 LDS14 lidar UDP bridge is working.
- RViz TF, lidar view, and saved map loading are working.
- SLAM mapping is usable.
- Temporary command-based odometry is active.
- Basic goal follower exists for early map-based driving tests.

## Stage 1: Stable Bringup

- Confirm `/imu/data_raw` stays near 20 Hz.
- Confirm `/scan` stays near 15 Hz.
- Confirm `/cmd_vel_executed` appears when teleop or goal commands are sent.
- Upload ESP32 status heartbeat firmware when ready.
- Use `robot_health_monitor` to diagnose sensor, command, and ESP32 status.

## Stage 2: Map Workflow

- Run mapping launch.
- Drive slowly with teleop.
- Save a clean map.
- Confirm RViz Map display uses `Transient Local` durability.
- Keep map files under a versioned folder such as `maps/v1`.

## Stage 3: Localization Workflow

- Launch saved map with AMCL.
- Set initial pose in RViz.
- Confirm `/amcl_pose` publishes.
- Confirm TF chain is available:
  - `map -> odom -> base_link -> laser`
- Test short movement and confirm robot pose follows the real robot.

## Stage 4: Simple Goal Driving

- Use `camping_robot_simple_goal.launch.py`.
- Set initial pose.
- Send a nearby goal in RViz.
- Verify `/cmd_vel_raw`, `/cmd_vel`, and `/cmd_vel_executed`.
- Tune:
  - `max_linear_mps`
  - `max_angular_radps`
  - `heading_kp`
  - `goal_tolerance_m`

## Stage 5: Hardware Reliability

- Fix or replace the stiff right rear motor when possible.
- Improve 5V wiring and connector reliability.
- Add common ground checks between battery, buck converter, ESP32, ESP32-S3, and L298N.
- Test voltage drop while motors start.
- Keep USB power disconnected during final field tests unless intentionally used for debugging.

## Stage 6: Safety Layer

- Re-enable forward safety after lidar direction and false-positive zones are confirmed.
- Tune:
  - `front_angle_deg`
  - `ignore_distance_below_m`
  - `stop_distance_m`
  - `slow_distance_m`
- Add warning buzzer output through ESP32.
- Add emergency stop command.

## Stage 7: Navigation Upgrade

- Replace or improve temporary `cmd_vel_odom`.
- Options:
  - Tune simple goal follower for short-range movement.
  - Add better IMU yaw integration.
  - Add visual/lidar odometry if stable.
  - Move toward Nav2 once odometry is reliable enough.
- Added Nav2 patrol launch with Regulated Pure Pursuit for map-based routing.
- Continue tuning:
  - costmap inflation radius
  - robot radius
  - lookahead distance
  - low-speed Ackermann path following

## Stage 8: Camping Robot Features

- Danger detection:
  - close obstacle
  - blocked path
  - unusual motion or tilt
  - low voltage
- User request modes:
  - delivery
  - guide
  - evacuation
  - warning buzzer
  - elevator interaction
- Added first destination-task layer for delivery, guide, evacuation, and return-home.
- Elevator interaction remains future work and needs building/elevator-specific hardware or user confirmation flow.
- Added a local web command UI for camera, status, and mission buttons.

## Stage 9: Mission Supervisor

- Combine camera, ESP32, hazard, goal, and patrol state.
- Publish `/mission/status`.
- Publish `/mission/level`.
- Use mission level before starting patrol, delivery, guide, evacuation, or buzzer behaviors.

## Stage 10: User Command Layer

- Add commands:
  - patrol
  - delivery
  - guide
  - evacuation
  - warning
  - stop
- Gate commands using `/mission/level`.
- Added `/mission/command`.
- Added `/mission/task_command`.
- Added local web control panel.

## Recommended Next Step

The next engineering target is Nav2 field validation and mission coordinates:

1. Put real coordinates in `mission_locations.yaml`.
2. Launch `camping_robot_nav2_patrol.launch.py`.
3. Set initial pose.
4. Enable RViz displays for Plan, Global Costmap, and Local Costmap.
5. Test patrol, delivery, guide, evacuation, and return-home.
6. Tune Nav2 until mapped obstacles are routed around reliably.
