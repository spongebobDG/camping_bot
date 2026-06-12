# Camping Robot ROS2 Workspace

ROS2 Humble workspace for the camping surveillance robot.

## Ubuntu CLI entry

Open Ubuntu 22.04 WSL and run:

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch camping_robot_bringup camping_robot_udp.launch.py
```

## Current network map

- ESP32 drive/IMU
  - Receives drive command UDP on `12347`
  - Sends IMU UDP to PC on `12348`
- ESP32-S3 LDS14 lidar bridge
  - Sends bundled LDS14 packets to PC on `12346`

Update IP addresses in:

```text
src/camping_robot_bringup/config/robot.yaml
```

The ESP32 firmware reference with motor trim is stored at:

```text
../firmware/ackermann_esp32_drive_imu/ackermann_esp32_drive_imu.ino
```

Tune `RIGHT_MOTOR_SCALE` if the right side is weaker than the left side.

## Topics

- `/scan`: reliable `sensor_msgs/LaserScan` for RF2O/SLAM
- `/scan_fast`: best-effort `sensor_msgs/LaserScan` for responsive RViz display
- `/scan_viz`: reliable `sensor_msgs/LaserScan` for RViz compatibility
- `/imu/data_raw`: `sensor_msgs/Imu` from MPU6050 UDP packets
- `/cmd_vel_raw`: input `geometry_msgs/Twist` before the lidar safety filter
- `/cmd_vel`: filtered drive command converted to ESP32 `linear,steering`
- `/cmd_vel_executed`: command actually sent to ESP32, converted back to `Twist` for temporary odometry

## Quick checks

```bash
ros2 topic echo /imu/data_raw
ros2 topic echo /scan
ros2 topic echo /scan_fast
ros2 topic echo /scan_viz
ros2 topic pub /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 2
```

## RViz2 check

```bash
rviz2
```

Set `Fixed Frame` to `base_link`, then add:

- `/scan_viz` as `LaserScan`
- `/imu/data_raw` as `Imu`
- `TF`

The launch file publishes fixed transforms:

- `base_link -> laser`: `x=0.18`, `y=0.00`, `z=0.16`
- `base_link -> imu_link`: `x=0.07`, `y=0.00`, `z=0.08`

The lidar yaw is currently corrected by `0.7854rad`.
`mirror_scan: true` is enabled because front/back were correct but left/right appeared swapped in RViz.
Adjust the transform numbers in `launch/camping_robot_udp.launch.py` after measuring the real sensor positions.

## Wheel smoke test

Lift the wheels before running this.

Terminal 1:

```bash
ros2 launch camping_robot_bringup camping_robot_udp.launch.py
```

Terminal 2:

```bash
ros2 run camping_robot_bringup drive_smoke_test
```

The test publishes a very slow `/cmd_vel_raw` for 2 seconds, then sends stop commands.

Steering smoke test:

```bash
ros2 run camping_robot_bringup drive_smoke_test --ros-args -p linear_mps:=0.08 -p angular_radps:=0.8
```

Manual steering command:

```bash
ros2 topic pub /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.08}, angular: {z: 0.8}}" -r 2
```

Teleop should publish to `/cmd_vel_raw`, not directly to `/cmd_vel`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw -p repeat_rate:=10.0 -p key_timeout:=0.6
```

For this Ackermann robot:

- `j` / `l`: steering servo test while stopped
- `u` / `o`: forward with steering
- `i`: forward
- `,`: reverse
- `m` / `.`: reverse with steering

## Straight trim test

Use this after the wheels, steering, lidar, and IMU are working.

Run bringup in terminal 1:

```bash
ros2 launch camping_robot_bringup camping_robot_mapping_cmd_odom.launch.py
```

Run a short floor test in terminal 2:

```bash
ros2 run camping_robot_bringup straight_trim_test --ros-args --params-file /mnt/c/Projects/Camping_bot/software/ros2_ws/src/camping_robot_bringup/config/robot.yaml
```

The node drives straight for 3 seconds, stops, and prints integrated IMU yaw drift.

Tune these if needed:

```text
src/camping_robot_bringup/config/robot.yaml
```

```yaml
straight_trim_test:
  ros__parameters:
    linear_mps: 0.20
    run_seconds: 3.0
```

Motor trim lives in:

```text
../firmware/ackermann_esp32_drive_imu/ackermann_esp32_drive_imu.ino
```

If the robot physically veers right, increase `RIGHT_MOTOR_SCALE`.
If it veers left, decrease `RIGHT_MOTOR_SCALE` or increase `LEFT_MOTOR_SCALE`.

Current trim note:

```text
0.32m/s, 3s floor test
Before trim: -6.73deg, -7.22deg, -7.76deg yaw drift, veered right
Accepted trim: RIGHT_MOTOR_SCALE = 1.28
After trim: -2.55deg, -3.03deg, -2.26deg yaw drift
Status: good enough for early mapping tests
```

## Lidar safety filter

The launch file starts `lidar_safety_stop`.

Use `/cmd_vel_raw` for manual or navigation commands. The safety filter publishes `/cmd_vel`.

- Current mapping mode has `enable_forward_safety: false`, so front distance is monitored but forward commands are not blocked.
- Current monitor settings: ignore under `0.08m`, stop threshold `0.12m`, slow threshold `0.20m`
- Set `enable_forward_safety: true` later when obstacle-stop behavior is tuned.
- Allows reverse motion by default

Tune these values in:

```text
src/camping_robot_bringup/config/robot.yaml
```

## Mapping

This project uses lidar odometry first, then SLAM:

```text
/scan -> rf2o_laser_odometry -> /odom + odom->base_link
/scan + /odom -> slam_toolbox -> /map + map->odom
```

Install missing packages if needed:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

`rf2o_laser_odometry` is expected from the existing `/home/spbdg/colcon_ws`.

Build and launch mapping:

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
source /home/spbdg/colcon_ws/install/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch camping_robot_bringup camping_robot_mapping.launch.py
```

If RF2O keeps waiting for scans, use temporary command odometry:

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch camping_robot_bringup camping_robot_mapping_cmd_odom.launch.py
```

Command odometry integrates `/cmd_vel`, so it drifts over time. Use it only to unblock early SLAM tests while encoders/RF2O are being fixed.

RViz2:

- Fixed Frame: `map`
- Add `/scan_viz` as `LaserScan`
- Add `/scan` as `LaserScan` only when checking RF2O/SLAM input
- Add `/map` as `Map`
- Add `TF`

Or start the prepared RViz view:

```bash
ros2 launch camping_robot_bringup rviz_mapping.launch.py
```

Save a map:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/camping_test_map
```

Recommended first mapping run:

```text
1. Place the robot in a small open indoor area.
2. Start mapping_cmd_odom launch.
3. Open RViz2 with Fixed Frame = map.
4. Add /scan_viz, /map, and TF.
5. Drive slowly with i, u, o, comma, m, and period keys.
6. Avoid long j/l steering-only commands while mapping.
7. Save the map after the walls look stable.
```

## Localization On A Saved Map

After saving `~/maps/camping_test_map.yaml`, run localization:

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch camping_robot_bringup camping_robot_localization.launch.py map:=/home/spbdg/maps/camping_test_map.yaml
```

RViz2:

```bash
ros2 launch camping_robot_bringup rviz_mapping.launch.py
```

Set the initial pose with RViz `2D Pose Estimate`, then drive slowly with teleop. AMCL should keep the robot pose aligned to the saved map if `/scan_viz`, `/scan`, `/odom`, and TF are stable.

## Nav2 Goal Test

After localization works, launch Nav2:

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch camping_robot_bringup camping_robot_nav2.launch.py map:=/home/spbdg/maps/camping_test_map.yaml
```

Open RViz:

```bash
ros2 launch camping_robot_bringup rviz_mapping.launch.py
```

Steps:

```text
1. Use 2D Pose Estimate first.
2. Confirm /scan_viz aligns with the saved map.
3. Use Nav2 Goal.
4. Keep the goal close for the first test.
```

Nav2 sends velocity commands to `/cmd_vel_raw`, then the safety/bridge pipeline sends them to the ESP32.

## Simple Goal Test

Before full Nav2 tuning, test a small goal follower:

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch camping_robot_bringup camping_robot_simple_goal.launch.py map:=/home/spbdg/maps/camping_test_map.yaml
```

Open RViz:

```bash
ros2 launch camping_robot_bringup rviz_mapping.launch.py
```

Steps:

```text
1. Use 2D Pose Estimate first.
2. Confirm /scan_viz aligns with /map.
3. Click a very close goal using Nav2 Goal or Publish Point/goal tool.
4. Keep the first goal within about 0.5m.
```

This node is not a full obstacle-avoiding navigator. It is only for validating saved-map localization and basic goal-driven motion before tuning Nav2.
