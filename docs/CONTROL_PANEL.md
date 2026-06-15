# Control Panel

The control panel is a small local web UI for the camping robot.

It shows:

- ESP32-S3 camera stream
- mission status
- hazard state
- current patrol/task state

It can send:

- patrol
- delivery
- guide
- evacuate
- return_home
- elevator
- alert
- stop
- next waypoint
- reset patrol
- obstacle assistance decisions
- elevator assisted-flow decisions

## Run

Start the robot ROS2 stack first:

```bash
cd /mnt/c/Projects/Camping_bot/software/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch camping_robot_bringup camping_robot_nav2_patrol.launch.py
```

In another terminal:

```bash
cd /mnt/c/Projects/Camping_bot
source /opt/ros/humble/setup.bash
source /mnt/c/Projects/Camping_bot/software/ros2_ws/install/local_setup.bash
python3 software/control_panel/camping_control_panel.py
```

Open:

```text
http://localhost:8088
```

## Environment Variables

Change port:

```bash
export CAMPING_PANEL_PORT=8090
```

Change camera stream URL:

```bash
export CAMPING_CAMERA_STREAM_URL=http://192.168.0.11/stream
```

## Notes

This panel uses `ros2 topic echo --once` and `ros2 topic pub` internally, so it
should be started from a terminal where ROS2 and the Camping Bot workspace have
already been sourced.

When the robot is blocked by an obstacle during a mission, the Assistance panel
shows the request from `/mission/assistance_request`.

Decision buttons publish to `/mission/decision`:

- `Wait`
- `Retry`
- `Next`
- `Alert`
- `Stop`

The Elevator panel publishes to `/mission/elevator_decision`:

- `Call`
- `Entered`
- `Floor Selected`
- `Exited`
- `Complete`
- `Cancel`

For field testing, keep a separate terminal ready with:

```bash
ros2 topic pub /mission/command std_msgs/msg/String "{data: stop}" --once
```
