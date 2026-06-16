# Control Panel

The control panel is a local web UI for the camping robot.

It shows:

- saved map
- robot pose from AMCL
- latest goal pose
- ESP32-S3 camera stream
- mission status
- hazard state
- battery state
- current patrol/task state
- elevator assist state

The layout is map-first. RViz is still useful for debugging, but normal field
operation can use this panel to see the saved map, robot pose, and goal.

The camera is intentionally small and placed under the controls. The map is the
main view.

Extra commands live in the right-side tab window:

- `Mission`
- `Assist`
- `Elevator`
- `Details`

## Map Controls

The map panel has three modes:

- `Goal`: click or drag on the map to publish `/goal_pose`
- `Estimate`: click or drag on the map to publish `/initialpose`
- `Move`: drag the map without sending robot commands
- `Inspect`: view the map without sending commands

Click sets the position. Drag sets both position and direction.

Use the mouse wheel to zoom the map. Press `Fit` to return to the full-map
view.

The web panel reads the saved map from:

```bash
export CAMPING_MAP_YAML=/home/spbdg/maps/camping_test_map.yaml
```

If this variable is not set, that same path is used as the default.

The map and camera panels each have a `Full` button for fullscreen viewing.

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

Change saved map:

```bash
export CAMPING_MAP_YAML=/home/spbdg/maps/camping_test_map.yaml
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
