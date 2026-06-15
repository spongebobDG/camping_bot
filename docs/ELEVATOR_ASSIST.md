# Elevator Assist

Elevator movement is implemented as an assisted workflow.

The robot can drive to the configured elevator waiting point, but it does not
press elevator buttons or detect doors by itself yet. A user confirms each
stage from the control panel.

## Location

Set the elevator waiting point in:

```text
software/ros2_ws/src/camping_robot_bringup/config/mission_locations.yaml
```

```yaml
locations:
  elevator_wait:
    x: 0.0
    y: 0.0
    yaw: 0.0
```

Use RViz `2D Goal Pose` or `waypoint_recorder` to collect the real coordinate.

## Start

From terminal:

```bash
ros2 topic pub /mission/command std_msgs/msg/String "{data: elevator}" --once
```

or press `Elevator` in the local control panel.

The robot sends a destination mission to `elevator_wait`.

## Status

Watch:

```bash
ros2 topic echo /mission/elevator_status --no-daemon
```

Phases:

- `idle`
- `navigating_to_elevator`
- `waiting_for_elevator`
- `inside_elevator`
- `riding`
- `exited`
- `blocked`

## User Decisions

Send decisions to:

```text
/mission/elevator_decision
```

Commands:

```bash
ros2 topic pub /mission/elevator_decision std_msgs/msg/String "{data: call}" --once
ros2 topic pub /mission/elevator_decision std_msgs/msg/String "{data: entered}" --once
ros2 topic pub /mission/elevator_decision std_msgs/msg/String "{data: floor_selected}" --once
ros2 topic pub /mission/elevator_decision std_msgs/msg/String "{data: exited}" --once
ros2 topic pub /mission/elevator_decision std_msgs/msg/String "{data: complete}" --once
ros2 topic pub /mission/elevator_decision std_msgs/msg/String "{data: cancel}" --once
```

Control panel buttons:

- `Call`: user has called or requested the elevator
- `Entered`: robot is inside the elevator
- `Floor Selected`: target floor has been selected
- `Exited`: robot has exited the elevator
- `Complete`: finish elevator workflow
- `Cancel`: stop/cancel the elevator workflow

## Future Hardware Options

Later versions can add:

- elevator button actuator
- AprilTag/QR marker near elevator doors
- door open detection from camera
- floor selection integration
- voice or phone confirmation
