# Camping Bot Navigation Strategy

## Current Problem

The robot is an Ackermann-style vehicle. It cannot rotate in place, so a simple
"drive toward the goal" controller can reach the position but struggle to match
the final direction.

When the robot is close to a wall, stopping too early makes navigation feel
blocked. Stopping too late risks collision. LDS14 also should not be trusted at
extremely short distances below its practical minimum range.

## Recommended Strategy

### 1. General Movement

- Prioritize reaching the goal position.
- Do not force exact final yaw for normal patrol, delivery, or guide movement.
- Use wide approach and swing maneuvers to reduce awkward front/back repetition.

### 2. Close Obstacle Reflex

- Keep forward safety active only at short range.
- Slow down near about 24 cm.
- Stop near about 14 cm.
- If the robot gets very close, about 10 cm, command a short reverse escape.
- Do not use 3 cm as the main threshold because it is too close for lidar,
  braking distance, and mechanical tolerance.

### 3. Precise Alignment

Use a separate precise alignment mode only for tasks such as:

- elevator entrance alignment
- docking
- charging station alignment
- package handoff position

Do not require precise final yaw for every goal.

### 4. Professional Approach

Field robots and autonomous cars usually avoid learning-first navigation for
this level of problem. They normally use:

- Hybrid A* or Smac Hybrid-A* for Ackermann feasible path planning
- Reeds-Shepp paths when forward and reverse are allowed
- Dubins paths when only forward driving is allowed
- Pure Pursuit, Stanley, or MPC for path tracking
- simulation for repeated tuning before real-world tests

Learning can be used later, but it is usually not the first tool for core safety
and path planning.

## Teleop Learning

Recording teleop driving is possible. It can be useful for analysis or later
imitation learning, but it is not recommended as the first solution.

Reasons:

- It needs many examples in many layouts.
- It can learn unsafe behavior from small mistakes.
- It is harder to debug than geometric path planning.
- It still needs a safety layer.

Recommended use of teleop data:

- record good driving sessions
- compare human paths with robot planned paths
- use it later to tune costs, speeds, and turning behavior

## Simulation

Simulation is recommended before field deployment.

Useful simulation goals:

- test Ackermann turning radius
- tune goal follower parameters
- test close obstacle escape
- prepare for Nav2 Smac Hybrid-A*

Simulation should not replace real tests because wheel slip, L298N behavior,
right rear motor drag, and Wi-Fi/power issues are hardware-specific.

## Next Upgrade

The best next major upgrade is Nav2 with an Ackermann-capable planner:

- Smac Hybrid-A* planner
- Reeds-Shepp motion model
- larger turning radius constraints
- Regulated Pure Pursuit or another path follower

Until odometry is improved, keep the simple goal follower as the practical test
controller.
