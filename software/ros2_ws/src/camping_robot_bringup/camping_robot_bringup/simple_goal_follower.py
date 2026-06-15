import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String


class SimpleGoalFollower(Node):
    def __init__(self):
        super().__init__("simple_goal_follower")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("cmd_topic", "/cmd_vel_raw")
        self.declare_parameter("max_linear_mps", 0.28)
        self.declare_parameter("min_linear_mps", 0.18)
        self.declare_parameter("max_angular_radps", 0.75)
        self.declare_parameter("heading_kp", 1.4)
        self.declare_parameter("goal_tolerance_m", 0.20)
        self.declare_parameter("slow_radius_m", 0.70)
        self.declare_parameter("max_heading_for_drive_rad", 1.2)
        self.declare_parameter("command_hz", 10.0)
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter("reverse_enter_heading_rad", 1.75)
        self.declare_parameter("reverse_exit_heading_rad", 1.20)
        self.declare_parameter("reverse_max_linear_mps", 0.22)
        self.declare_parameter("reverse_min_linear_mps", 0.16)
        self.declare_parameter("direction_switch_pause_sec", 0.50)
        self.declare_parameter("use_goal_orientation", True)
        self.declare_parameter("final_yaw_tolerance_rad", 0.45)
        self.declare_parameter("approach_distance_m", 1.20)
        self.declare_parameter("approach_tolerance_m", 0.35)
        self.declare_parameter("require_final_yaw", True)
        self.declare_parameter("use_swing_maneuver", True)
        self.declare_parameter("swing_forward_distance_m", 0.80)
        self.declare_parameter("swing_lateral_offset_m", 0.65)
        self.declare_parameter("swing_tolerance_m", 0.30)
        self.declare_parameter("reverse_angular_sign", -1.0)
        self.declare_parameter("max_final_alignment_attempts", 3)
        self.declare_parameter("parking_radius_m", 0.35)
        self.declare_parameter("parking_completion_radius_m", 0.30)
        self.declare_parameter("parking_drift_limit_m", 0.70)
        self.declare_parameter("parking_linear_mps", 0.18)
        self.declare_parameter("parking_angular_radps", 0.65)
        self.declare_parameter("parking_reverse_seconds", 0.75)
        self.declare_parameter("parking_forward_seconds", 0.75)
        self.declare_parameter("max_parking_cycles", 0)
        self.declare_parameter("parking_reverse_angular_sign", 1.0)
        self.declare_parameter("parking_forward_angular_sign", 1.0)
        self.declare_parameter("settle_radius_m", 0.35)
        self.declare_parameter("settle_yaw_tolerance_rad", 0.65)
        self.declare_parameter("settle_time_sec", 0.80)

        self.max_linear = float(self.get_parameter("max_linear_mps").value)
        self.min_linear = float(self.get_parameter("min_linear_mps").value)
        self.max_angular = float(self.get_parameter("max_angular_radps").value)
        self.heading_kp = float(self.get_parameter("heading_kp").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self.slow_radius = float(self.get_parameter("slow_radius_m").value)
        self.max_heading_for_drive = float(
            self.get_parameter("max_heading_for_drive_rad").value
        )
        command_hz = float(self.get_parameter("command_hz").value)
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self.reverse_enter_heading = float(
            self.get_parameter("reverse_enter_heading_rad").value
        )
        self.reverse_exit_heading = float(
            self.get_parameter("reverse_exit_heading_rad").value
        )
        self.reverse_max_linear = float(
            self.get_parameter("reverse_max_linear_mps").value
        )
        self.reverse_min_linear = float(
            self.get_parameter("reverse_min_linear_mps").value
        )
        self.direction_switch_pause = float(
            self.get_parameter("direction_switch_pause_sec").value
        )
        self.use_goal_orientation = bool(
            self.get_parameter("use_goal_orientation").value
        )
        self.final_yaw_tolerance = float(
            self.get_parameter("final_yaw_tolerance_rad").value
        )
        self.approach_distance = float(
            self.get_parameter("approach_distance_m").value
        )
        self.approach_tolerance = float(
            self.get_parameter("approach_tolerance_m").value
        )
        self.require_final_yaw = bool(
            self.get_parameter("require_final_yaw").value
        )
        self.use_swing_maneuver = bool(
            self.get_parameter("use_swing_maneuver").value
        )
        self.swing_forward_distance = float(
            self.get_parameter("swing_forward_distance_m").value
        )
        self.swing_lateral_offset = float(
            self.get_parameter("swing_lateral_offset_m").value
        )
        self.swing_tolerance = float(
            self.get_parameter("swing_tolerance_m").value
        )
        self.reverse_angular_sign = float(
            self.get_parameter("reverse_angular_sign").value
        )
        self.max_final_alignment_attempts = int(
            self.get_parameter("max_final_alignment_attempts").value
        )
        self.parking_radius = float(self.get_parameter("parking_radius_m").value)
        self.parking_completion_radius = float(
            self.get_parameter("parking_completion_radius_m").value
        )
        self.parking_drift_limit = float(
            self.get_parameter("parking_drift_limit_m").value
        )
        self.parking_linear = float(self.get_parameter("parking_linear_mps").value)
        self.parking_angular = float(
            self.get_parameter("parking_angular_radps").value
        )
        self.parking_reverse_seconds = float(
            self.get_parameter("parking_reverse_seconds").value
        )
        self.parking_forward_seconds = float(
            self.get_parameter("parking_forward_seconds").value
        )
        self.max_parking_cycles = int(self.get_parameter("max_parking_cycles").value)
        self.parking_reverse_angular_sign = float(
            self.get_parameter("parking_reverse_angular_sign").value
        )
        self.parking_forward_angular_sign = float(
            self.get_parameter("parking_forward_angular_sign").value
        )
        self.settle_radius = float(self.get_parameter("settle_radius_m").value)
        self.settle_yaw_tolerance = float(
            self.get_parameter("settle_yaw_tolerance_rad").value
        )
        self.settle_time_sec = float(self.get_parameter("settle_time_sec").value)

        goal_topic = self.get_parameter("goal_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.status_pub = self.create_publisher(String, "simple_goal/status", 10)
        self.create_subscription(PoseStamped, goal_topic, self.on_goal, 10)
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, 10)
        self.create_subscription(String, "simple_goal/cancel", self.on_cancel, 10)
        self.create_timer(1.0 / command_hz, self.control)

        self.goal = None
        self.pose = None
        self.drive_direction = 1
        self.pause_until = None
        self.phase = "final"
        self.swing_lateral = 0.0
        self.final_alignment_attempts = 0
        self.parking_cycles = 0
        self.parking_phase_until = None
        self.goal_completed = False
        self.settle_start_time = None
        self.get_logger().info(
            f"Simple goal follower ready: goal={goal_topic}, pose={pose_topic}, cmd={cmd_topic}"
        )

    def on_goal(self, msg: PoseStamped):
        self.goal = msg
        self.drive_direction = 1
        self.pause_until = None
        self.phase = "approach" if self.use_goal_orientation else "final"
        self.swing_lateral = 0.0
        self.final_alignment_attempts = 0
        self.parking_cycles = 0
        self.parking_phase_until = None
        self.goal_completed = False
        self.settle_start_time = None
        goal_yaw = self.yaw_from_pose_stamped(msg)
        self.get_logger().info(
            f"New goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, "
            f"yaw={goal_yaw:.2f}, phase={self.phase}"
        )
        self.publish_status("active")

    def on_cancel(self, msg: String):
        command = msg.data.strip().lower()
        if command not in ("cancel", "stop", "reset"):
            return
        self.goal = None
        self.drive_direction = 1
        self.pause_until = None
        self.phase = "idle"
        self.parking_phase_until = None
        self.goal_completed = False
        self.settle_start_time = None
        self.cmd_pub.publish(Twist())
        self.publish_status("cancelled")
        self.get_logger().info("Goal cancelled")

    def on_pose(self, msg: PoseWithCovarianceStamped):
        self.pose = msg

    def control(self):
        if self.goal is None or self.pose is None:
            return

        if self.goal_completed:
            self.cmd_pub.publish(Twist())
            self.publish_status("reached")
            return

        now = self.get_clock().now()
        if self.pause_until is not None and now.nanoseconds < self.pause_until:
            self.cmd_pub.publish(Twist())
            return
        self.pause_until = None

        current_x = self.pose.pose.pose.position.x
        current_y = self.pose.pose.pose.position.y
        current_yaw = self.yaw_from_pose(self.pose)
        goal_x = self.goal.pose.position.x
        goal_y = self.goal.pose.position.y
        goal_yaw = self.yaw_from_pose_stamped(self.goal)

        final_dx = goal_x - current_x
        final_dy = goal_y - current_y
        final_distance = math.hypot(final_dx, final_dy)
        final_yaw_error = self.wrap_angle(goal_yaw - current_yaw)

        cmd = Twist()
        if self.update_settle_state(now, final_distance, final_yaw_error):
            self.complete_goal("Goal settled near target")
            return

        near_goal_for_parking = (
            self.require_final_yaw
            and final_distance <= self.parking_radius
            and abs(final_yaw_error) > self.final_yaw_tolerance
        )

        if near_goal_for_parking and not self.phase.startswith("parking"):
            self.phase = "final"

        if self.use_goal_orientation and not near_goal_for_parking:
            self.update_phase(current_x, current_y, final_distance, final_yaw_error)

        if self.phase.startswith("parking"):
            self.control_parking(now, final_distance, final_yaw_error)
            return

        if (
            final_distance <= self.goal_tolerance
            and (
                not self.require_final_yaw
                or abs(final_yaw_error) <= self.final_yaw_tolerance
                or self.final_alignment_attempts >= self.max_final_alignment_attempts
                or (
                    self.max_parking_cycles > 0
                    and self.parking_cycles >= self.max_parking_cycles
                )
            )
        ):
            if (
                self.use_goal_orientation
                and self.require_final_yaw
                and abs(final_yaw_error) > self.final_yaw_tolerance
            ):
                self.get_logger().warn(
                    "Goal position reached; accepting yaw error after parking attempts",
                    throttle_duration_sec=2.0,
                )
            self.complete_goal("Goal pose reached")
            return

        if (
            self.require_final_yaw
            and final_distance <= self.parking_radius
            and abs(final_yaw_error) > self.final_yaw_tolerance
            and (
                self.max_parking_cycles == 0
                or self.parking_cycles < self.max_parking_cycles
            )
        ):
            self.start_parking_phase(now, "parking_reverse", final_yaw_error)
            return

        target_x, target_y = self.active_target(goal_x, goal_y, goal_yaw)
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)

        if distance <= 0.02:
            self.cmd_pub.publish(cmd)
            return

        target_yaw = math.atan2(dy, dx)
        forward_error = self.wrap_angle(target_yaw - current_yaw)
        reverse_error = self.wrap_angle(target_yaw + math.pi - current_yaw)
        self.update_drive_direction(forward_error, reverse_error)
        heading_error = forward_error if self.drive_direction > 0 else reverse_error

        speed_scale = min(1.0, max(0.25, distance / self.slow_radius))
        if self.drive_direction > 0:
            min_linear = self.min_linear
            max_linear = self.max_linear
        else:
            min_linear = self.reverse_min_linear
            max_linear = self.reverse_max_linear

        if abs(heading_error) > self.max_heading_for_drive:
            linear = min_linear * 0.6
        else:
            linear = max(min_linear, max_linear * speed_scale)

        self.publish_drive_command(linear, heading_error)

    def active_target(self, goal_x, goal_y, goal_yaw):
        if self.use_goal_orientation and self.phase == "approach":
            return (
                goal_x - self.approach_distance * math.cos(goal_yaw),
                goal_y - self.approach_distance * math.sin(goal_yaw),
            )
        if self.use_goal_orientation and self.phase == "swing":
            return self.swing_target(goal_x, goal_y, goal_yaw)
        return goal_x, goal_y

    def update_phase(self, current_x, current_y, final_distance, final_yaw_error):
        goal_x = self.goal.pose.position.x
        goal_y = self.goal.pose.position.y
        goal_yaw = self.yaw_from_pose_stamped(self.goal)
        approach_x = goal_x - self.approach_distance * math.cos(goal_yaw)
        approach_y = goal_y - self.approach_distance * math.sin(goal_yaw)
        approach_distance = math.hypot(approach_x - current_x, approach_y - current_y)

        if self.phase == "approach" and approach_distance <= self.approach_tolerance:
            if self.use_swing_maneuver:
                self.swing_lateral = self.opposite_lateral_offset(
                    current_x, current_y, goal_x, goal_y, goal_yaw
                )
                self.phase = "swing"
                self.drive_direction = 1
                self.get_logger().info(
                    "Approach point reached; swinging to opposite front side"
                )
            else:
                self.phase = "final"
                self.get_logger().info("Approach point reached; moving to final pose")
            return

        if self.phase == "swing":
            swing_x, swing_y = self.swing_target(goal_x, goal_y, goal_yaw)
            swing_distance = math.hypot(swing_x - current_x, swing_y - current_y)
            if swing_distance <= self.swing_tolerance:
                self.phase = "final"
                self.get_logger().info("Swing point reached; moving to final pose")
            return

    def swing_target(self, goal_x, goal_y, goal_yaw):
        left_x = -math.sin(goal_yaw)
        left_y = math.cos(goal_yaw)
        return (
            goal_x
            + self.swing_forward_distance * math.cos(goal_yaw)
            + self.swing_lateral * left_x,
            goal_y
            + self.swing_forward_distance * math.sin(goal_yaw)
            + self.swing_lateral * left_y,
        )

    def opposite_lateral_offset(self, current_x, current_y, goal_x, goal_y, goal_yaw):
        rel_x = current_x - goal_x
        rel_y = current_y - goal_y
        local_left = -math.sin(goal_yaw) * rel_x + math.cos(goal_yaw) * rel_y

        if abs(local_left) < 0.05:
            local_left = 1.0

        return -math.copysign(self.swing_lateral_offset, local_left)

    def update_drive_direction(self, forward_error, reverse_error):
        if not self.allow_reverse:
            self.drive_direction = 1
            return

        next_direction = self.drive_direction
        if self.drive_direction > 0 and abs(forward_error) > self.reverse_enter_heading:
            next_direction = -1
        elif self.drive_direction < 0 and abs(forward_error) < self.reverse_exit_heading:
            next_direction = 1
        elif self.drive_direction < 0 and abs(reverse_error) > self.reverse_enter_heading:
            next_direction = 1

        if next_direction == self.drive_direction:
            return

        self.drive_direction = next_direction
        self.pause_until = (
            self.get_clock().now().nanoseconds
            + int(self.direction_switch_pause * 1e9)
        )
        direction_name = "reverse" if self.drive_direction < 0 else "forward"
        self.get_logger().info(f"Switching drive direction to {direction_name}")

    def publish_drive_command(self, linear, heading_error):
        cmd = Twist()
        cmd.linear.x = linear * self.drive_direction
        angular = self.heading_kp * heading_error
        if self.drive_direction < 0:
            angular *= self.reverse_angular_sign
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, angular))
        self.cmd_pub.publish(cmd)

    def start_parking_phase(self, now, phase, final_yaw_error):
        self.phase = phase
        self.parking_phase_until = now.nanoseconds + int(
            (
                self.parking_reverse_seconds
                if phase == "parking_reverse"
                else self.parking_forward_seconds
            )
            * 1e9
        )
        if phase == "parking_reverse":
            self.parking_cycles += 1
        self.get_logger().info(
            f"Parking phase {phase} "
            f"cycle={self.parking_cycles}/"
            f"{self.max_parking_cycles if self.max_parking_cycles > 0 else 'unlimited'}, "
            f"yaw_error={final_yaw_error:.2f}"
        )

    def control_parking(self, now, final_distance, final_yaw_error):
        if self.update_settle_state(now, final_distance, final_yaw_error):
            self.complete_goal("Parking settled near target")
            return

        if abs(final_yaw_error) <= self.final_yaw_tolerance:
            if final_distance <= self.parking_completion_radius:
                self.complete_goal("Parking pose aligned")
            else:
                self.phase = "final"
                self.parking_phase_until = None
                self.cmd_pub.publish(Twist())
                self.get_logger().info(
                    "Parking yaw aligned; returning to goal position"
                )
            return

        if final_distance > self.parking_drift_limit:
            self.phase = "final"
            self.parking_phase_until = None
            self.get_logger().info("Parking drifted away; returning directly to goal")
            return

        if (
            self.max_parking_cycles > 0
            and self.parking_cycles > self.max_parking_cycles
            and self.phase == "parking_reverse"
        ):
            self.phase = "final"
            self.parking_phase_until = None
            self.cmd_pub.publish(Twist())
            self.get_logger().warn("Parking cycle limit reached; stopping near goal")
            return

        if self.parking_phase_until is not None and now.nanoseconds >= self.parking_phase_until:
            next_phase = (
                "parking_forward"
                if self.phase == "parking_reverse"
                else "parking_reverse"
            )
            self.start_parking_phase(now, next_phase, final_yaw_error)

        yaw_sign = math.copysign(1.0, final_yaw_error)
        cmd = Twist()
        if self.phase == "parking_reverse":
            cmd.linear.x = -self.parking_linear
            angular_sign = self.parking_reverse_angular_sign
        else:
            cmd.linear.x = self.parking_linear
            angular_sign = self.parking_forward_angular_sign
        cmd.angular.z = max(
            -self.max_angular,
            min(self.max_angular, yaw_sign * angular_sign * self.parking_angular),
        )
        self.cmd_pub.publish(cmd)

    def update_settle_state(self, now, final_distance, final_yaw_error):
        settled = (
            final_distance <= self.settle_radius
            and abs(final_yaw_error) <= self.settle_yaw_tolerance
        )
        if not settled:
            self.settle_start_time = None
            return False

        if self.settle_start_time is None:
            self.settle_start_time = now
            self.cmd_pub.publish(Twist())
            return False

        elapsed = (now - self.settle_start_time).nanoseconds / 1e9
        self.cmd_pub.publish(Twist())
        return elapsed >= self.settle_time_sec

    def complete_goal(self, reason):
        self.goal_completed = True
        self.phase = "done"
        self.parking_phase_until = None
        self.settle_start_time = None
        self.cmd_pub.publish(Twist())
        self.publish_status("reached")
        self.get_logger().info(f"{reason}; goal latched complete")

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    @staticmethod
    def yaw_from_pose(msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_from_pose_stamped(msg: PoseStamped):
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle: float):
        return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = SimpleGoalFollower()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
