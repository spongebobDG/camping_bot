import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__("waypoint_recorder")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("name_prefix", "wp")
        self.count = 0
        self.name_prefix = self.get_parameter("name_prefix").value
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.on_goal,
            10,
        )
        self.get_logger().info(
            "Waypoint recorder ready. Click 2D Goal Pose in RViz to print YAML."
        )

    def on_goal(self, msg: PoseStamped):
        self.count += 1
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = self.yaw_from_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        name = f"{self.name_prefix}_{self.count:02d}"
        yaml_text = (
            f"  - name: {name}\n"
            f"    x: {x:.3f}\n"
            f"    y: {y:.3f}\n"
            f"    yaw: {yaw:.3f}"
        )
        self.get_logger().info("\n" + yaml_text)

    @staticmethod
    def yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
