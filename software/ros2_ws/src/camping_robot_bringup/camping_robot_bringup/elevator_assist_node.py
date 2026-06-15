import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ElevatorAssistNode(Node):
    def __init__(self):
        super().__init__("elevator_assist_node")
        self.phase = "idle"
        self.last_status = None
        self.elevator_requested = False

        self.status_pub = self.create_publisher(String, "mission/elevator_status", 10)
        self.command_pub = self.create_publisher(String, "mission/command", 10)

        self.create_subscription(String, "mission/task_command", self.on_task_command, 10)
        self.create_subscription(String, "mission/task_status", self.on_task_status, 10)
        self.create_subscription(
            String, "mission/elevator_decision", self.on_decision, 10
        )
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info("Elevator assist node ready")

    def on_task_command(self, msg):
        command = msg.data.strip().lower()
        if command == "elevator":
            self.elevator_requested = True
            self.phase = "navigating_to_elevator"
            self.publish_status("started")
        elif command in ("cancel", "stop", "idle") and self.elevator_requested:
            self.reset("cancelled")

    def on_task_status(self, msg):
        text = msg.data
        if not self.elevator_requested:
            return
        if "event=reached" in text and self.phase == "navigating_to_elevator":
            self.phase = "waiting_for_elevator"
            self.publish_status("arrived_at_elevator")
        elif "blocked_by_danger" in text or "failed" in text:
            self.phase = "blocked"
            self.publish_status("navigation_blocked")

    def on_decision(self, msg):
        decision = msg.data.strip().lower()
        if decision == "cancel":
            self.publish_mission_command("stop")
            self.reset("cancelled")
            return
        if decision == "call":
            self.elevator_requested = True
            if self.phase == "idle":
                self.phase = "waiting_for_elevator"
            self.publish_status("call_requested")
            return
        if decision == "entered":
            self.elevator_requested = True
            self.phase = "inside_elevator"
            self.publish_status("entered")
            return
        if decision == "floor_selected":
            self.elevator_requested = True
            self.phase = "riding"
            self.publish_status("floor_selected")
            return
        if decision == "exited":
            self.elevator_requested = True
            self.phase = "exited"
            self.publish_status("exited")
            return
        if decision == "complete":
            self.reset("complete")
            return
        self.publish_status(f"unknown_decision:{decision}")

    def publish_mission_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def reset(self, event):
        self.elevator_requested = False
        self.phase = "idle"
        self.publish_status(event)

    def publish_status(self, event=None):
        text = f"phase={self.phase}"
        if event:
            text += f"; event={event}"
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if text != self.last_status:
            self.get_logger().info(text)
            self.last_status = text


def main():
    rclpy.init()
    node = ElevatorAssistNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
