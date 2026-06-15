from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_share = Path(get_package_share_directory("camping_robot_bringup"))
    params = str(package_share / "config" / "robot.yaml")
    mission_locations = str(package_share / "config" / "mission_locations.yaml")

    return LaunchDescription(
        [
            Node(
                package="camping_robot_bringup",
                executable="ackermann_udp_bridge",
                name="ackermann_udp_bridge",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="camping_robot_bringup",
                executable="udp_imu_node",
                name="udp_imu_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="camping_robot_bringup",
                executable="esp32_status_node",
                name="esp32_status_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="camping_robot_bringup",
                executable="esp32_camera_monitor",
                name="esp32_camera_monitor",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="camping_robot_bringup",
                executable="lds14_udp_node",
                name="lds14_udp_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="camping_robot_bringup",
                executable="robot_health_monitor",
                name="robot_health_monitor",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="camping_robot_bringup",
                executable="hazard_monitor",
                name="hazard_monitor",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="camping_robot_bringup",
                executable="mission_supervisor",
                name="mission_supervisor",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="camping_robot_bringup",
                executable="mission_command_node",
                name="mission_command_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="camping_robot_bringup",
                executable="mission_assistance_node",
                name="mission_assistance_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="camping_robot_bringup",
                executable="mission_task_manager",
                name="mission_task_manager",
                output="screen",
                parameters=[params, {"mission_locations_file": mission_locations}],
            ),
            Node(
                package="camping_robot_bringup",
                executable="warning_buzzer_udp_bridge",
                name="warning_buzzer_udp_bridge",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="camping_robot_bringup",
                executable="lidar_safety_stop",
                name="lidar_safety_stop",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                arguments=[
                    "--x", "0.18",
                    "--y", "0.0",
                    "--z", "0.16",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.7854",
                    "--frame-id", "base_link",
                    "--child-frame-id", "laser",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_tf",
                arguments=[
                    "--x", "0.07",
                    "--y", "0.0",
                    "--z", "0.08",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.0",
                    "--frame-id", "base_link",
                    "--child-frame-id", "imu_link",
                ],
            ),
        ]
    )
