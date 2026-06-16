from glob import glob
from setuptools import setup

package_name = "camping_robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/config", glob("config/*.rviz")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="UDP bringup nodes for the camping surveillance robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ackermann_udp_bridge = camping_robot_bringup.ackermann_udp_bridge:main",
            "battery_monitor = camping_robot_bringup.battery_monitor:main",
            "cmd_vel_odom = camping_robot_bringup.cmd_vel_odom:main",
            "drive_smoke_test = camping_robot_bringup.drive_smoke_test:main",
            "elevator_assist_node = camping_robot_bringup.elevator_assist_node:main",
            "esp32_camera_monitor = camping_robot_bringup.esp32_camera_monitor:main",
            "esp32_status_node = camping_robot_bringup.esp32_status_node:main",
            "hazard_monitor = camping_robot_bringup.hazard_monitor:main",
            "lidar_safety_stop = camping_robot_bringup.lidar_safety_stop:main",
            "lds14_udp_node = camping_robot_bringup.lds14_udp_node:main",
            "mission_supervisor = camping_robot_bringup.mission_supervisor:main",
            "mission_assistance_node = camping_robot_bringup.mission_assistance_node:main",
            "mission_command_node = camping_robot_bringup.mission_command_node:main",
            "mission_event_logger = camping_robot_bringup.mission_event_logger:main",
            "mission_task_manager = camping_robot_bringup.mission_task_manager:main",
            "robot_health_monitor = camping_robot_bringup.robot_health_monitor:main",
            "simple_goal_follower = camping_robot_bringup.simple_goal_follower:main",
            "straight_trim_test = camping_robot_bringup.straight_trim_test:main",
            "udp_imu_node = camping_robot_bringup.udp_imu_node:main",
            "waypoint_patrol = camping_robot_bringup.waypoint_patrol:main",
            "waypoint_recorder = camping_robot_bringup.waypoint_recorder:main",
            "warning_buzzer_udp_bridge = camping_robot_bringup.warning_buzzer_udp_bridge:main",
        ],
    },
)
