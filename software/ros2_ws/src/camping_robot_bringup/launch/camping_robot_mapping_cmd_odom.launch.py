from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("camping_robot_bringup"))
    udp_launch = package_share / "launch" / "camping_robot_udp.launch.py"
    params = package_share / "config" / "robot.yaml"
    slam_params = package_share / "config" / "slam_toolbox.yaml"

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(udp_launch))),
            Node(
                package="camping_robot_bringup",
                executable="cmd_vel_odom",
                name="cmd_vel_odom",
                output="screen",
                parameters=[str(params)],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[str(slam_params)],
            ),
        ]
    )
