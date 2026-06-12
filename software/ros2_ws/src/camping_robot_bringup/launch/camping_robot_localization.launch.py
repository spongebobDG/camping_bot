from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("camping_robot_bringup"))
    udp_launch = package_share / "launch" / "camping_robot_udp.launch.py"
    robot_params = package_share / "config" / "robot.yaml"
    nav2_params = package_share / "config" / "nav2_localization.yaml"

    map_arg = DeclareLaunchArgument(
        "map",
        default_value="/home/spbdg/maps/camping_test_map.yaml",
        description="Full path to the saved map YAML file.",
    )

    return LaunchDescription(
        [
            map_arg,
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(udp_launch))),
            Node(
                package="camping_robot_bringup",
                executable="cmd_vel_odom",
                name="cmd_vel_odom",
                output="screen",
                parameters=[str(robot_params)],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    str(nav2_params),
                    {"yaml_filename": LaunchConfiguration("map")},
                ],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[str(nav2_params)],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[str(nav2_params)],
            ),
        ]
    )
