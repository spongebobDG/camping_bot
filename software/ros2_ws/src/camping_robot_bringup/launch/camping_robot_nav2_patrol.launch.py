from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("camping_robot_bringup"))
    nav2_launch = package_share / "launch" / "camping_robot_nav2.launch.py"
    waypoints = package_share / "config" / "patrol_waypoints.yaml"

    map_arg = DeclareLaunchArgument(
        "map",
        default_value="/home/spbdg/maps/camping_test_map.yaml",
        description="Full path to the saved map YAML file.",
    )
    waypoints_arg = DeclareLaunchArgument(
        "waypoints",
        default_value=str(waypoints),
        description="YAML waypoint file for Nav2 patrol.",
    )
    loop_arg = DeclareLaunchArgument(
        "loop",
        default_value="false",
        description="Repeat waypoints forever when true.",
    )

    return LaunchDescription(
        [
            map_arg,
            waypoints_arg,
            loop_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_launch)),
                launch_arguments={"map": LaunchConfiguration("map")}.items(),
            ),
            Node(
                package="camping_robot_bringup",
                executable="waypoint_patrol",
                name="waypoint_patrol",
                output="screen",
                parameters=[
                    {
                        "waypoints_file": LaunchConfiguration("waypoints"),
                        "loop": LaunchConfiguration("loop"),
                        "start_paused": True,
                        "navigation_mode": "nav2",
                    }
                ],
            ),
        ]
    )
