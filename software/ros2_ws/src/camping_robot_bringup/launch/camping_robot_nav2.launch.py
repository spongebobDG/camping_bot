from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("camping_robot_bringup"))
    localization_launch = package_share / "launch" / "camping_robot_localization.launch.py"
    nav2_params = package_share / "config" / "nav2_params.yaml"
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="/home/spbdg/maps/camping_test_map.yaml",
        description="Full path to the saved map YAML file.",
    )

    return LaunchDescription(
        [
            map_arg,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(localization_launch)),
                launch_arguments={
                    "map": LaunchConfiguration("map")
                }.items(),
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[str(nav2_params)],
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[str(nav2_params)],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[str(nav2_params)],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[str(nav2_params)],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[str(nav2_params)],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[str(nav2_params)],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[str(nav2_params)],
                remappings=[
                    ("cmd_vel", "cmd_vel_nav"),
                    ("cmd_vel_smoothed", "cmd_vel_raw"),
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[str(nav2_params)],
            ),
        ]
    )
