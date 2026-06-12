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
    robot_params = package_share / "config" / "robot.yaml"

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
                launch_arguments={"map": LaunchConfiguration("map")}.items(),
            ),
            Node(
                package="camping_robot_bringup",
                executable="simple_goal_follower",
                name="simple_goal_follower",
                output="screen",
                parameters=[str(robot_params)],
            ),
        ]
    )
