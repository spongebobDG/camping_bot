from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("camping_robot_bringup"))
    rviz_config = package_share / "config" / "mapping.rviz"

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_mapping",
                output="screen",
                arguments=["-d", str(rviz_config)],
            )
        ]
    )
