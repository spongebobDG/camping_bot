from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("camping_robot_bringup"))
    udp_launch = package_share / "launch" / "camping_robot_udp.launch.py"
    slam_params = package_share / "config" / "slam_toolbox.yaml"

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(udp_launch))),
            Node(
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry",
                output="screen",
                parameters=[
                    {
                        "laser_scan_topic": "/scan",
                        "odom_topic": "/odom",
                        "publish_tf": True,
                        "base_frame_id": "base_link",
                        "odom_frame_id": "odom",
                        "freq": 15.0,
                    }
                ],
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
