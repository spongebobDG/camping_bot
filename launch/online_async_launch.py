# { /* Reason: 오도메트리, 라이다 무선 브릿지, 정적 TF 및 SLAM Toolbox를 일괄적으로 시퀀스 구동하는 ROS2 통합 자율주행 런치 스크립트입니다. */ }
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 정적 좌표계 변환 방송 (로봇 중심 base_link와 라이다 laser_frame의 물리적 위치 연결)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
        ),
        
        # 2. ESP32 바퀴 오도메트리 수신 노드
        Node(
            package='camping_bot_bridge',
            executable='odom_publisher',
            name='esp32_odom_publisher'
        ),
        
        # 3. ESP32 무선 라이다 수신 노드
        Node(
            package='camping_bot_bridge',
            executable='lidar_bridge',
            name='lidar_udp_bridge'
        ),
        
        # 4. 실시간 2D SLAM 매핑 엔진 (SLAM Toolbox 아동기 모드 가동)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'use_sim_time': False,
                'max_laser_range': 3.5 # LDS-01 최대 측정 반경 맞춤 수치
            }]
        )
    ])