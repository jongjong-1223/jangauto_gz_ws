"""`imu_real` 노드 하나를 실행하는 launch 파일(실기체 전용).

실제 IMU를 시리얼로 읽어 `/imu`로 발행한다 — 시뮬레이션에서는 Gazebo IMU
센서+브릿지가 이 역할을 대신하므로 _simul 대응 파일이 없다.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_node = Node(
        package="jangauto_imu_driver",
        executable="imu_real.py",
        name="imu_real",
        output="screen",
    )

    return LaunchDescription([
        imu_node,
    ])
