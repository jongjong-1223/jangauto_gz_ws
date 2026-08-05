"""`gps_covariance_filler` 노드 하나를 실행하는 launch 파일.

시뮬레이션 navsat 센서가 채우지 않는 `position_covariance`를 고정값으로
채워서 재발행한다 — `navsat_transform_node`/`ekf_global`이 GPS 데이터에
제대로 가중치를 두고 융합하려면 이 값이 필요하다.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gps_covariance_filler_node = Node(
        package="jangauto_gps_driver",
        executable="gps_covariance_filler_simul.py",
        name="gps_covariance_filler",
        output="screen",
        remappings=[('navsat', '/navsat'), ('navsat_fixed', '/navsat_fixed')],
    )

    return LaunchDescription([
        gps_covariance_filler_node,
    ])
