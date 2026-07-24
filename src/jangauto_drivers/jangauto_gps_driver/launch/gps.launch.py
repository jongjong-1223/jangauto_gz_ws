from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # GPS covariance filler: 시뮬레이션 navsat 센서가 채우지 않는 position_covariance를
    # 고정값으로 채워서 재발행 (navsat_transform_node/ekf_global이 제대로 가중치를 두고 쓰도록)
    gps_covariance_filler_node = Node(
        package="jangauto_gps_driver",
        executable="gps_covariance_filler.py",
        name="gps_covariance_filler",
        output="screen",
        remappings=[('navsat', '/navsat'), ('navsat_fixed', '/navsat_fixed')],
    )

    return LaunchDescription([
        gps_covariance_filler_node,
    ])
