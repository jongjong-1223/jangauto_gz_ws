"""GPS+IMU 센서 퓨전 로컬라이제이션 launch 파일.

4단계 파이프라인을 구성한다:
- cmd_vel LPF: `cmd_vel_out`(명령 속도)을 저역통과 필터링해 로컬 EKF의
  선속도(Vx) pseudo-measurement로 변환(바퀴 엔코더 등 실제 속도 센서 없음).
- 로컬 EKF: IMU+위 pseudo-velocity를 융합해 부드럽지만 드리프트가 있는
  `odom` 프레임 추정.
- NavSat Transform: GPS 위경도를 EKF가 쓸 수 있는 좌표계로 변환.
- 글로벌 EKF: GPS+IMU를 융합해 드리프트 없는 `map` 프레임 추정.

EKF 두 노드는 같은 `ekf.yaml` 설정 파일을 공유하되, remapping으로 입출력
토픽만 다르게 잡아 로컬/글로벌 두 EKF 인스턴스를 구분한다.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_perception = get_package_share_directory('jangauto_perception')

    # 6-0. cmd_vel LPF: cmd_vel_out(명령 속도)을 필터링해 ekf_local의 twist0
    # 입력(pseudo-velocity)으로 변환합니다.
    cmd_vel_twist_lpf_node = Node(
        package='jangauto_perception',
        executable='cmd_vel_twist_lpf.py',
        name='cmd_vel_twist_lpf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 6-1. 로컬 EKF (IMU -> odom): IMU 데이터만 사용하여 부드럽지만 드리프트가 있는 지역(local) 주행 거리계를 생성합니다.
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[os.path.join(pkg_project_perception, "config", "ekf.yaml"),
                    {'use_sim_time': True}
                    ],
        remappings=[('odometry/filtered', 'odom')]  # nav2 odom_topic(/odom)에 맞춤
    )

    # 6-2. 글로벌 EKF (GPS+IMU -> map): GPS와 IMU 데이터를 융합하여 드리프트가 없는 전역(global) 위치를 추정합니다.
    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[os.path.join(pkg_project_perception, "config", "ekf.yaml"),
                    {'use_sim_time': True}
                    ],
        remappings=[('odometry/filtered', 'odometry/global')]  # 출력 토픽 이름을 변경
    )

    # 6-3. NavSat Transform: GPS의 위도/경도 데이터를 EKF가 사용할 수 있는 UTM 또는 지역 좌표계(x,y)로 변환합니다.
    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[os.path.join(pkg_project_perception, "config", "ekf.yaml"),
                    {'use_sim_time': True}
                    ],
        remappings=[
            ("imu", "/imu"),
            ("gps/fix", "/navsat_fixed"),
            ("odometry/filtered", "/odometry/global"),
            ("gps/filtered", "/gps/filtered"),
            ("odometry/gps", "/odometry/gps"),
        ]
    )

    return LaunchDescription([
        cmd_vel_twist_lpf_node,
        ekf_local_node,
        ekf_global_node,
        navsat_transform_node,
    ])
