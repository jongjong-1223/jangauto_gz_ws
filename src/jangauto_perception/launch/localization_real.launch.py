"""UWB+IMU 센서 퓨전 로컬라이제이션 launch 파일(실기체 전용).

4단계 파이프라인을 구성한다(GPS/NavSat Transform이 없는 대신 UWB가 map 프레임
절대위치를 직접 낸다):
- IMU yaw 보정: 원본 `/imu`를 CAL이 계산한 offset만큼 회전시켜
  `/imu_calibrated`로 재발행(EKF들의 실제 imu0 입력) — localization_simul과
  완전히 동일한 노드, 로직 변경 없음.
- cmd_vel LPF: `cmd_vel_out`(명령 속도)을 저역통과 필터링해 로컬 EKF의
  선속도(Vx) pseudo-measurement로 변환 — 동일 노드, 로직 변경 없음.
- 로컬 EKF: IMU+위 pseudo-velocity를 융합해 부드럽지만 드리프트가 있는
  `odom` 프레임 추정 — ekf_real.yaml의 ekf_local 블록은 ekf_simul.yaml과
  100% 동일.
- 글로벌 EKF: UWB(`jangauto_uwb_driver`의 `/abs_xy_fixed`)+IMU를 융합해
  드리프트 없는 `map` 프레임 추정(ekf_real.yaml의 pose0 블록).

이 워크스페이스의 실제 절대위치 측위 센서는 GPS가 아니라 UWB라서, GPS
경로(navsat_transform_node)는 이 launch에 포함하지 않는다.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_perception = get_package_share_directory('jangauto_perception')

    imu_yaw_corrector_node = Node(
        package='jangauto_perception',
        executable='imu_yaw_corrector.py',
        name='imu_yaw_corrector',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    cmd_vel_twist_lpf_node = Node(
        package='jangauto_perception',
        executable='cmd_vel_twist_lpf.py',
        name='cmd_vel_twist_lpf',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # 로컬 EKF (IMU -> odom): IMU 데이터만 사용하여 부드럽지만 드리프트가 있는 지역(local) 주행 거리계를 생성합니다.
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[os.path.join(pkg_project_perception, "config", "ekf_real.yaml"),
                    {'use_sim_time': False}
                    ],
        remappings=[('odometry/filtered', 'odom')]  # nav2 odom_topic(/odom)에 맞춤
    )

    # 글로벌 EKF (UWB+IMU -> map): UWB와 IMU 데이터를 융합하여 드리프트가 없는 전역(global) 위치를 추정합니다.
    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[os.path.join(pkg_project_perception, "config", "ekf_real.yaml"),
                    {'use_sim_time': False}
                    ],
        remappings=[('odometry/filtered', 'odometry/global')]  # 출력 토픽 이름을 변경
    )

    return LaunchDescription([
        imu_yaw_corrector_node,
        cmd_vel_twist_lpf_node,
        ekf_local_node,
        ekf_global_node,
    ])
