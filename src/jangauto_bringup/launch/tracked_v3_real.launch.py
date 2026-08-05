"""tracked_v3 로봇 전체 스택을 띄우는 최상위 bringup launch 파일(실기체 전용).

tracked_v3_simul.launch.py와 골격은 같지만 Gazebo/시뮬 전용 요소를 실제
하드웨어 드라이버로 교체한다:
- `description`: robot_state_publisher + 센서 프레임 static TF(모델 자산은
  시뮬과 공용, use_sim_time만 다름) — `gazebo` 스폰은 없음.
- `localization`: UWB+IMU 로컬라이제이션(EKF 2단) — 이 워크스페이스의 실제
  절대위치 측위 센서는 GPS가 아니라 UWB라서 GPS/navsat 경로 자체가 없다.
- `uwb`: 실제 UWB 시리얼 드라이버(절대위치, EKF 입력) + `/map` 가상 발행
  노드(실제 매핑 솔루션이 생기기 전까지의 placeholder, 시뮬과 동일 노드 재사용).
- `motor`: cmd_vel_out을 시리얼로 실제 모터 컨트롤러에 전달(신규
  `jangauto_motor_driver`) — 시뮬에서는 Gazebo diff_drive 플러그인이 대신하던
  역할.
- `imu`: 실제 IMU 시리얼 드라이버 -> `/imu`(신규 `jangauto_imu_driver`) —
  시뮬에서는 Gazebo IMU 센서+브릿지가 대신하던 역할.
- `navigation2`: Nav2 내비게이션 서버 묶음, `nav2_params_real.yaml` +
  `use_sim_time:false`로 include(파일 자체는 시뮬과 완전히 동일, 인자만 다름).
- `hmi`/`mission`/`control`/`diagnostics`: 하드웨어에 안 종속적이라 시뮬과
  무수정 재사용.
- GPS(`jangauto_gps_driver`)는 이 real bringup에 포함하지 않는다(이 로봇은
  GPS를 쓰지 않음 — 위 `localization`/`uwb` 참고).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_project_description = get_package_share_directory('jangauto_description')
    pkg_project_perception = get_package_share_directory('jangauto_perception')
    pkg_project_uwb_driver = get_package_share_directory('jangauto_uwb_driver')
    pkg_project_motor_driver = get_package_share_directory('jangauto_motor_driver')
    pkg_project_imu_driver = get_package_share_directory('jangauto_imu_driver')
    pkg_project_navigation2 = get_package_share_directory('jangauto_navigation2')
    pkg_project_hmi = get_package_share_directory('jangauto_hmi')
    pkg_project_mission = get_package_share_directory('jangauto_mission')
    pkg_project_control = get_package_share_directory('jangauto_control')

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_description, 'launch', 'description_real.launch.py')),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_perception, 'launch', 'localization_real.launch.py')),
    )

    uwb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_uwb_driver, 'launch', 'uwb_real.launch.py')),
    )

    motor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_motor_driver, 'launch', 'motor_real.launch.py')),
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_imu_driver, 'launch', 'imu_real.launch.py')),
    )

    # AMCL/SLAM 분기가 있는 bringup_launch.py 대신 navigation_launch.py만 include —
    # 로컬라이제이션은 UWB+IMU EKF(localization)가 이미 담당하므로 nav2 쪽
    # AMCL/SLAM을 같이 띄우면 map->odom TF가 두 시스템에서 충돌한다.
    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_navigation2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': os.path.join(pkg_project_navigation2, 'params', 'nav2_params_real.yaml'),
        }.items(),
    )

    hmi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_hmi, 'launch', 'hmi.launch.py')),
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_mission, 'launch', 'mission.launch.py')),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_control, 'launch', 'control.launch.py')),
    )

    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('jangauto_bringup'), 'launch', 'diagnostics.launch.py')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        description,
        localization,
        uwb,
        motor,
        imu,
        navigation2,
        hmi,
        mission,
        control,
        diagnostics,
    ])
