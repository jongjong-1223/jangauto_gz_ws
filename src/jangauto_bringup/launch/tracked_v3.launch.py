# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""tracked_v3 로봇 전체 스택을 띄우는 최상위 bringup launch 파일.

각 하위 패키지의 launch 파일을 include해서 한 번에 실행한다:
- `gazebo`/`description`: 시뮬레이션 월드 + 로봇 모델(URDF, 전방 뎁스카메라 포함).
- `localization`: GPS+IMU 로컬라이제이션(EKF 2단 + navsat transform) —
  `map`→`odom`→`base_link` TF를 직접 발행하므로 Nav2 쪽 AMCL/SLAM은 쓰지 않는다.
- `gps`: 시뮬레이션 GPS의 `position_covariance`를 채워 재발행.
- `uwb`: 실제 UWB 하드웨어 전까지 nav2 정적 맵(`/map`)을 가상 데이터로 채우는
  임시 노드(`jangauto_uwb_driver`).
- `navigation2`: Nav2 내비게이션 서버 묶음(`navigation_launch.py`) —
  AMCL/SLAM 분기가 있는 `bringup_launch.py` 대신, GPS EKF 로컬라이제이션과
  충돌하지 않도록 서버들만 포함한 이 launch 파일을 직접 include한다.
- `hmi`: 앱 웹소켓 브릿지.
- `mission`: YASMIN 미션 상태머신 + CAL/ALIGN/RUN 액션 서버 + 진단→에러 연결.
- `control`: `cmd_vel_arbiter` + 조이스틱 수동조종(`jangauto_control`).
- `diagnostics`: `diagnostic_aggregator`.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_project_gazebo = get_package_share_directory('jangauto_gazebo')
    pkg_project_description = get_package_share_directory('jangauto_description')
    pkg_project_perception = get_package_share_directory('jangauto_perception')
    pkg_project_gps_driver = get_package_share_directory('jangauto_gps_driver')
    pkg_project_uwb_driver = get_package_share_directory('jangauto_uwb_driver')
    pkg_project_navigation2 = get_package_share_directory('jangauto_navigation2')
    pkg_project_hmi = get_package_share_directory('jangauto_hmi')
    pkg_project_mission = get_package_share_directory('jangauto_mission')
    pkg_project_control = get_package_share_directory('jangauto_control')

    # Visualize in RViz
    # rviz = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments=['-d', os.path.join(get_package_share_directory('jangauto_bringup'), 'config', 'tracked_v1.rviz')],
    #    condition=IfCondition(LaunchConfiguration('rviz'))
    # )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, 'launch', 'gazebo.launch.py')),
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_description, 'launch', 'description.launch.py')),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_perception, 'launch', 'localization.launch.py')),
    )

    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gps_driver, 'launch', 'gps.launch.py')),
    )

    uwb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_uwb_driver, 'launch', 'uwb.launch.py')),
    )

    # AMCL/SLAM 분기가 있는 bringup_launch.py 대신 navigation_launch.py만 include —
    # 로컬라이제이션은 GPS+IMU EKF(localization)가 이미 담당하므로 nav2 쪽
    # AMCL/SLAM을 같이 띄우면 map->odom TF가 두 시스템에서 충돌한다.
    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_navigation2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
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
        gazebo,
        description,
        localization,
        gps,
        uwb,
        navigation2,
        hmi,
        mission,
        control,
        diagnostics,
        # rviz,
    ])
