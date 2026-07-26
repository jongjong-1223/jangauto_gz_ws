# Copyright (c) 2020 Samsung Research Russia
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

"""SLAM(slam_toolbox)으로 실시간 지도를 생성하며 동시에 지도 저장 서비스를 띄우는 launch 파일.

**참고용 — 미사용.** 이 프로젝트는 SLAM 대신 GPS+IMU EKF(jangauto_perception)로
map->odom->base_link TF를 직접 발행하고, 정적 맵도 jangauto_uwb_driver가 발행하는
가상 맵으로 대신한다(`nav2_params.yaml` 헤더 주석 참고). 이 파일은 stock nav2_bringup
구조 문서화 및 향후 참고용으로만 `launch/references/`에 보관한다.

## 역할
- `start_map_server`: 지도를 파일로 저장할 수 있는 `map_saver_server`와, 이를
  관리하는 `lifecycle_manager_slam`을 실행한다(지도 자체를 만드는 건 아래
  slam_toolbox의 역할이고, 이 그룹은 "저장" 서비스만 담당).
- `start_slam_toolbox_cmd`: slam_toolbox 패키지의 `online_sync_launch.py`를
  include해 실제 SLAM(스캔 매칭 기반 지도 생성 + 위치추정)을 수행한다.
  - `params_file`(nav2_params.yaml)에 `slam_toolbox` 노드 설정이 있는지를
    `HasNodeParams`로 검사해서, 있으면 그 파일을 slam_toolbox에 그대로
    전달하고, 없으면 전달하지 않아 slam_toolbox 자체 기본값을 쓰게 한다
    (`params_file`을 무조건 넘기면, 그 안에 slam_toolbox 섹션이 없을 때
    기본 파라미터 로드를 막아버리기 때문 — `IfCondition`/`UnlessCondition`으로
    두 include가 배타적으로 하나만 실행됨).
  - `/scan`·`/tf`·`/tf_static`·`/map` 절대 토픽을 상대 경로로 remap해,
    네임스페이스가 적용된 SLAM 세션도 올바른 토픽을 구독/발행하게 한다.

## 선언하는 launch 인자
- `namespace`: SLAM 세션 및 토픽 remap에 사용되는 네임스페이스.
- `params_file`: 전체 노드에 쓰이는 ROS2 파라미터 yaml 경로(slam_toolbox 설정 포함 가능).
- `use_sim_time`, `autostart`, `use_respawn`, `log_level`: 공통 실행 옵션.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import HasNodeParams, RewrittenYaml


def generate_launch_description():
    # 아래 DeclareLaunchArgument들이 실제 값을 선언하며, 여기서는 참조 핸들만 만든다.
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # lifecycle_manager_slam이 관리할 lifecycle 노드 이름 목록(map_saver_server 하나뿐).
    lifecycle_nodes = ['map_saver']

    # 기본 파라미터 파일 경로, 그리고 slam_toolbox 패키지가 제공하는 SLAM launch 파일 경로.
    bringup_dir = get_package_share_directory('jangauto_navigation2')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')

    # params_file(yaml)을 실행 시점 값으로 재작성한 임시 파일로 감싸서 map_saver_server에 전달.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # 이 launch 파일이 받는 인자 선언(각 default_value/description은 모듈 docstring 참고).
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    # 지도 저장 서비스(map_saver_server)와 그 lifecycle 관리자.
    # 실제 SLAM(지도 생성) 자체는 아래 start_slam_toolbox_cmd가 담당한다.
    start_map_server = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[configured_params],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ]
    )

    # params_file 안에 slam_toolbox 노드 설정이 실제로 있는지 검사한다.
    # 없는데도 params_file을 slam_toolbox에 그대로 넘기면, 그 파일에
    # slam_toolbox 섹션이 없어서 오히려 slam_toolbox 자체 기본 파라미터
    # 로드가 막혀버리기 때문에 이 분기가 필요하다.
    has_slam_toolbox_params = HasNodeParams(
        source_file=params_file, node_name='slam_toolbox'
    )

    start_slam_toolbox_cmd = GroupAction(

        actions=[
            # 절대 토픽(/scan, /tf, /tf_static, /map)을 상대 경로로 remap —
            # 이렇게 해야 SLAM 세션도 네임스페이스가 적용됐을 때 올바른
            # 토픽을 구독/발행한다.
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            SetRemap(src='/map', dst='map'),

            # has_slam_toolbox_params가 거짓일 때만: slam_params_file을 넘기지
            # 않아 slam_toolbox 자체 기본 파라미터를 쓰게 한다.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch_file),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
                condition=UnlessCondition(has_slam_toolbox_params),
            ),

            # has_slam_toolbox_params가 참일 때만: params_file을 slam_params_file로
            # 그대로 전달(위 include와 배타적).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch_file),
                launch_arguments={'use_sim_time': use_sim_time,
                                  'slam_params_file': params_file}.items(),
                condition=IfCondition(has_slam_toolbox_params),
            )
        ]
    )

    ld = LaunchDescription()

    # 이 launch 파일이 받는 인자들을 선언.
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 지도 저장 서비스 서버 실행.
    ld.add_action(start_map_server)

    # SLAM 실행 — has_slam_toolbox_params 조건에 따라 둘 중 하나만 실제로 뜬다.
    ld.add_action(start_slam_toolbox_cmd)

    return ld
