# Copyright (c) 2018 Intel Corporation
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

"""map_server + amcl로 (SLAM 없이) 기존 지도 위에서 위치추정을 담당하는 launch 파일.

**참고용 — 미사용.** 이 프로젝트는 AMCL 대신 GPS+IMU EKF(jangauto_perception)로
map->odom->base_link TF를 직접 발행한다(`nav2_params_simul.yaml` 헤더 주석 참고). 이 파일은
stock nav2_bringup 구조 문서화 및 향후 참고용으로만 `launch/references/`에 보관한다.

## 역할
- `map_server`: `map` 인자가 빈 문자열이면 파라미터 파일에 설정된 기본 지도를,
  값이 있으면 그 경로의 지도 yaml을 강제로 덮어써서 로드한다
  (`EqualsSubstitution`/`NotEqualsSubstitution` 조건으로 둘 중 하나만 실제 실행됨).
- `amcl`: 그 지도 위에서 파티클 필터로 로봇 위치를 추정한다.
- `lifecycle_manager_localization`: 위 두 노드(`map_server`, `amcl`)의 lifecycle을
  `autostart` 값에 따라 일괄 관리(configure/activate)한다.
- `use_composition`에 따라 위 3개 노드를 실행하는 방식이 갈린다:
  - False -> 각각 독립 프로세스(`Node`)로 실행(`load_nodes`).
  - True  -> `container_name`으로 지정된 컨테이너에 컴포저블 노드로 로드
    (`load_composable_nodes`) — 이 launch가 자체 컨테이너를 새로 띄우지는
    않고, bringup_launch.py 등 상위에서 미리 띄운 컨테이너를 재사용한다.

## 선언하는 launch 인자(그룹별)
- 네임스페이스/지도: `namespace`, `map`
- 공통 실행 옵션: `use_sim_time`, `params_file`, `autostart`, `use_composition`,
  `container_name`, `use_respawn`, `log_level`
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import NotEqualsSubstitution
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 기본 파라미터 파일(nav2_params_simul.yaml) 경로를 구성하는 데 쓰는 패키지 공유 디렉터리.
    bringup_dir = get_package_share_directory('jangauto_navigation2')

    # 아래 DeclareLaunchArgument들이 실제 값을 선언하며, 여기서는 참조 핸들만 만든다.
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # lifecycle_manager_localization이 관리할 lifecycle 노드 이름 목록.
    lifecycle_nodes = ['map_server', 'amcl']

    # 절대 경로(/tf, /tf_static)를 상대 경로로 바꿔서, 노드에 네임스페이스가
    # 붙을 때 tf 토픽도 그 네임스페이스 아래로 자동으로 들어가게 한다.
    # tf만 이렇게 별도 처리하는 이유는 geometry2/robot_state_publisher 쪽에
    # 아직 더 나은 대안이 없기 때문(TODO(orduno): PushNodeRemapping로 대체 예정).
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # params_file(yaml)을 실행 시점 값으로 재작성한 임시 파일로 감싸서 각 노드의
    # parameters로 넘긴다(root_key=namespace로 네임스페이스별 섹션을 선택).
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # 로그를 줄 단위로 즉시 flush해서, 표준출력이 리다이렉트돼도 로그가
    # 버퍼링되어 늦게 보이는 걸 방지한다.
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # 아래는 이 launch 파일이 받는 인자 선언들 — default_value/description은
    # 코드에 문서화돼 있으므로 여기서는 역할별로만 묶어 설명한다(모듈 docstring 참고).
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value='', description='Full path to map yaml file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params_simul.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    # use_composition=False일 때 실행 — 각 노드를 독립 프로세스로 띄운다.
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            # map 인자가 빈 문자열일 때만 실행: 파라미터 파일의 기본 지도 설정을 그대로 사용.
            Node(
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('map'), '')
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # map 인자가 비어있지 않을 때만 실행: yaml_filename 파라미터를 덮어써서
            # CLI/launch에서 넘긴 지도를 강제로 로드(위 노드와 배타적).
            Node(
                condition=IfCondition(
                    NotEqualsSubstitution(LaunchConfiguration('map'), '')
                ),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'yaml_filename': map_yaml_file}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # map_server + amcl의 lifecycle(configure/activate)을 autostart 값에
            # 따라 일괄 관리.
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )
    # use_composition=True일 때 실행 — 같은 노드들을 container_name 컨테이너에
    # 컴포저블 노드로 로드한다(새 컨테이너를 만들지 않고 기존 컨테이너를 재사용).
    # map_server용 LoadComposableNodes를 두 번 나눈 이유: 조건(IfCondition)이
    # LoadComposableNodes 액션 단위로만 걸리고 ComposableNode 자체에는 걸 수
    # 없기 때문 — load_nodes와 동일한 map 유무 분기를 재현하기 위함.
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('map'), '')
                ),
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                ],
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                condition=IfCondition(
                    NotEqualsSubstitution(LaunchConfiguration('map'), '')
                ),
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[
                            configured_params,
                            {'yaml_filename': map_yaml_file},
                        ],
                        remappings=remappings,
                    ),
                ],
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_amcl',
                        plugin='nav2_amcl::AmclNode',
                        name='amcl',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_localization',
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

    # LaunchDescription을 만들고 액션들을 순서대로 채운다.
    ld = LaunchDescription()

    # 환경변수 설정을 가장 먼저 적용.
    ld.add_action(stdout_linebuf_envvar)

    # 이 launch 파일이 받는 인자들을 선언.
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # use_composition 값에 따라 둘 중 하나만 조건이 참이 되어 실제로 실행된다.
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
