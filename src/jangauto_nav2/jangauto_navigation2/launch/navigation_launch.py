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

"""Nav2 내비게이션 서버 묶음(경로 추종/계획/행동/도킹 등)을 실행하는 launch 파일.

## 역할
- 아래 lifecycle 노드들을 띄우고 `lifecycle_manager_navigation`이 일괄
  configure/activate 한다:
  - `controller_server`: 로컬 경로 추종(costmap 기반 속도 명령 산출)
  - `smoother_server`: 전역 경로 스무딩
  - `planner_server`: 전역 경로 계획
  - `route_server`: 그래프 기반 라우팅
  - `behavior_server`: 회복 행동(예: 후진, 제자리 회전)
  - `bt_navigator`: 행동트리(BT)로 위 서버들을 조합해 내비게이션 태스크 수행
    (사용할 기본 BT XML은 `default_nav_to_pose_bt_xml`/
    `default_nav_through_poses_bt_xml`로 주입)
  - `waypoint_follower`: 다중 목표점 순회
  - `velocity_smoother`: 속도 명령을 가감속 한계 안에서 부드럽게 다듬음
  - `collision_monitor`: 근접 장애물 감지 시 속도 제한/정지
  - `docking_server`: 자동 도킹
- `controller_server`/`behavior_server`는 내부적으로 `cmd_vel` 토픽에 속도를
  내는데, 이를 `cmd_vel_nav`로 remap한다. `velocity_smoother`는 자신의 입력
  `cmd_vel`을 `cmd_vel_nav`로 remap해서 이 값을 받아 스무딩한 뒤, 노드 자체에
  고정된 출력 토픽 `cmd_vel_smoothed`로 발행한다(이건 remap이 아니라
  `nav2_velocity_smoother` 코드가 입력과 다른 이름으로 발행하도록 만들어져
  있는 것). `collision_monitor`는 이 `cmd_vel_smoothed`를 `cmd_vel_in_topic`
  파라미터(`nav2_params.yaml`)로 구독해서 근접 충돌 예측 시 감속시킨 뒤,
  `cmd_vel_out_topic` 파라미터로 지정된 `cmd_vel_nav_out`을 최종 발행한다 —
  즉 `controller_server`/`behavior_server` -> `cmd_vel_nav` ->
  `velocity_smoother` -> `cmd_vel_smoothed` -> `collision_monitor` ->
  `cmd_vel_nav_out` 순서의 파이프라인이다. 로봇 베이스로 바로 가는 게 아니라
  `cmd_vel_nav_out`이 `cmd_vel_arbiter.py`(jangauto_control)가 구독하는
  소스 중 하나로 들어가서, 거기서 다른 소스(수동조작/안전정지)와 중재된
  뒤에야 로봇 베이스행 `cmd_vel_out`이 된다.
- `use_composition`에 따라 실행 방식이 갈린다:
  - False -> 각 서버를 독립 프로세스(`Node`)로 실행(`load_nodes`).
  - True  -> `container_name` 컨테이너에 컴포저블 노드로 로드
    (`load_composable_nodes`, 새 컨테이너를 만들지 않고 재사용).

## 선언하는 launch 인자(그룹별)
- 네임스페이스: `namespace`
- 공통 실행 옵션: `use_sim_time`, `params_file`, `autostart`, `use_composition`,
  `container_name`, `use_respawn`, `log_level`
- 행동트리(BT) XML 경로: `default_nav_to_pose_bt_xml`,
  `default_nav_through_poses_bt_xml`
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 기본 파라미터 파일(nav2_params.yaml)/BT XML 경로를 구성하는 데 쓰는 공유 디렉터리.
    bringup_dir = get_package_share_directory('jangauto_navigation2')

    # 아래 DeclareLaunchArgument들이 실제 값을 선언하며, 여기서는 참조 핸들만 만든다.
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml')

    # lifecycle_manager_navigation이 일괄 관리할 lifecycle 노드 이름 목록.
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
        'docking_server',
    ]

    # 절대 경로(/tf, /tf_static)를 상대 경로로 바꿔서, 노드에 네임스페이스가
    # 붙을 때 tf 토픽도 그 네임스페이스 아래로 자동으로 들어가게 한다.
    # tf만 이렇게 별도 처리하는 이유는 geometry2/robot_state_publisher 쪽에
    # 아직 더 나은 대안이 없기 때문(TODO(orduno): PushNodeRemapping로 대체 예정).
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # params_file(yaml)에 launch 인자 값을 주입해 임시 파일로 재작성한다.
    # bt_navigator 항목은 fully-qualified 키('<노드>.ros__parameters.<파라미터>')로
    # 줘야 실제로 삽입된다 — RewrittenYaml.add_params()는 이 형태가 아니면
    # 원본 yaml에 해당 키가 이미 없는 한 아무 효과가 없다(단순 이름만으로는 no-op).
    param_substitutions = {
        'autostart': autostart,
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml':
            default_nav_through_poses_bt_xml,
    }

    # 위에서 만든 param_substitutions를 반영한 최종 파라미터를 각 노드에 전달.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
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

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
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

    declare_default_nav_to_pose_bt_xml_cmd = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=os.path.join(bringup_dir, 'behavior_trees', 'navigate_to_pose.xml'),
        description='Full path to the behavior tree xml file to use for navigate_to_pose',
    )

    declare_default_nav_through_poses_bt_xml_cmd = DeclareLaunchArgument(
        'default_nav_through_poses_bt_xml',
        default_value=os.path.join(bringup_dir, 'behavior_trees', 'navigate_through_poses.xml'),
        description='Full path to the behavior tree xml file to use for navigate_through_poses',
    )

    # use_composition=False일 때 실행 — 모듈 docstring에 정리된 10개 서버 +
    # lifecycle_manager를 각각 독립 프로세스로 띄운다. 구성 패턴이 동일하게
    # 반복되므로(패키지/파라미터/respawn/remap), 눈에 띄는 예외만 아래에 표시한다.
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            # cmd_vel -> cmd_vel_nav remap: 내부 출력 토픽명을 바꿔서, 아래
            # velocity_smoother가 이 값을 입력으로 받아 스무딩한 뒤 최종
            # cmd_vel로 내보내는 파이프라인을 구성한다(모듈 docstring 참고).
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # controller_server와 마찬가지로 cmd_vel -> cmd_vel_nav remap
            # (회복 행동 중 속도 명령도 같은 스무딩 파이프라인을 거치게 함).
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # 이번엔 반대 방향 remap: velocity_smoother의 "입력" 토픽명(cmd_vel)을
            # cmd_vel_nav로 바꿔서, 위 controller_server/behavior_server가 내보낸
            # cmd_vel_nav를 실제로 구독하게 한다. 출력은 remap하지 않은 그대로
            # 'cmd_vel'로 나가며, 이게 로봇 베이스가 최종적으로 구독하는 토픽이다.
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
                + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='opennav_docking',
                executable='opennav_docking',
                name='docking_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # 위 10개 서버의 lifecycle을 autostart 값에 따라 일괄 관리.
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    # use_composition=True일 때 실행 — 위와 동일한 노드 구성을 container_name
    # 컨테이너에 컴포저블 노드로 로드한다(새 컨테이너를 만들지 않고 재사용).
    # cmd_vel remap 방향/의미는 load_nodes 쪽 주석과 동일하다.
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_route',
                        plugin='nav2_route::RouteServer',
                        name='route_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params],
                        remappings=remappings
                        + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_collision_monitor',
                        plugin='nav2_collision_monitor::CollisionMonitor',
                        name='collision_monitor',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='opennav_docking',
                        plugin='opennav_docking::DockingServer',
                        name='docking_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
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
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_default_nav_to_pose_bt_xml_cmd)
    ld.add_action(declare_default_nav_through_poses_bt_xml_cmd)
    # use_composition 값에 따라 둘 중 하나만 조건이 참이 되어 실제로 실행된다.
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
