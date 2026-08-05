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

"""Nav2 스택 전체를 한 번에 띄우는 최상위 bringup launch 파일.

**참고용 — 이 프로젝트의 실제 bringup(`jangauto_bringup/launch/tracked_v3_simul.launch.py`)은
이 파일을 쓰지 않는다.** 로컬라이제이션이 GPS+IMU EKF로 대체되어 AMCL/SLAM이 필요
없어졌고(`nav2_params_simul.yaml` 헤더 주석 참고), 실제로는 `navigation_launch.py`만 직접
include한다. 이 파일과 그것이 참조하는 `localization_launch.py`/`slam_launch.py`는
stock nav2_bringup 구조 문서화 및 향후 참고용으로만 `launch/references/`에 보관한다.

## 역할
- `use_composition`이 참이면 `nav2_container`(component_container_isolated)를
  하나 띄우고, 지도/위치추정/내비게이션 서버들을 그 안에 컴포저블 노드로 로드한다
  (프로세스 1개로 묶어 통신 오버헤드를 줄이는 nav2 표준 구성).
- `slam`/`use_localization` 조합에 따라 지도 관련 서브 launch를 배타적으로 include:
  - `slam=True` and `use_localization=True` -> `slam_launch.py` (SLAM으로 지도 생성)
  - `slam=False` and `use_localization=True` -> `localization_launch.py`
    (`map_server` + `amcl`로 기존 지도에서 위치추정)
  - `use_localization=False`면 둘 다 건너뛴다(외부에서 위치를 준다고 가정).
- `navigation_launch.py`(controller/planner/bt_navigator 등 내비게이션 서버 묶음)는
  위 조합과 무관하게 항상 include된다.
- `use_namespace`가 참이면 전체 그룹에 네임스페이스를 씌운다(`PushROSNamespace`).

## 선언하는 launch 인자(그룹별)
- 네임스페이스: `namespace`, `use_namespace`
- 지도/위치추정 모드: `slam`, `map`, `use_localization`
- 공통 실행 옵션: `use_sim_time`, `params_file`, `autostart`, `use_composition`,
  `use_respawn`, `log_level`
- 내비게이션 행동트리(BT) XML 경로: `default_nav_to_pose_bt_xml`,
  `default_nav_through_poses_bt_xml`
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # 이 패키지의 launch 디렉터리 — 하위 slam/localization/navigation launch 파일을
    # include할 때 경로 조합에 쓴다. slam/localization은 미사용 참고용이라
    # launch/references/에, navigation은 실제로 쓰이는 launch/에 그대로 있다.
    bringup_dir = get_package_share_directory('jangauto_navigation2')
    launch_dir = os.path.join(bringup_dir, 'launch')
    references_dir = os.path.join(launch_dir, 'references')

    # 아래 DeclareLaunchArgument들이 실제로 값을 선언하며, 여기서는 그 값을
    # 나중에 참조하기 위한 핸들만 만든다(선언 자체는 함수 하단에 모아 있음).
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_localization = LaunchConfiguration('use_localization')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml')

    # 절대 경로(/tf, /tf_static)를 상대 경로로 바꿔서, 노드에 네임스페이스가
    # 붙을 때 tf 토픽도 그 네임스페이스 아래로 자동으로 들어가게 한다.
    # tf만 이렇게 별도 처리하는 이유는 geometry2/robot_state_publisher 쪽에
    # 아직 더 나은 대안이 없기 때문(TODO(orduno): PushNodeRemapping로 대체 예정).
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # use_namespace가 참일 때만 적용: 파라미터 yaml 안의 '<robot_namespace>'
    # 플레이스홀더를 실제 namespace 값으로 치환한다. 멀티로봇 파라미터 파일
    # (nav2_multirobot_params.yaml)이 이 키워드를 쓰는 걸 전제로 한 기능이며,
    # 사용자 정의 파라미터 파일도 치환을 받으려면 같은 키워드를 넣어둬야 한다.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace),
    )

    # params_file(yaml)을 실행 시점 값으로 재작성한 임시 파일로 감싸서,
    # 아래 nav2_container 노드의 parameters로 그대로 넘길 수 있게 한다.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # 로그를 줄 단위로 즉시 flush해서, 표준출력이 파이프로 리다이렉트돼도
    # (예: launch 로그 파일로) 로그가 버퍼링되어 늦게 보이는 걸 방지한다.
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # 아래는 이 launch 파일이 받는 인자 선언들 — 각 default_value/description은
    # 코드에 그대로 문서화돼 있으므로, 여기서는 역할별로만 묶어서 설명한다
    # (자세한 그룹 구분은 모듈 docstring의 "선언하는 launch 인자" 참고).
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value='', description='Full path to map yaml file to load'
    )

    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization', default_value='True',
        description='Whether to enable localization or not'
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
        default_value='True',
        description='Whether to use composed bringup',
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

    # 실제로 실행되는 액션들을 하나의 그룹으로 묶는다 — 이렇게 묶어야
    # PushROSNamespace가 그룹 내부의 모든 노드/include에 함께 적용된다.
    bringup_cmd_group = GroupAction(
        [
            # use_namespace가 참일 때만 이 그룹 전체에 네임스페이스를 씌운다.
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            # use_composition이 참일 때만 컴포저블 컨테이너를 띄운다 — 이 컨테이너에
            # slam/localization/navigation의 노드들이 조건부로 로드된다(각 서브
            # launch의 use_composition/container_name 인자로 대상 컨테이너를 지정).
            Node(
                condition=IfCondition(use_composition),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': autostart}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                output='screen',
            ),
            # slam=True AND use_localization=True 일 때만: SLAM으로 지도를
            # 만들며 동시에 위치추정도 겸한다(localization_launch와 배타적).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(references_dir, 'slam_launch.py')
                ),
                condition=IfCondition(PythonExpression([slam, ' and ', use_localization])),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'use_respawn': use_respawn,
                    'params_file': params_file,
                }.items(),
            ),
            # slam=False AND use_localization=True 일 때만: 기존 지도(map_yaml_file)
            # 위에서 map_server + amcl로 위치추정한다(slam_launch와 배타적).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(references_dir, 'localization_launch.py')
                ),
                condition=IfCondition(PythonExpression(['not ', slam, ' and ', use_localization])),
                launch_arguments={
                    'namespace': namespace,
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                }.items(),
            ),
            # 조건 없이 항상 include — controller/planner/behavior/bt_navigator 등
            # 내비게이션 서버 묶음은 slam/localization 모드와 무관하게 항상 필요하다.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                    'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
                    'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
                }.items(),
            ),
        ]
    )

    # LaunchDescription을 만들고 액션들을 순서대로 채운다.
    ld = LaunchDescription()

    # 환경변수 설정을 가장 먼저 적용.
    ld.add_action(stdout_linebuf_envvar)

    # 이 launch 파일이 받는 인자들을 선언.
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_localization_cmd)
    ld.add_action(declare_default_nav_to_pose_bt_xml_cmd)
    ld.add_action(declare_default_nav_through_poses_bt_xml_cmd)

    # 위에서 구성한 실제 실행 그룹(컨테이너 + 조건부 include들)을 추가.
    ld.add_action(bringup_cmd_group)

    return ld
