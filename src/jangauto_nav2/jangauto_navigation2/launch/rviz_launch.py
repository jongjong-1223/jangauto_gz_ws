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

"""rviz2를 실행해 Nav2 시각화(지도/tf/경로/goal 등)를 제공하는 launch 파일.

## 역할
- `use_namespace`에 따라 둘 중 하나의 rviz2 노드만 조건부로 실행된다:
  - False -> `start_rviz_cmd`: 네임스페이스 없이, 주어진 rviz 설정 파일을 그대로 사용.
  - True  -> `start_namespaced_rviz_cmd`: rviz 설정 파일 안의 `<robot_namespace>`
    플레이스홀더를 실제 `namespace` 값으로 치환하고, 노드 자체도 그 네임스페이스로
    실행하며, `/map`·`/tf`·`/tf_static`·`/goal_pose`·`/clicked_point`·
    `/initialpose` 같은 절대 토픽을 상대 경로로 remap해 네임스페이스 아래로 들어가게 한다.
- rviz2 프로세스가 종료되면(사용자가 창을 닫는 등) `OnProcessExit` 이벤트 핸들러가
  전체 launch에 `Shutdown` 이벤트를 발생시켜 같이 종료되게 한다 — rviz만 남고
  나머지 노드들이 계속 떠있는 상태를 방지하기 위함.

## 선언하는 launch 인자
- `namespace`: rviz 설정 파일의 `<robot_namespace>` 치환 및 노드 네임스페이스에 사용.
- `use_namespace`: 위 두 실행 방식 중 어느 쪽을 쓸지 결정.
- `rviz_config`: 사용할 rviz 설정 파일 경로.
- `use_sim_time`: 시뮬레이션(Gazebo) 시간 사용 여부.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # 기본 rviz 설정 파일(nav2_default_view.rviz) 경로를 구성하는 데 쓰는 공유 디렉터리.
    bringup_dir = get_package_share_directory('jangauto_navigation2')

    # 아래 DeclareLaunchArgument들이 실제 값을 선언하며, 여기서는 참조 핸들만 만든다.
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 이 launch 파일이 받는 인자 선언(각 default_value/description은 모듈 docstring 참고).
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='navigation',
        description=(
            'Top-level namespace. The value will be used to replace the '
            '<robot_namespace> keyword on the rviz config file.'
        ),
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # use_namespace가 거짓일 때만 실행 — 네임스페이스 없는 일반 rviz2.
    start_rviz_cmd = Node(
        condition=UnlessCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # rviz 설정 파일 안의 '<robot_namespace>' 플레이스홀더를 실제 namespace 값으로 치환.
    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={'<robot_namespace>': ('/', namespace)},
    )

    # use_namespace가 참일 때만 실행 — 노드를 namespace 아래로 띄우고,
    # 절대 토픽들을 상대 경로로 remap해 같은 네임스페이스로 들어가게 한다.
    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', namespaced_rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[
            ('/map', 'map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose'),
        ],
    )

    # rviz 프로세스가 종료되면 launch 전체를 함께 종료시킨다(각각 자신이
    # 담당하는 rviz 노드 쪽에만 조건부로 등록).
    exit_event_handler = RegisterEventHandler(
        condition=UnlessCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    # LaunchDescription을 만들고 액션들을 순서대로 채운다.
    ld = LaunchDescription()

    # 이 launch 파일이 받는 인자들을 선언.
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # use_namespace 값에 따라 둘 중 하나만 조건이 참이 되어 실제로 실행된다.
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_namespaced_rviz_cmd)

    # rviz 종료 시 전체 launch를 종료시키는 이벤트 핸들러 등록.
    ld.add_action(exit_event_handler)
    ld.add_action(exit_event_handler_namespaced)

    return ld
