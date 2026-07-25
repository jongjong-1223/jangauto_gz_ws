"""애플리케이션 계층 노드 launch 파일.

- `pure_pursuit_node`: 경로 추종(pure pursuit) 제어기.
- `slip_plot_node`: 바퀴 슬립 대 시뮬레이션 시간 플로팅 노드.
- `twist_mux_node`: 여러 `cmd_vel` 소스의 우선순위를 중재하는 `twist_mux`.

현재 세 노드 모두 `LaunchDescription`에 등록되지 않고 주석 처리되어 있다 —
즉 이 launch 파일을 include해도 실제로는 아무 노드도 뜨지 않는다.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_application = get_package_share_directory('jangauto_application')

    pure_pursuit_node = Node(
        package='jangauto_application',
        executable='pure_pursuit_controller.py',
        name='pure_pursuit_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    slip_plot_node = Node(
        package='jangauto_application',
        executable='plot_slip_vs_sim_time.py',
        name='slip_plot_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # twist_mux_node for priority of cmd_vel
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(pkg_project_application, 'config', 'twist_mux.yaml')],
        output='screen',
    )

    return LaunchDescription([
        # pure_pursuit_node,
        # slip_plot_node,
        # twist_mux_node,
    ])
