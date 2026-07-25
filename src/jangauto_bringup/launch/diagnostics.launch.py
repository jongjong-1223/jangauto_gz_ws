"""`diagnostic_aggregator`의 `aggregator_node` 하나를 실행하는 launch 파일.

- `config/diagnostic_analyzers.yaml`을 파라미터로 넘겨서, 커스텀 노드들이
  `diagnostic_updater`로 내보내는 `/diagnostics`를 그룹별로 집계해
  `/diagnostics_agg`로 발행하게 한다.
- 집계 그룹(HMI/GPS) 정의는 전부 그 yaml 쪽에 있고, 이 launch 파일은
  단순히 노드를 띄우고 파라미터 파일 경로만 연결한다.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    analyzers_yaml = os.path.join(
        get_package_share_directory('jangauto_bringup'), 'config', 'diagnostic_analyzers.yaml')

    aggregator_node = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        parameters=[analyzers_yaml],
    )

    return LaunchDescription([
        aggregator_node,
    ])
