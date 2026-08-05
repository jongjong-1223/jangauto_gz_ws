"""실제 UWB 관련 노드 전체를 실행하는 launch 파일(실기체 전용).

- `uwb_publisher_real`: 시리얼 UWB 태그 위치 -> `/abs_xy`.
- `uwb_cov_filter_real`: 회전 중엔 UWB를 무시하도록 covariance 동적 조정 ->
  `/abs_xy_fixed`(jangauto_perception ekf_global의 pose0 입력).
- `uwb_virtual_map_publisher`: 시뮬용으로 만들어진(`_simul` 접미사) 노드지만,
  실기체에서도 그대로 재사용한다 — UWB가 이번에 측위(localization) 용도로
  쓰이게 되면서, "실제 UWB 하드웨어 전까지"라던 `/map` 가상 발행 역할을 대체할
  진짜 맵 소스가 아직 없기 때문. 실제 매핑 솔루션이 생기면 이 노드를 그걸로
  교체해야 한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mark_border_occupied_arg = DeclareLaunchArgument(
        'mark_border_occupied', default_value='true',
        description='가장 바깥 1셀 테두리를 occupied(100)로 채울지 여부(안쪽은 항상 free).')

    uwb_publisher_node = Node(
        package="jangauto_uwb_driver",
        executable="uwb_publisher_real.py",
        name="uwb_publisher_real",
        output="screen",
    )

    uwb_cov_filter_node = Node(
        package="jangauto_uwb_driver",
        executable="uwb_cov_filter_real.py",
        name="uwb_cov_filter_real",
        output="screen",
    )

    uwb_virtual_map_node = Node(
        package="jangauto_uwb_driver",
        executable="uwb_virtual_map_publisher_simul.py",
        name="uwb_virtual_map_publisher",
        output="screen",
        parameters=[{
            'mark_border_occupied': LaunchConfiguration('mark_border_occupied'),
        }],
    )

    return LaunchDescription([
        mark_border_occupied_arg,
        uwb_publisher_node,
        uwb_cov_filter_node,
        uwb_virtual_map_node,
    ])
