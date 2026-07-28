"""`uwb_virtual_map_publisher` 노드 하나를 실행하는 launch 파일.

실제 UWB 하드웨어/알고리즘이 준비되기 전까지, nav2의 `global_costmap`
`static_layer`가 참조할 `/map`을 가상 데이터로 대신 채워 넣는다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mark_border_occupied_arg = DeclareLaunchArgument(
        'mark_border_occupied', default_value='true',
        description='가장 바깥 1셀 테두리를 occupied(100)로 채울지 여부(안쪽은 항상 free).')

    uwb_virtual_map_node = Node(
        package="jangauto_uwb_driver",
        executable="uwb_virtual_map_publisher.py",
        name="uwb_virtual_map_publisher",
        output="screen",
        parameters=[{
            'mark_border_occupied': LaunchConfiguration('mark_border_occupied'),
        }],
    )

    return LaunchDescription([
        mark_border_occupied_arg,
        uwb_virtual_map_node,
    ])
