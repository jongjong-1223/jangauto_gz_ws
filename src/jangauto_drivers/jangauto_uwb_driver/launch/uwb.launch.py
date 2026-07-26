"""`uwb_virtual_map_publisher` 노드 하나를 실행하는 launch 파일.

실제 UWB 하드웨어/알고리즘이 준비되기 전까지, nav2의 `global_costmap`
`static_layer`가 참조할 `/map`을 가상 데이터로 대신 채워 넣는다.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    uwb_virtual_map_node = Node(
        package="jangauto_uwb_driver",
        executable="uwb_virtual_map_publisher.py",
        name="uwb_virtual_map_publisher",
        output="screen",
    )

    return LaunchDescription([
        uwb_virtual_map_node,
    ])
