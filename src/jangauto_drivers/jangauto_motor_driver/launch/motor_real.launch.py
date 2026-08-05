"""`motor_cmd_vel_trx` 노드 하나를 실행하는 launch 파일(실기체 전용).

cmd_vel_out을 시리얼로 실제 모터 컨트롤러에 전달한다 — 시뮬레이션에서는
Gazebo diff_drive 플러그인이 이 역할을 대신하므로 _simul 대응 파일이 없다.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_cmd_vel_node = Node(
        package="jangauto_motor_driver",
        executable="motor_cmd_vel_real.py",
        name="motor_cmd_vel_trx",
        output="screen",
    )

    return LaunchDescription([
        motor_cmd_vel_node,
    ])
