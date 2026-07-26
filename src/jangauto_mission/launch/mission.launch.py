"""미션 관련 노드들을 실행하는 launch 파일.

- `mission_state_machine`: YASMIN 상태머신(오케스트레이션).
- `cmd_vel_arbiter`: nav2/수동조작/안전정지 cmd_vel 중재.
- `calibration_action_server`/`align_action_server`/`run_action_server`: CAL/ALIGN/RUN
  상태가 호출하는 액션 서버(TODO placeholder — 실제 알고리즘 미정).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_state_machine_node = Node(
        package="jangauto_mission",
        executable="mission_state_machine.py",
        name="mission_state_machine",
        output="screen",
    )

    cmd_vel_arbiter_node = Node(
        package="jangauto_mission",
        executable="cmd_vel_arbiter.py",
        name="cmd_vel_arbiter",
        output="screen",
    )

    calibration_action_server_node = Node(
        package="jangauto_mission",
        executable="calibration_action_server.py",
        name="calibration_action_server",
        output="screen",
    )

    align_action_server_node = Node(
        package="jangauto_mission",
        executable="align_action_server.py",
        name="align_action_server",
        output="screen",
    )

    run_action_server_node = Node(
        package="jangauto_mission",
        executable="run_action_server.py",
        name="run_action_server",
        output="screen",
    )

    return LaunchDescription([
        mission_state_machine_node,
        cmd_vel_arbiter_node,
        calibration_action_server_node,
        align_action_server_node,
        run_action_server_node,
    ])
