"""미션 관련 노드들을 실행하는 launch 파일.

- `mission_state_machine`: YASMIN 상태머신(오케스트레이션).
- `calibration_action_server`/`align_action_server`/`run_action_server`: CAL/ALIGN/RUN
  상태가 호출하는 액션 서버(TODO placeholder — 실제 알고리즘 미정).
- `mission_diagnostics_monitor`: `/diagnostics_agg`의 실제 문제(기본 GPS 그룹)를
  `/jangauto_mission/error`로 연결해 mission_state_machine이 강제 STOP하게 함.

cmd_vel 관련 저수준 제어(`cmd_vel_arbiter`, `key_manual_driver`)는
`jangauto_control` 패키지로 분리됐다 — `tracked_v3.launch.py`가 이 launch 파일과
`jangauto_control/launch/control.launch.py`를 둘 다 include한다.
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

    mission_diagnostics_monitor_node = Node(
        package="jangauto_mission",
        executable="mission_diagnostics_monitor.py",
        name="mission_diagnostics_monitor",
        output="screen",
    )

    return LaunchDescription([
        mission_state_machine_node,
        calibration_action_server_node,
        align_action_server_node,
        run_action_server_node,
        mission_diagnostics_monitor_node,
    ])
