"""저수준 cmd_vel 제어 노드들을 실행하는 launch 파일.

- `cmd_vel_arbiter`: nav2/수동조작/안전정지 cmd_vel 중재(`/robot_status` 모드 기준).
- `key_manual_driver`: 조이스틱(key_bits/speed_bits) -> `cmd_vel_manual` 변환. 아래
  launch argument들은 이 노드의 속도 단계/타임아웃 파라미터로 그대로 전달된다(실제
  현장 테스트 후 조정할 수 있도록 노출 — 기본값은 `key_manual_driver.py` 자체의
  `declare_parameter` 기본값과 같아야 함).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_state_stale_timeout_arg = DeclareLaunchArgument(
        'control_state_stale_timeout_sec', default_value='1.0',
        description='key_manual_driver: /app/control_state 수신이 이보다 오래 끊기면 강제 정지.')
    speed_low_linear_arg = DeclareLaunchArgument('speed_low_linear', default_value='0.25')
    speed_low_angular_arg = DeclareLaunchArgument('speed_low_angular', default_value='0.3')
    speed_normal_linear_arg = DeclareLaunchArgument('speed_normal_linear', default_value='0.5')
    speed_normal_angular_arg = DeclareLaunchArgument('speed_normal_angular', default_value='0.6')
    speed_high_linear_arg = DeclareLaunchArgument('speed_high_linear', default_value='0.75')
    speed_high_angular_arg = DeclareLaunchArgument('speed_high_angular', default_value='0.9')

    cmd_vel_arbiter_node = Node(
        package="jangauto_control",
        executable="cmd_vel_arbiter.py",
        name="cmd_vel_arbiter",
        output="screen",
    )

    key_manual_driver_node = Node(
        package="jangauto_control",
        executable="key_manual_driver.py",
        name="key_manual_driver",
        output="screen",
        parameters=[{
            'control_state_stale_timeout_sec': LaunchConfiguration('control_state_stale_timeout_sec'),
            'speed_low_linear': LaunchConfiguration('speed_low_linear'),
            'speed_low_angular': LaunchConfiguration('speed_low_angular'),
            'speed_normal_linear': LaunchConfiguration('speed_normal_linear'),
            'speed_normal_angular': LaunchConfiguration('speed_normal_angular'),
            'speed_high_linear': LaunchConfiguration('speed_high_linear'),
            'speed_high_angular': LaunchConfiguration('speed_high_angular'),
        }],
    )

    return LaunchDescription([
        control_state_stale_timeout_arg,
        speed_low_linear_arg,
        speed_low_angular_arg,
        speed_normal_linear_arg,
        speed_normal_angular_arg,
        speed_high_linear_arg,
        speed_high_angular_arg,
        cmd_vel_arbiter_node,
        key_manual_driver_node,
    ])
