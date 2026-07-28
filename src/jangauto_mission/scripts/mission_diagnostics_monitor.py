#!/usr/bin/env python3
"""진단(diagnostics)→미션 에러 연결 노드.

## 역할
- `diagnostic_aggregator`가 집계해서 내보내는 `/diagnostics_agg`
  (`diagnostic_msgs/DiagnosticArray`)를 구독한다.
- `included_groups`(기본 `['GPS']`)에 속한 상태 중 하나라도
  `min_level_to_stop`(기본 ERROR) 이상이면 "나쁜 상태"로 판단해
  `/jangauto_mission/error`(`mission_state_machine.py`가 구독 — 수신 시
  무조건 STOP으로 강제 전이)로 사유를 발행한다.
- HMI 그룹(app_websocket_bridge)은 기본 제외 — "앱 연결 안 됨" WARN은
  정상적으로도 자주 뜨고, 애초에 앱이 끊기면 control_state도 안 와서
  mission_state_machine.py의 기존 명령 타임아웃 fail-safe가 이미 커버한다.
  GPS 유실은 그런 대체 안전장치가 없어서 우선 이것만 기본 포함.
- 엣지 트리거로만 발행한다(나쁜 상태가 유지되는 동안 반복 발행하지 않음) —
  안 그러면 aggregator가 주기적으로 재발행할 때마다 STOP 자기전이가
  계속 반복된다. 나쁜 상태에서 벗어나면 빈 문자열을 발행해 내부
  `_last_in_error` 플래그만 리셋한다(강제 전이는 없음 — 실제 복귀는 앱이
  새 명령을 보내야 일어나는 기존 안전 설계와 동일).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

DIAGNOSTICS_AGG_TOPIC = '/diagnostics_agg'
ERROR_TOPIC = '/jangauto_mission/error'


class MissionDiagnosticsMonitor(Node):
    """`/diagnostics_agg` -> `/jangauto_mission/error` 엣지 트리거 변환기."""

    def __init__(self):
        super().__init__('mission_diagnostics_monitor')

        self.declare_parameter('included_groups', ['GPS'])
        self.declare_parameter('min_level_to_stop', DiagnosticStatus.ERROR)

        self._included_groups = self.get_parameter(
            'included_groups').get_parameter_value().string_array_value
        self._min_level_to_stop = self.get_parameter(
            'min_level_to_stop').get_parameter_value().integer_value

        self._error_pub = self.create_publisher(String, ERROR_TOPIC, 10)
        self._is_bad = False

        self.create_subscription(
            DiagnosticArray, DIAGNOSTICS_AGG_TOPIC, self._on_diagnostics_agg, 10)

        self.get_logger().info(
            f'[MissionDiagMonitor] Watching groups {list(self._included_groups)} '
            f'for level >= {self._min_level_to_stop}')

    def _on_diagnostics_agg(self, msg: DiagnosticArray) -> None:
        """`diagnostic_analyzers.yaml`의 `contains` 매칭과 같은 방식(부분 문자열
        매칭)으로 관심 그룹에 속하면서 임계 레벨 이상인 status를 골라낸다."""
        offending = [
            status for status in msg.status
            if status.level >= self._min_level_to_stop
            and any(group in status.name for group in self._included_groups)
        ]

        if offending and not self._is_bad:
            self._is_bad = True
            reason = '; '.join(f'{s.name}: {s.message}' for s in offending)
            self.get_logger().warning(f'[MissionDiagMonitor] Forcing STOP: {reason}')
            self._publish_error(reason)
        elif not offending and self._is_bad:
            self._is_bad = False
            self.get_logger().info('[MissionDiagMonitor] Cleared')
            self._publish_error('')

    def _publish_error(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._error_pub.publish(msg)


def main(args=None):
    """노드 진입점 — `rclpy.spin()`으로 상주하며 `/diagnostics_agg`를 계속 감시한다."""
    rclpy.init(args=args)
    node = MissionDiagnosticsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
