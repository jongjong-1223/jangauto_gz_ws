#!/usr/bin/env python3
"""진단(diagnostics)→미션 에러 연결 노드.

## 역할
- `diagnostic_aggregator`가 집계해서 내보내는 `/diagnostics_agg`
  (`diagnostic_msgs/DiagnosticArray`)를 구독한다.
- `included_groups`(코드 기본값 `['GPS']` — 실제 운용값은
  `mission.launch.py`에서 `['GPS', 'Localization', 'Nav2', 'Control']`로
  override)에 속한 상태 중 하나라도 `min_level_to_stop`(기본 ERROR) 이상이면
  "나쁜 상태"로 판단해 `/jangauto_mission/error`(`mission_state_machine.py`가
  구독 — 수신 시 무조건 STOP으로 강제 전이)로 사유를 발행한다.
- GPS 유실, EKF 로컬라이제이션 이상(Localization), nav2 서버 다운(Nav2),
  cmd_vel 파이프라인 끊김(Control)은 전부 대체 안전장치가 없어서 포함.
- HMI 그룹(app_websocket_bridge)은 계속 제외 — 앱은 선택 장비라 로봇은 앱
  없이도 동작해야 한다. "앱 연결 안 됨"은 정상 운용 상태 중 하나일 뿐이라
  STOP 사유가 아니다(mission_state_machine.py도 같은 이유로 명령 침묵을
  에러로 취급하지 않음).
- 엣지 트리거로만 발행한다(나쁜 상태가 유지되는 동안 반복 발행하지 않음) —
  안 그러면 aggregator가 주기적으로 재발행할 때마다 STOP 자기전이가
  계속 반복된다. 나쁜 상태에서 벗어나면 빈 문자열을 발행해 내부
  `_last_in_error` 플래그만 리셋한다(강제 전이는 없음 — 실제 복귀는 앱이
  새 명령을 보내야 일어나는 기존 안전 설계와 동일).
- `startup_grace_period_sec`(기본 20.0초) 동안은 들어오는 `/diagnostics_agg`를
  전부 무시한다 — 전체 스택을 한 번에 부팅하면 GPS/Nav2 bond/cmd_vel
  파이프라인이 첫 데이터를 내기 전 몇 초간 ERROR/STALE이 정상적으로 스쳐
  지나가는데, 이 노드가 그 찰나를 그대로 잡으면 `mission_state_machine`이
  부팅 직후 영구 에러 래치(물리 E-stop과 동급, 노드 재시작 전엔 안 풀림)에
  걸려버린다. 유예시간은 노드 실행 시각 기준(첫 콜백 시각 아님)이라, 이
  노드 자체가 늦게 뜨는 경우엔 그만큼 유예도 짧아진다는 점 유의.
"""

import time

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
        # DiagnosticStatus.ERROR는 생성 코드상 bytes(b'\x02') 타입이라 declare_parameter의
        # 타입 추론에 안 맞아 TypeError로 죽는다 — 단일 바이트를 인덱싱하면 파이썬3에서
        # 바로 int가 나오므로(int(b'\x02')처럼 10진 문자열로 파싱 시도하면 오히려 에러) 이 방식으로 캐스팅.
        self.declare_parameter('min_level_to_stop', DiagnosticStatus.ERROR[0])
        self.declare_parameter('startup_grace_period_sec', 20.0)

        self._included_groups = self.get_parameter(
            'included_groups').get_parameter_value().string_array_value
        self._min_level_to_stop = self.get_parameter(
            'min_level_to_stop').get_parameter_value().integer_value
        self._startup_grace_period_sec = self.get_parameter(
            'startup_grace_period_sec').get_parameter_value().double_value

        self._error_pub = self.create_publisher(String, ERROR_TOPIC, 10)
        self._is_bad = False
        self._start_monotonic = time.monotonic()

        self.create_subscription(
            DiagnosticArray, DIAGNOSTICS_AGG_TOPIC, self._on_diagnostics_agg, 10)

        self.get_logger().info(
            f'[MissionDiagMonitor] Watching groups {list(self._included_groups)} '
            f'for level >= {self._min_level_to_stop}')

    def _on_diagnostics_agg(self, msg: DiagnosticArray) -> None:
        """`diagnostic_analyzers.yaml`의 `contains` 매칭과 같은 방식(부분 문자열
        매칭)으로 관심 그룹에 속하면서 임계 레벨 이상인 status를 골라낸다.
        `status.level`은 ROS `byte` 타입이라 파이썬에서 길이 1 `bytes`로
        들어오므로, 정수인 `self._min_level_to_stop`과 비교하려면 인덱싱해서
        int로 꺼내야 한다."""
        if (time.monotonic() - self._start_monotonic) < self._startup_grace_period_sec:
            return

        offending = [
            status for status in msg.status
            if status.level[0] >= self._min_level_to_stop
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
