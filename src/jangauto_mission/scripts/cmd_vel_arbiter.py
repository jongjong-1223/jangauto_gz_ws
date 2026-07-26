#!/usr/bin/env python3
"""`cmd_vel` 중재 노드.

## 역할
- 로봇을 움직이려는 소스가 여러 개(nav2 자율주행, 앱 수동조작, 안전정지)일 때
  그중 정확히 하나만 골라 실제 출력(`cmd_vel_out`)으로 내보낸다 — 이게 없으면
  여러 소스가 동시에 `cmd_vel`을 발행해 서로 덮어쓰며 충돌한다.
- 범용 `twist_mux`(우선순위+타임아웃만 봄)와 달리, `mission_state_machine.py`가
  이미 발행 중인 `/robot_status`(현재 모드)를 그대로 재사용해 미션 상태에
  맞는 소스만 통과시킨다 — RUN이 아니면 nav2 명령을 걸러내는 식.
- `cmd_vel_stop`은 모드와 무관하게 항상 최우선 — 안전정지 신호가 들어오면
  그 즉시 정지 명령으로 덮어쓴다.

## 입력/출력
- 구독: `cmd_vel_nav_out`(nav2 최종 출력), `cmd_vel_manual`(앱 수동조작 변환값,
  아직 이 토픽을 발행하는 노드는 없음 — 존재한다고 가정만 함), `cmd_vel_stop`
  (안전정지 트리거, 내용은 안 보고 수신 자체만 봄), `/robot_status`(현재 모드).
- 발행: `cmd_vel_out` — `jangauto_bridge.yaml`이 이미 보고 있는 이름이라
  시뮬레이션 로봇까지 배선 그대로 이어진다.

## main()의 동작 순서
1. rclpy 초기화
2. `CmdVelArbiter` 노드 생성 — 이 시점에 4개 구독과 발행자, 타이머가 생성됨
3. `rclpy.spin()` — 타이머 주기(`ARBITRATION_RATE_HZ`)마다 `_arbitrate()`가
   최신 수신 상태를 보고 출력을 결정, Ctrl+C 전까지 계속 반복
4. 종료 시 정리
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist

from jangauto_msg.msg import Status

# 주행이 허용되는 모드 -> 그 모드에서 통과시킬 입력 토픽 이름.
# 나머지 모드(STOP/CAL/ALIGN)는 목록에 없으므로 항상 정지.
MODE_TO_SOURCE_TOPIC = {
    "RUN": "cmd_vel_nav_out",
    "KEY": "cmd_vel_manual",
}

# 이 시간(초) 안에 메시지가 안 들어오면 그 소스는 "죽은 것"으로 간주 —
# nav2(controller_frequency 20Hz)/수동조작 모두 훨씬 짧은 주기로 계속 발행되는
# 것을 전제로 하므로, 이 값보다 늦으면 명령이 아니라 침묵으로 본다(fail-safe).
SOURCE_TIMEOUT_SEC = 0.5
ARBITRATION_RATE_HZ = 20.0


class CmdVelArbiter(Node):
    """`/robot_status` 모드 기준으로 3개 cmd_vel 입력 중 하나만 통과시키는 노드."""

    def __init__(self):
        super().__init__('cmd_vel_arbiter')

        self._pub = self.create_publisher(Twist, 'cmd_vel_out', 10)

        # 소스별 (마지막 수신 시각, 마지막 메시지) — cmd_vel_stop은 내용을 안 쓰므로
        # 메시지 자체는 저장하지 않고 수신 시각만 추적한다.
        self._last_nav = (None, None)
        self._last_manual = (None, None)
        self._last_stop_monotonic = None

        # /robot_status가 아직 한 번도 안 왔을 때의 기본값 — 부팅 직후에도
        # 안전하도록 주행이 허용되지 않는 STOP으로 취급.
        self._current_mode = "STOP"

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Status, '/robot_status', self._on_status, status_qos)
        self.create_subscription(Twist, 'cmd_vel_nav_out', self._on_nav, 10)
        self.create_subscription(Twist, 'cmd_vel_manual', self._on_manual, 10)
        self.create_subscription(Twist, 'cmd_vel_stop', self._on_stop, 10)

        self.create_timer(1.0 / ARBITRATION_RATE_HZ, self._arbitrate)

    def _on_status(self, msg: Status) -> None:
        self._current_mode = msg.mode

    def _on_nav(self, msg: Twist) -> None:
        self._last_nav = (time.monotonic(), msg)

    def _on_manual(self, msg: Twist) -> None:
        self._last_manual = (time.monotonic(), msg)

    def _on_stop(self, _msg: Twist) -> None:
        """내용은 보지 않고 수신 시각만 기록 — 이 토픽에 뭐가 오든 정지 트리거."""
        self._last_stop_monotonic = time.monotonic()

    def _is_recent(self, last_monotonic) -> bool:
        return (
            last_monotonic is not None
            and (time.monotonic() - last_monotonic) <= SOURCE_TIMEOUT_SEC
        )

    def _arbitrate(self) -> None:
        """타이머 콜백 — 매 주기 규칙대로 출력을 하나 결정해 발행.

        1. cmd_vel_stop이 최근에 왔으면 모드 무관 무조건 정지.
        2. 현재 모드에 맞는 소스가 있고 그게 최근에 왔으면 그대로 통과.
        3. 그 외(허용 안 되는 모드거나, 맞는 소스가 죽었거나)는 정지.
        """
        if self._is_recent(self._last_stop_monotonic):
            self._pub.publish(Twist())
            return

        source_topic = MODE_TO_SOURCE_TOPIC.get(self._current_mode)
        if source_topic == 'cmd_vel_nav_out':
            last_monotonic, msg = self._last_nav
        elif source_topic == 'cmd_vel_manual':
            last_monotonic, msg = self._last_manual
        else:
            last_monotonic, msg = (None, None)

        if source_topic is not None and self._is_recent(last_monotonic):
            self._pub.publish(msg)
        else:
            self._pub.publish(Twist())


def main():
    """노드 진입점 — `rclpy.spin()`으로 상주하며 매 주기 중재 결과를 발행한다."""
    rclpy.init()
    node = CmdVelArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
