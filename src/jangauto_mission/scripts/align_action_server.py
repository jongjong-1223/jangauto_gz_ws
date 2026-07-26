#!/usr/bin/env python3
"""정렬(align) 액션 서버 (스켈레톤).

## 역할
- `mission_state_machine.py`의 ALIGN 상태가 호출하는 `align` 액션 서버.
- 헤딩만 맞추는지 특정 위치까지 이동하는지, 목표 자세를 어디서 얻는지가
  아직 정해지지 않아, goal을 받으면 일정 시간 뒤 성공을 리턴하는 TODO
  placeholder만 담고 있다 — `calibration_action_server.py`와 동일한 목적
  (오케스트레이션 배관을 먼저 검증)이자 동일한 지연 이유(아래 참고).
- `PLACEHOLDER_DELAY_SEC`만큼 일부러 지연 후 응답한다: 즉시 성공하면
  mission_state_machine.py가 상태 진입 때마다 새 goal을 보내고 바로 성공
  응답을 받아 self-loop 전이를 쉴 새 없이 반복하는 게 실행 확인됨(로그
  스팸·CPU 낭비). 실제 알고리즘이 들어가 처리 시간이 생기면 이 지연은
  제거한다.
- 실제 정렬 제어 로직이 정해지면 `_execute_callback()` 내부만 채우면 되고,
  `mission_state_machine.py`의 호출부는 바뀔 필요가 없다.
"""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from jangauto_msg.action import Align

# 즉시 성공 응답으로 인한 self-loop 스팸을 막기 위한 임시 지연(초).
# TODO: 실제 정렬 제어 로직이 들어가면 제거.
PLACEHOLDER_DELAY_SEC = 2.0


class AlignActionServer(Node):
    """`align` 액션 서버 — 액션 이름/타입만 확정, 내부 로직은 TODO."""

    def __init__(self):
        super().__init__('align_action_server')
        self._server = ActionServer(
            self, Align, 'align', self._execute_callback)

    def _execute_callback(self, goal_handle):
        """goal 수신 콜백. TODO: 실제 정렬 제어 로직으로 교체."""
        time.sleep(PLACEHOLDER_DELAY_SEC)
        goal_handle.succeed()
        result = Align.Result()
        result.success = True
        result.message = "TODO: alignment logic not implemented yet"
        return result


def main():
    """노드 진입점 — `rclpy.spin()`으로 상주하며 액션 goal을 처리한다."""
    rclpy.init()
    node = AlignActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
