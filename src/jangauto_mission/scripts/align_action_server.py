#!/usr/bin/env python3
"""정렬(align) 액션 서버.

## 역할
- `mission_state_machine.py`의 ALIGN 상태가 호출하는 `align` 액션 서버.
- 앱이 고른 `CoveragePath`(`/jangauto_mission/selected_coverage_path`, latched,
  `run_action_server.py`와 동일 토픽/캐싱 패턴)의 `waypoints[0]`(kind="start")
  까지 Nav2 `NavigateToPose` 한 번으로 이동한다.
- RUN(`run_action_server.py`)은 이 지점을 더 이상 스스로 주행하지 않고
  `waypoints[1:]`부터 이어받는다 — 즉 "경로 시작점까지 정렬 이동" 책임을
  ALIGN이, "실제 작업 주행"을 RUN이 나눠 맡는다(mission_state_machine.py의
  전이 규칙상 RUN은 ALIGN을 거쳐야만 진입 가능하므로, RUN 시작 시점엔 로봇이
  항상 waypoints[0]에 도착해 있다는 전제가 구조적으로 보장된다).
- self-loop(ALIGN 액션 성공 직후 YASMIN이 자동으로 ALIGN에 재전이하는 경우)는
  `mission_state_machine.py`가 goal의 `self_loop=true`로 알려준다 — CAL/RUN과
  동일한 패턴. 이때는 다시 이동하지 않고 취소될 때까지 대기만 한다.
- Nav2 goal 전송/취소 대기 로직(`_pose_from_waypoint`, `_send_and_wait`)은
  `run_action_server.py`의 동일 함수를 의도적으로 복제한 것이다 — 두 서버가
  각자 독립 스크립트로 유지되는 게 이 코드베이스의 기존 관례(CAL/RUN/coverage
  경로 서버 모두 서로 import하지 않음)이고, 여기서 공용 모듈로 뽑으면 RUN의
  주행 크리티컬 패스(취소 처리)에 불필요한 회귀 리스크가 생기기 때문이다.
  **한쪽을 고치면 다른 쪽도 확인할 것.**

## 동작 순서 (goal 하나당)
1. `self_loop=true`면 재이동 없이 취소될 때까지 대기만 하고 리턴
2. 캐시된 `CoveragePath`가 없으면 즉시 실패 리턴
3. `waypoints[0]`으로 `NavigateToPose` 이동
4. cancel 요청 시 진행 중인 Nav2 sub-goal을 취소하고 Align goal도 canceled 처리
5. Nav2 sub-goal이 실패(취소가 아닌 abort/reject 등)로 끝나면 Align을 실패로 중단
"""

import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose

from jangauto_msg.action import Align
from jangauto_msg.msg import CoveragePath

SELECTED_PATH_TOPIC = '/jangauto_mission/selected_coverage_path'

NAVIGATE_ACTION_NAME = 'navigate_to_pose'
NAV2_WAIT_FOR_SERVER_TIMEOUT_SEC = 5.0

CANCEL_POLL_PERIOD_SEC = 0.1      # 서브골 완료/취소 확인 폴링 주기
IDLE_POLL_PERIOD_SEC = 0.1        # self-loop 대기 중 취소 확인 주기


def _pose_from_waypoint(wp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = wp.x
    pose.pose.position.y = wp.y
    pose.pose.orientation.z = math.sin(wp.yaw / 2.0)
    pose.pose.orientation.w = math.cos(wp.yaw / 2.0)
    return pose


class AlignActionServer(Node):
    """`align` 액션 서버 — 선택된 커버리지 경로의 시작점으로 Nav2 이동."""

    def __init__(self):
        super().__init__('align_action_server')

        # run_action_server.py와 동일한 latched QoS — 늦게 뜬 경우도 마지막
        # 선택을 바로 받는다.
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._selected_path: CoveragePath = None
        self.create_subscription(
            CoveragePath, SELECTED_PATH_TOPIC, self._on_selected_path, latched_qos)

        self._navigate_client = ActionClient(self, NavigateToPose, NAVIGATE_ACTION_NAME)

        self._server = ActionServer(
            self, Align, 'align', self._execute_callback,
            cancel_callback=self._cancel_callback)

    def _on_selected_path(self, msg: CoveragePath) -> None:
        self._selected_path = msg
        self.get_logger().info(
            f'[Align] Selected coverage path updated: {len(msg.waypoints)} waypoint(s), '
            f'first_row_side={msg.first_row_side}')

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        # rclpy ActionServer 기본값은 REJECT라 명시적으로 ACCEPT해야 취소가
        # 실제로 반영된다(calibration_action_server.py/run_action_server.py에서
        # 확인된 동일 함정).
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        if goal_handle.request.self_loop:
            return self._wait_idle(goal_handle)

        path = self._selected_path
        if path is None or not path.waypoints:
            goal_handle.abort()
            return Align.Result(success=False, message='선택된 경로 없음')

        wp = path.waypoints[0]
        goal = NavigateToPose.Goal()
        goal.pose = _pose_from_waypoint(wp)

        outcome = self._send_and_wait(goal_handle, self._navigate_client, NAVIGATE_ACTION_NAME, goal)
        if outcome == 'ok':
            goal_handle.publish_feedback(Align.Feedback(status='시작 지점 도착'))
            goal_handle.succeed()
            return Align.Result(success=True, message='시작 지점 정렬 완료')
        if outcome == 'cancelled':
            goal_handle.canceled()
            return Align.Result(success=False, message='정렬 이동 중 취소됨')
        goal_handle.abort()
        return Align.Result(success=False, message='정렬 이동 중 Nav2 하위 목표 실패로 중단됨')

    def _wait_idle(self, goal_handle):
        """self-loop 대기 분기 — 이미 정렬을 마쳤으므로 재이동 없이 취소될
        때까지 블로킹한다(run_action_server.py의 동일 패턴 참고)."""
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Align.Result(success=False, message='self-loop 대기 중 취소됨')
            time.sleep(IDLE_POLL_PERIOD_SEC)
        return Align.Result(success=False, message='노드 종료')

    def _send_and_wait(self, align_goal_handle, client: ActionClient,
                        action_name: str, goal) -> str:
        """Nav2 sub-goal을 보내고 완료를 기다린다. `MultiThreadedExecutor` 위에서
        돌기 때문에, 여기서 폴링 루프로 이 워커 스레드를 블로킹해도
        goal-response/result 콜백(다른 스레드)이나 `selected_coverage_path`
        구독은 계속 처리된다.

        반환: 'ok'(정상 완료) / 'cancelled'(Align이 취소 요청됨) /
        'failed'(서버 없음/goal 거부/sub-goal이 성공이 아닌 상태로 종료).
        """
        if not client.wait_for_server(timeout_sec=NAV2_WAIT_FOR_SERVER_TIMEOUT_SEC):
            self.get_logger().error(f"[Align] Action server '{action_name}' not available")
            return 'failed'

        done = {'flag': False, 'outcome': 'failed'}
        sub_goal_handle_holder = [None]

        def _on_result(future):
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                done['outcome'] = 'ok'
            else:
                self.get_logger().error(
                    f"[Align] '{action_name}' sub-goal ended with status {status} (not succeeded)")
                done['outcome'] = 'failed'
            done['flag'] = True

        def _on_goal_response(future):
            sub_handle = future.result()
            if not sub_handle.accepted:
                self.get_logger().error(f"[Align] '{action_name}' goal rejected")
                done['flag'] = True
                return
            sub_goal_handle_holder[0] = sub_handle
            sub_handle.get_result_async().add_done_callback(_on_result)

        client.send_goal_async(goal).add_done_callback(_on_goal_response)

        while not done['flag']:
            if align_goal_handle.is_cancel_requested:
                if sub_goal_handle_holder[0] is not None:
                    sub_goal_handle_holder[0].cancel_goal_async()
                return 'cancelled'
            time.sleep(CANCEL_POLL_PERIOD_SEC)
        return done['outcome']


def main():
    """노드 진입점 — `MultiThreadedExecutor`로 상주한다.

    `_send_and_wait()`의 블로킹 폴링이 실행 콜백 스레드 하나를 잡고 있는
    동안에도 `selected_coverage_path` 구독과 Nav2 액션의 goal-response/result
    콜백이 다른 스레드에서 계속 처리돼야 하므로(기본 `SingleThreadedExecutor`면
    콜백이 영영 안 돌아 데드락) `MultiThreadedExecutor`를 명시적으로 사용한다.
    """
    rclpy.init()
    node = AlignActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
