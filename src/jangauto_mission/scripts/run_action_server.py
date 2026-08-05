#!/usr/bin/env python3
"""주행(run) 액션 서버.

## 역할
- `mission_state_machine.py`의 RUN 상태가 호출하는 `run` 액션 서버
  (goal 필드 없음 — CAL이 결과를 goal이 아닌 latched 토픽으로 흘리는 것과
  같은 이유로, 선택된 경로도 goal 필드가 아니라
  `/jangauto_mission/selected_coverage_path`(latched)로 전달된다).
- 앱이 고른 `CoveragePath`를 이 노드가 계속 구독해 최신값을 캐싱하고 있다가,
  RUN goal이 들어오면 그 시점의 캐시를 그대로 주행한다. 캐시는 성공적인
  주행 뒤에도 지우지 않는다 — 같은 경로를 다시 RUN해도 재선택 없이 그대로
  재사용 가능(사용자 확정 사항).
- `waypoints[0]`(시작점)은 RUN이 아니라 ALIGN(`align_action_server.py`)이
  먼저 이동을 담당한다 — RUN은 `waypoints[1:]`부터 주행한다. 앱으로 나가는
  `waypoints` 배열 자체는 그대로(스키마 변경 없음), 로봇 측 실행만 ALIGN과
  RUN으로 나뉜다. mission_state_machine.py의 전이 규칙상 RUN은 ALIGN을 거쳐야만
  진입 가능하므로, RUN 시작 시점엔 로봇이 항상 waypoints[0]에 도착해 있다.
- 웨이포인트를 **하나씩** Nav2 `NavigateToPose`로 순서대로 이동시킨다 —
  지점 하나에 goal 하나, 성공해야 다음 지점의 goal을 보낸다(배치로 묶어서
  한 번에 보내지 않음). 이렇게 지점 단위로 끊어둔 이유는 나중에
  `Waypoint.kind`(work_start/work_end 등)별로 정지 후 커스텀 동작(예: 작업
  장치 제어)을 끼워 넣을 확장 지점이 필요하기 때문 — 실제 지점별 동작은
  아직 미구현(TODO).
- 두둑 진입/이탈(`turn_out`/`turn_in`, `turn_angle`이 유의미한 지점)은 별도
  `Spin` 액션 없이, `controller_server`의
  `general_goal_checker.yaw_goal_tolerance`를 그 지점 goal에 한해서만
  일시적으로 타이트하게(`TIGHT_YAW_GOAL_TOLERANCE_RAD`) 낮춰서 처리한다 —
  `FollowPath`(RPP) 자신이 goal의 목표 방향(`wp.yaw`, 다음 구간 방향)까지
  정밀하게 맞춰야 그 goal이 끝나도록 만드는 것. 원래는 `NavigateToPose` 도착
  후 별도로 `Spin(상대 회전 turn_angle)`을 또 호출했는데, goal 자체가 이미
  목표 방향까지 회전을 마친 상태라 그 위에 상대 회전이 또 더해져 이중으로
  과회전(조향이 과도하게 꺾였다 원복)하는 버그가 있어 Spin 호출을 제거하고
  이 방식으로 바꿨다. 회전이 필요 없는 지점(`work_start`/`work_end` 등)은
  `NORMAL_YAW_GOAL_TOLERANCE_RAD`(기본값)로 두는데, 그 지점들은 들어오는
  방향=나가는 방향이라 애초에 orientation이 문제되지 않는다.
  Nav2 출력(`cmd_vel_smoothed`->`cmd_vel_nav_out`)은 `cmd_vel_arbiter.py`가
  RUN 모드에서 그대로 로봇으로 흘려보내므로 별도 배관 변경은 필요 없다.
- goal 실행 중 블로킹 대기는 `calibration_action_server.py`와 동일하게
  `MultiThreadedExecutor` 위에서 이뤄진다 — `_send_and_wait()`이 워커
  스레드 하나를 블로킹하는 동안에도 `selected_coverage_path` 구독이나
  Nav2 액션의 goal-response/result 콜백은 다른 스레드에서 계속 처리된다.
- self-loop(RUN 액션 성공 직후 YASMIN이 자동으로 RUN에 재전이하는 경우, 즉
  완주 후에도 앱이 sw_bits를 RUN에서 안 내린 경우)는
  `mission_state_machine.py`가 goal의 `self_loop=true`로 알려준다 — CAL과
  동일한 패턴. 이때는 경로를 처음부터 다시 주행하지 않고 취소될 때까지
  대기만 한다(안 그러면 완주할 때마다 처음부터 재시작해버린다 — 확인된
  버그).

## 동작 순서 (goal 하나당)
1. `self_loop=true`면 재주행 없이 취소될 때까지 대기만 하고 리턴
2. 캐시된 `CoveragePath`가 없으면 즉시 실패 리턴
3. `waypoints[1:]`(시작점 제외, ALIGN이 이미 주행함)을 순서대로 하나씩
   `NavigateToPose`로 이동(회전이 필요한 지점은 그 goal에 한해
   `yaw_goal_tolerance`를 타이트하게) -> 다음 웨이포인트로. 남은 지점이
   없으면(총 1개뿐이던 경로) 즉시 성공 처리
4. cancel 요청 시 진행 중인 Nav2 sub-goal을 취소하고 Run goal도 canceled 처리
5. Nav2 sub-goal이 실패(취소가 아닌 abort/reject 등)로 끝나면 즉시 Run 전체를
   실패로 중단 — 실패한 구간을 못 본 척 다음 단계로 넘어가지 않는다.
6. 모든 웨이포인트 완료 시 성공 리턴, `yaw_goal_tolerance`는 항상(성공/실패/
   취소 불문) 기본값으로 복원
"""

import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from nav2_msgs.action import NavigateToPose

from jangauto_msg.action import Run
from jangauto_msg.msg import CoveragePath

SELECTED_PATH_TOPIC = '/jangauto_mission/selected_coverage_path'

NAVIGATE_ACTION_NAME = 'navigate_to_pose'
NAV2_WAIT_FOR_SERVER_TIMEOUT_SEC = 5.0
GOAL_CHECKER_PARAM_SERVICE = '/controller_server/set_parameters'
YAW_GOAL_TOLERANCE_PARAM = 'general_goal_checker.yaw_goal_tolerance'

TURN_ANGLE_THRESHOLD_RAD = 0.05          # 이 이하 회전은 곡선 추종으로 흡수
NORMAL_YAW_GOAL_TOLERANCE_RAD = 0.25     # nav2_params_simul.yaml 기본값과 동일
TIGHT_YAW_GOAL_TOLERANCE_RAD = 0.05      # 두둑 회전 지점: 정밀하게 맞춰야 함
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


class RunActionServer(Node):
    """`run` 액션 서버 — 선택된 커버리지 경로를 Nav2로 실제 주행."""

    def __init__(self):
        super().__init__('run_action_server')

        self.declare_parameter('turn_angle_threshold_rad', TURN_ANGLE_THRESHOLD_RAD)
        self.declare_parameter('normal_yaw_goal_tolerance_rad', NORMAL_YAW_GOAL_TOLERANCE_RAD)
        self.declare_parameter('tight_yaw_goal_tolerance_rad', TIGHT_YAW_GOAL_TOLERANCE_RAD)
        self._turn_angle_threshold = float(
            self.get_parameter('turn_angle_threshold_rad').value)
        self._normal_yaw_goal_tolerance = float(
            self.get_parameter('normal_yaw_goal_tolerance_rad').value)
        self._tight_yaw_goal_tolerance = float(
            self.get_parameter('tight_yaw_goal_tolerance_rad').value)

        # app_websocket_bridge.py가 select_coverage_path 처리 시 발행하는 latched
        # 토픽 — 이 노드 생명주기 동안 계속 구독해서, 재선택이 오면 캐시가 즉시
        # 갱신된다(늦게 뜬 경우도 latched라 마지막 선택을 바로 받음).
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
        # 두둑 회전 지점에서 general_goal_checker.yaw_goal_tolerance를
        # 일시적으로 타이트하게 바꾸는 데 씀(SimpleGoalChecker는 런타임 파라미터
        # 변경을 지원 — controller_server 재시작 불필요).
        self._goal_checker_param_client = self.create_client(
            SetParameters, GOAL_CHECKER_PARAM_SERVICE)

        self._server = ActionServer(
            self, Run, 'run', self._execute_callback,
            cancel_callback=self._cancel_callback)

    def _on_selected_path(self, msg: CoveragePath) -> None:
        self._selected_path = msg
        self.get_logger().info(
            f'[Run] Selected coverage path updated: {len(msg.waypoints)} waypoint(s), '
            f'first_row_side={msg.first_row_side}')

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        # rclpy ActionServer 기본값은 REJECT라 명시적으로 ACCEPT해야 취소가
        # 실제로 반영된다(calibration_action_server.py에서 확인된 동일 함정).
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        if goal_handle.request.self_loop:
            return self._wait_idle(goal_handle)

        path = self._selected_path
        if path is None or not path.waypoints:
            goal_handle.abort()
            return Run.Result(success=False, message='선택된 경로 없음')

        waypoints = path.waypoints[1:]  # [0]은 ALIGN이 이미 주행함
        n = len(waypoints)
        if n == 0:
            goal_handle.succeed()
            return Run.Result(success=True, message='ALIGN이 유일한 웨이포인트를 이미 주행함')

        try:
            for i, wp in enumerate(waypoints):
                is_turn_point = abs(wp.turn_angle) > self._turn_angle_threshold
                self._set_yaw_goal_tolerance(
                    self._tight_yaw_goal_tolerance if is_turn_point
                    else self._normal_yaw_goal_tolerance)

                outcome = self._run_navigate_to_point(goal_handle, wp, i + 1, n)
                if outcome != 'ok':
                    return self._on_subgoal_ended(goal_handle, outcome)

                # TODO(추후): wp.kind(work_start/work_end 등)에 따른 정지 후
                # 커스텀 동작(예: 작업 장치 제어)은 여기에 끼워 넣는다.

            goal_handle.succeed()
            return Run.Result(success=True, message=f'{n}개 웨이포인트 주행 완료(시작점은 ALIGN 담당)')
        finally:
            # 성공/실패/취소 어느 경로든 controller_server의 공유 파라미터를
            # 반드시 기본값으로 되돌린다 — 안 그러면 이후 다른 Nav2 goal에도
            # 타이트한 tolerance가 그대로 남아 영향을 준다.
            self._set_yaw_goal_tolerance(self._normal_yaw_goal_tolerance)

    def _wait_idle(self, goal_handle):
        """self-loop 대기 분기 — 이미 완주했으므로 재주행 없이 취소될 때까지
        블로킹한다(calibration_action_server.py의 동일 패턴 참고)."""
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Run.Result(success=False, message='self-loop 대기 중 취소됨')
            time.sleep(IDLE_POLL_PERIOD_SEC)
        return Run.Result(success=False, message='노드 종료')

    def _on_subgoal_ended(self, goal_handle, outcome: str):
        """Nav2 sub-goal이 'ok'가 아니게 끝났을 때 Run 전체를 어떻게 마무리할지
        결정 — 취소는 canceled로, 그 외(실패/거부/서버 없음)는 abort로 처리해서
        실패한 구간을 못 본 척 다음 단계로 넘어가지 않는다."""
        if outcome == 'cancelled':
            goal_handle.canceled()
            return Run.Result(success=False, message='주행 중 취소됨')
        goal_handle.abort()
        return Run.Result(success=False, message='주행 중 Nav2 하위 목표 실패로 중단됨')

    def _publish_progress(self, goal_handle, done_idx: int, total: int) -> None:
        goal_handle.publish_feedback(Run.Feedback(status=f'{done_idx}/{total} 웨이포인트 진행 중'))

    def _run_navigate_to_point(self, goal_handle, wp, done_idx: int, total: int) -> str:
        """웨이포인트 `wp` 하나로 NavigateToPose 이동. 반환값은 `_send_and_wait` 참고."""
        goal = NavigateToPose.Goal()
        goal.pose = _pose_from_waypoint(wp)
        outcome = self._send_and_wait(
            goal_handle, self._navigate_client, NAVIGATE_ACTION_NAME, goal)
        if outcome == 'ok':
            self._publish_progress(goal_handle, done_idx, total)
        return outcome

    def _set_yaw_goal_tolerance(self, value: float) -> bool:
        """`controller_server`의 `general_goal_checker.yaw_goal_tolerance`를
        런타임에 바꾼다(`SimpleGoalChecker`가 동적 파라미터 변경을 지원해
        재시작 불필요). 실패해도 예외를 던지지 않고 로그만 남긴다 — tolerance
        조정은 두둑 회전 정밀도를 위한 보조 수단이라, 실패했다고 주행
        자체를 막을 필요는 없다(다만 의도한 tolerance가 아닐 수 있음)."""
        if not self._goal_checker_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"[Run] '{GOAL_CHECKER_PARAM_SERVICE}' 서비스를 찾을 수 없음")
            return False

        request = SetParameters.Request()
        request.parameters = [Parameter(
            name=YAW_GOAL_TOLERANCE_PARAM,
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value),
        )]
        future = self._goal_checker_param_client.call_async(request)
        while not future.done():
            time.sleep(CANCEL_POLL_PERIOD_SEC)

        result = future.result()
        ok = bool(result and result.results and result.results[0].successful)
        if not ok:
            self.get_logger().error(f'[Run] yaw_goal_tolerance={value} 설정 실패')
        return ok

    def _send_and_wait(self, run_goal_handle, client: ActionClient,
                        action_name: str, goal) -> str:
        """Nav2 sub-goal을 보내고 완료를 기다린다. `MultiThreadedExecutor` 위에서
        돌기 때문에, 여기서 폴링 루프로 이 워커 스레드를 블로킹해도
        goal-response/result 콜백(다른 스레드)이나 `selected_coverage_path`
        구독은 계속 처리된다(calibration_action_server.py의 blocking 패턴과 동일).

        반환: 'ok'(정상 완료) / 'cancelled'(Run이 취소 요청됨) /
        'failed'(서버 없음/goal 거부/sub-goal이 성공이 아닌 상태로 종료).
        Nav2 액션 자체의 성공 여부(`GoalStatus`)까지 확인해야, 타임아웃 등으로
        sub-goal이 실패했는데도 성공한 것처럼 다음 단계로 넘어가는 걸 막는다.
        """
        if not client.wait_for_server(timeout_sec=NAV2_WAIT_FOR_SERVER_TIMEOUT_SEC):
            self.get_logger().error(f"[Run] Action server '{action_name}' not available")
            return 'failed'

        done = {'flag': False, 'outcome': 'failed'}
        sub_goal_handle_holder = [None]

        def _on_result(future):
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                done['outcome'] = 'ok'
            else:
                self.get_logger().error(
                    f"[Run] '{action_name}' sub-goal ended with status {status} (not succeeded)")
                done['outcome'] = 'failed'
            done['flag'] = True

        def _on_goal_response(future):
            sub_handle = future.result()
            if not sub_handle.accepted:
                self.get_logger().error(f"[Run] '{action_name}' goal rejected")
                done['flag'] = True
                return
            sub_goal_handle_holder[0] = sub_handle
            sub_handle.get_result_async().add_done_callback(_on_result)

        client.send_goal_async(goal).add_done_callback(_on_goal_response)

        while not done['flag']:
            if run_goal_handle.is_cancel_requested:
                if sub_goal_handle_holder[0] is not None:
                    sub_goal_handle_holder[0].cancel_goal_async()
                return 'cancelled'
            time.sleep(CANCEL_POLL_PERIOD_SEC)
        return done['outcome']


def main():
    """노드 진입점 — `MultiThreadedExecutor`로 상주(이유는 모듈 docstring 참고)."""
    rclpy.init()
    node = RunActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
