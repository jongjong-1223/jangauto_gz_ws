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
- 웨이포인트를 Nav2 `NavigateThroughPoses`(직선 구간 이동)와 `Spin`(제자리
  회전)으로 나눠서 순서대로 실행한다: 두둑을 넘어갈 때마다 거의 180도에
  가까운 제자리 회전이 필요한데, 이건 연속 경로 추종(Regulated Pure
  Pursuit)이 다루는 곡선형 기동이 아니라 "멈춰서 돈다"는 별개의 동작이라
  Waypoint.turn_angle이 유의미한 지점마다 배치를 끊고 Spin을 끼워 넣는다.
  Nav2 출력(`cmd_vel_smoothed`->`cmd_vel_nav_out`)은 `cmd_vel_arbiter.py`가
  RUN 모드에서 그대로 로봇으로 흘려보내므로 별도 배관 변경은 필요 없다.
- goal 실행 중 블로킹 대기는 `calibration_action_server.py`와 동일하게
  `MultiThreadedExecutor` 위에서 이뤄진다 — `_send_and_wait()`이 워커
  스레드 하나를 블로킹하는 동안에도 `selected_coverage_path` 구독이나
  Nav2 액션의 goal-response/result 콜백은 다른 스레드에서 계속 처리된다.

## 동작 순서 (goal 하나당)
1. 캐시된 `CoveragePath`가 없으면 즉시 실패 리턴
2. 웨이포인트를 순회하며 `turn_angle`이 임계값을 넘는 지점마다 배치를 끊음
3. 각 배치를 `NavigateThroughPoses`로 이동 -> 도착 후 필요하면 `Spin`으로 제자리 회전
4. cancel 요청 시 진행 중인 Nav2 sub-goal을 취소하고 Run goal도 canceled 처리
5. 모든 웨이포인트 완료 시 성공 리턴
"""

import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateThroughPoses, Spin

from jangauto_msg.action import Run
from jangauto_msg.msg import CoveragePath

SELECTED_PATH_TOPIC = '/jangauto_mission/selected_coverage_path'

NAVIGATE_ACTION_NAME = 'navigate_through_poses'
SPIN_ACTION_NAME = 'spin'
NAV2_WAIT_FOR_SERVER_TIMEOUT_SEC = 5.0

TURN_ANGLE_THRESHOLD_RAD = 0.05   # 이 이하 회전은 곡선 추종으로 흡수, Spin 안 함
SPIN_TIME_ALLOWANCE_SEC = 30.0
CANCEL_POLL_PERIOD_SEC = 0.1      # 서브골 완료/취소 확인 폴링 주기


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
        self.declare_parameter('spin_time_allowance_sec', SPIN_TIME_ALLOWANCE_SEC)
        self._turn_angle_threshold = float(
            self.get_parameter('turn_angle_threshold_rad').value)
        self._spin_time_allowance = float(
            self.get_parameter('spin_time_allowance_sec').value)

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

        self._navigate_client = ActionClient(self, NavigateThroughPoses, NAVIGATE_ACTION_NAME)
        self._spin_client = ActionClient(self, Spin, SPIN_ACTION_NAME)

        self._server = ActionServer(
            self, Run, 'run', self._execute_callback,
            cancel_callback=self._cancel_callback)

    def _on_selected_path(self, msg: CoveragePath) -> None:
        self._selected_path = msg
        self.get_logger().info(
            f'[Run] Selected coverage path updated: {len(msg.waypoints)} waypoint(s), '
            f'start_side={msg.start_side}')

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        # rclpy ActionServer 기본값은 REJECT라 명시적으로 ACCEPT해야 취소가
        # 실제로 반영된다(calibration_action_server.py에서 확인된 동일 함정).
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        path = self._selected_path
        if path is None or not path.waypoints:
            goal_handle.abort()
            return Run.Result(success=False, message='선택된 경로 없음')

        waypoints = path.waypoints
        n = len(waypoints)
        batch = [waypoints[0]]

        for i in range(1, n):
            wp = waypoints[i]
            if abs(wp.turn_angle) > self._turn_angle_threshold:
                # 이 지점에서 회전이 필요 -> 여기까지 먼저 이동한 뒤 제자리 회전.
                batch.append(wp)
                cancelled = self._run_navigate_batch(goal_handle, batch, i, n)
                if cancelled:
                    return self._on_cancelled(goal_handle)
                cancelled = self._run_spin(goal_handle, wp.turn_angle, i, n)
                if cancelled:
                    return self._on_cancelled(goal_handle)
                batch = [wp]  # 회전 후 이 지점에서 새 배치 시작
            else:
                batch.append(wp)

        if len(batch) > 1:
            cancelled = self._run_navigate_batch(goal_handle, batch, n, n)
            if cancelled:
                return self._on_cancelled(goal_handle)

        goal_handle.succeed()
        return Run.Result(success=True, message=f'{n}개 웨이포인트 주행 완료')

    def _on_cancelled(self, goal_handle):
        goal_handle.canceled()
        return Run.Result(success=False, message='주행 중 취소됨')

    def _publish_progress(self, goal_handle, done_idx: int, total: int) -> None:
        goal_handle.publish_feedback(Run.Feedback(status=f'{done_idx}/{total} 웨이포인트 진행 중'))

    def _run_navigate_batch(self, goal_handle, wp_batch: list, done_idx: int, total: int) -> bool:
        """`wp_batch`를 NavigateThroughPoses 하나로 이동. cancel되면 True."""
        goal = NavigateThroughPoses.Goal()
        goal.poses = [_pose_from_waypoint(wp) for wp in wp_batch]
        cancelled = self._send_and_wait(
            goal_handle, self._navigate_client, NAVIGATE_ACTION_NAME, goal)
        if not cancelled:
            self._publish_progress(goal_handle, done_idx, total)
        return cancelled

    def _run_spin(self, goal_handle, angle: float, done_idx: int, total: int) -> bool:
        """헤드랜드에서 `angle`만큼 제자리 회전. cancel되면 True."""
        goal = Spin.Goal()
        goal.target_yaw = angle
        goal.time_allowance = Duration(sec=int(self._spin_time_allowance))
        cancelled = self._send_and_wait(
            goal_handle, self._spin_client, SPIN_ACTION_NAME, goal)
        if not cancelled:
            self._publish_progress(goal_handle, done_idx, total)
        return cancelled

    def _send_and_wait(self, run_goal_handle, client: ActionClient,
                        action_name: str, goal) -> bool:
        """Nav2 sub-goal을 보내고 완료를 기다린다. `MultiThreadedExecutor` 위에서
        돌기 때문에, 여기서 폴링 루프로 이 워커 스레드를 블로킹해도
        goal-response/result 콜백(다른 스레드)이나 `selected_coverage_path`
        구독은 계속 처리된다(calibration_action_server.py의 blocking 패턴과 동일).

        반환: Run goal이 취소 요청됐으면 True, sub-goal이 정상 완료됐으면 False.
        """
        if not client.wait_for_server(timeout_sec=NAV2_WAIT_FOR_SERVER_TIMEOUT_SEC):
            self.get_logger().error(f"[Run] Action server '{action_name}' not available")
            return True  # 계속 진행할 수 없으므로 취소와 동일하게 중단

        done = {'flag': False}
        sub_goal_handle_holder = [None]

        def _on_goal_response(future):
            sub_handle = future.result()
            if not sub_handle.accepted:
                self.get_logger().error(f"[Run] '{action_name}' goal rejected")
                done['flag'] = True
                return
            sub_goal_handle_holder[0] = sub_handle
            sub_handle.get_result_async().add_done_callback(lambda _f: done.update(flag=True))

        client.send_goal_async(goal).add_done_callback(_on_goal_response)

        while not done['flag']:
            if run_goal_handle.is_cancel_requested:
                if sub_goal_handle_holder[0] is not None:
                    sub_goal_handle_holder[0].cancel_goal_async()
                return True
            time.sleep(CANCEL_POLL_PERIOD_SEC)
        return False


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
