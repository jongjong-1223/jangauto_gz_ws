#!/usr/bin/env python3
"""ㄹ자 커버리지 경로 생성 액션 서버.

## 역할
- `app_websocket_bridge.py`가 앱의 `generate_coverage_path` 명령을 그대로
  옮겨 보내는 `generate_coverage_path` 액션 서버. 실제 계산은
  `coverage_path_lib.run_pipeline()`에 위임하고, 이 노드는 ROS 배관과
  입력 검증/상태 게이팅만 담당한다.
- sw_bits 전이(YASMIN)와 무관한 on-demand 액션이다 — 앱이 STOP/KEY/CAL
  상태일 때 언제든 호출할 수 있고, 몇 번을 다시 호출해도 매번 독립적으로
  새 결과를 계산한다(이전 결과를 참조하지 않음). ALIGN/RUN 중에는
  `goal_callback`에서 거부한다 — 실주행 중에 계산 자원을 뺏기지 않고,
  주행 중인 경로가 바뀌는 레이스도 원천 차단하기 위함.
- 결과(near/far 두 후보 — 어느 두둑부터 도는지만 다름)는 액션 result로만
  나간다 — 선택된 이후의 경로는 `run_action_server.py`가 별도로 구독하는
  latched 토픽(`/jangauto_mission/selected_coverage_path`)으로 전달되므로,
  이 서버는 "계산해서 두 후보를 돌려주는" 책임만 진다.

## 동작 순서 (goal 하나당)
1. `goal_callback`: 현재 `/robot_status.current_state`가 STOP/KEY/CAL이
   아니거나 이미 goal이 진행 중이면 거부
2. `execute_callback`: goal의 Polygon/edge_safety_dist 길이 검증
3. `coverage_path_lib.run_pipeline()` 호출(near/far 두 버전 + 헤드랜드 코너)
4. 각 버전을 `CoveragePath` 메시지로 변환해 result에 담아 반환
"""

import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point32

from jangauto_msg.action import GenerateCoveragePath
from jangauto_msg.msg import CoveragePath, Status, Waypoint

import coverage_path_lib as cpl

ROBOT_STATUS_TOPIC = '/robot_status'
ALLOWED_STATES = {'STOP', 'KEY', 'CAL'}

DEFAULT_CELL_SIZE = 0.2


def _waypoints_to_msg(waypoints_world: list) -> list:
    """`enrich_waypoints_world()`가 만든 dict 리스트 -> `Waypoint[]`.
    마지막 지점은 yaw_to_next/dist_to_next가 None이라, 직전 yaw를 그대로 쓴다."""
    out = []
    last_yaw = 0.0
    for wp in waypoints_world:
        yaw = wp['yaw_to_next_rad'] if wp['yaw_to_next_rad'] is not None else last_yaw
        last_yaw = yaw
        out.append(Waypoint(
            x=wp['x'], y=wp['y'], yaw=yaw,
            turn_angle=wp['turn_angle_rad'],
            dist_to_next=wp['dist_to_next'] if wp['dist_to_next'] is not None else 0.0,
            kind=wp['kind'],
            row_index=wp['row_index'],
        ))
    return out


def _corners_to_msg(corners: list) -> list:
    """헤드랜드 사각형 꼭짓점 [(x,y) x4] -> `Point32[4]`."""
    return [Point32(x=float(cx), y=float(cy), z=0.0) for cx, cy in corners]


class CoveragePathActionServer(Node):
    """`generate_coverage_path` 액션 서버 — 다각형+파라미터로 ㄹ자 경로 후보 2개 계산."""

    def __init__(self):
        super().__init__('coverage_path_action_server')

        self.declare_parameter('default_cell_size', DEFAULT_CELL_SIZE)
        self._default_cell_size = float(self.get_parameter('default_cell_size').value)

        # 게이팅 판단용 — mission_state_machine.py가 발행하는 latched 토픽을
        # app_websocket_bridge.py와 동일한 QoS로 구독.
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._current_state = None
        self.create_subscription(Status, ROBOT_STATUS_TOPIC, self._on_robot_status, latched_qos)

        # calibration_action_server.py와 동일한 이유(경쟁 상황 방어)로 동시 goal 1개만 허용.
        self._goal_active = False

        self._server = ActionServer(
            self, GenerateCoveragePath, 'generate_coverage_path', self._execute_callback,
            goal_callback=self._goal_callback)

    def _on_robot_status(self, msg: Status) -> None:
        self._current_state = msg.current_state

    def _goal_callback(self, goal_request) -> GoalResponse:
        if self._goal_active:
            self.get_logger().warning('[CoveragePath] Rejecting goal: another goal is already active')
            return GoalResponse.REJECT
        if self._current_state not in ALLOWED_STATES:
            self.get_logger().warning(
                f'[CoveragePath] Rejecting goal: current_state={self._current_state!r} '
                f'not in {ALLOWED_STATES}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self._goal_active = True
        try:
            return self._compute(goal_handle)
        finally:
            self._goal_active = False

    def _compute(self, goal_handle):
        req = goal_handle.request
        polygon = [(p.x, p.y) for p in req.polygon.points]
        edge_safety_dist = list(req.edge_safety_dist)

        if len(polygon) < 3:
            goal_handle.abort()
            return GenerateCoveragePath.Result(
                success=False, message=f'다각형 꼭짓점이 3개 미만: {len(polygon)}')
        if len(edge_safety_dist) != len(polygon):
            goal_handle.abort()
            return GenerateCoveragePath.Result(
                success=False,
                message=(f'edge_safety_dist 길이({len(edge_safety_dist)})가 '
                         f'polygon 꼭짓점 수({len(polygon)})와 다름'))

        cell_size = req.cell_size if req.cell_size > 0.0 else self._default_cell_size

        try:
            result = cpl.run_pipeline(
                polygon, edge_safety_dist, req.robot_radius, req.yaw_deg,
                req.ridge_spacing, req.headland_length, cell_size=cell_size)
        except (RuntimeError, ValueError, np.linalg.LinAlgError) as e:
            goal_handle.abort()
            return GenerateCoveragePath.Result(success=False, message=f'경로 생성 실패: {e}')

        # 헤드랜드 코너는 near/far 후보 공통값이지만, rosidl 배열 필드가 참조를
        # 공유하지 않도록 후보마다 새로 변환한다.
        near = CoveragePath(
            waypoints=_waypoints_to_msg(result['waypoints_first_row_near']),
            rect_length=result['rect_L'], rect_width=result['rect_W'],
            work_len=result['work_len'], n_ridges=result['n_ridges'], first_row_side='near',
            start_headland_corners=_corners_to_msg(result['start_headland_corners']),
            far_headland_corners=_corners_to_msg(result['far_headland_corners']))
        far = CoveragePath(
            waypoints=_waypoints_to_msg(result['waypoints_first_row_far']),
            rect_length=result['rect_L'], rect_width=result['rect_W'],
            work_len=result['work_len'], n_ridges=result['n_ridges'], first_row_side='far',
            start_headland_corners=_corners_to_msg(result['start_headland_corners']),
            far_headland_corners=_corners_to_msg(result['far_headland_corners']))

        self.get_logger().info(
            f"[CoveragePath] rect={result['rect_L']:.2f}x{result['rect_W']:.2f}m "
            f"work_len={result['work_len']:.2f}m n_ridges={result['n_ridges']}")

        goal_handle.succeed()
        return GenerateCoveragePath.Result(
            success=True, message='', candidate_paths=[near, far])


def main():
    """노드 진입점 — `MultiThreadedExecutor`로 상주(calibration_action_server.py와
    동일한 이유: `/robot_status` 구독 콜백이 goal 계산 중에도 계속 처리돼야 함)."""
    rclpy.init()
    node = CoveragePathActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
