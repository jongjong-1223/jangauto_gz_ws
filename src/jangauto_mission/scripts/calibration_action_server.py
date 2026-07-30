#!/usr/bin/env python3
"""캘리브레이션 액션 서버.

## 역할
- `mission_state_machine.py`의 CAL 상태가 호출하는 `calibrate` 액션 서버.
- "주행체가 파워온 때 잡은 IMU yaw=0"과 "GPS/지도 좌표계의 yaw=0" 사이의
  미지의 회전을 구하는 게 목적 — IMU 장착 오차가 아니라, 부팅마다 새로
  생기는 좌표계 정렬 문제라 매 세션 다시 수행한다.
- 전진(`forward_time_sec`초)→후진(`backward_time_sec`초)을 `num_repetitions`회
  반복하며 `/odometry/gps`(x, y)와 `/imu`(원본 yaw)를 계속 샘플링한다.
  이 구간엔 nav2(closed-loop 경로추종)를 쓰지 않고 `cmd_vel_calibration`에
  열린루프로 직접 Twist를 발행한다 — CAL 자체가 "IMU yaw가 아직 안 맞다"는
  전제이므로, 틀린 헤딩을 기준으로 조향을 보정하려 드는 closed-loop 제어는
  이 계산의 "명령한 그대로 똑바로 움직였다"는 전제를 깨버린다(그래서
  `collision_monitor`를 거치지 않음 — 사람이 안전한 공터로 옮겨놓고 실행하는
  운용을 전제로 함).
- 계산: GPS 궤적 전체에 PCA(SVD)를 돌려 주성분(진행방향) `gps_angle`을
  구하고, IMU yaw들을 단위벡터로 평균낸 방향 `imu_angle`을 구해
  `yaw_offset = gps_angle - imu_angle`을 산출한다. 이 값을
  `/imu_yaw_offset`에 주기 발행 — `imu_yaw_corrector.py`가 받아 `/imu`를
  보정해 `/imu_calibrated`로 재발행한다(EKF들의 실제 입력).
- self-loop(액션 성공 직후 YASMIN이 자동으로 CAL에 재전이) 시엔
  `mission_state_machine.py`가 goal의 `self_loop=true`로 알려주고, 이때는
  다시 주행하지 않고 취소될 때까지 대기만 한다 — 안 그러면 self-loop가
  self-loop를 부르며 무한 반복 주행하게 된다. 다른 상태에 있다가 CAL로
  (다시) 들어오면(`self_loop=false`) 매번 실제로 재측정한다.
- 캘리브레이션이 한 번이라도 성공하면 `/jangauto_mission/calibration_complete`
  (`std_msgs/Bool`)를 주기 발행한다 — `app_websocket_bridge.py`가 이걸로
  MoveRequest 수락 조건("CAL을 거친 적 있는가")을 판단한다.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64

from jangauto_msg.action import Calibrate

CALIBRATION_COMPLETE_TOPIC = '/jangauto_mission/calibration_complete'
CALIBRATION_COMPLETE_PUBLISH_PERIOD_SEC = 1.0

CMD_VEL_OUT_TOPIC = 'cmd_vel_calibration'
GPS_ODOM_TOPIC = '/odometry/gps'
IMU_TOPIC = '/imu'
YAW_OFFSET_TOPIC = 'imu_yaw_offset'
YAW_OFFSET_PUBLISH_PERIOD_SEC = 1.0

CONTROL_PERIOD_SEC = 0.1          # 주행/취소확인 tick 주기(레거시와 동일)
IDLE_POLL_PERIOD_SEC = 0.1        # self-loop 대기 중 취소 확인 주기


class CalibrationActionServer(Node):
    """`calibrate` 액션 서버 — GPS-IMU 비교로 yaw_offset을 계산해 발행한다."""

    def __init__(self):
        super().__init__('calibration_action_server')

        # CAL 파라미터(레거시 state_machine.py 기본값과 동일).
        self.declare_parameter('forward_time_sec', 10.0)
        self.declare_parameter('backward_time_sec', 10.0)
        self.declare_parameter('forward_speed', 0.3)
        self.declare_parameter('backward_speed', -0.3)
        self.declare_parameter('num_repetitions', 3)
        self.declare_parameter('min_gps_points', 20)
        self.declare_parameter('min_imu_yaws', 50)
        self.declare_parameter('min_forward_distance_m', 0.5)
        self._forward_time = float(self.get_parameter('forward_time_sec').value)
        self._backward_time = float(self.get_parameter('backward_time_sec').value)
        self._forward_speed = float(self.get_parameter('forward_speed').value)
        self._backward_speed = float(self.get_parameter('backward_speed').value)
        self._num_repetitions = int(self.get_parameter('num_repetitions').value)
        self._min_gps_points = int(self.get_parameter('min_gps_points').value)
        self._min_imu_yaws = int(self.get_parameter('min_imu_yaws').value)
        self._min_forward_distance_m = float(self.get_parameter('min_forward_distance_m').value)

        # 늦게 붙는 구독자(app_websocket_bridge 재시작 등)도 최신값을 즉시
        # 받도록 latched QoS — mission_state_machine.py의 /robot_status와 동일한 계약.
        complete_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._complete_pub = self.create_publisher(Bool, CALIBRATION_COMPLETE_TOPIC, complete_qos)
        self._complete = False
        self.create_timer(CALIBRATION_COMPLETE_PUBLISH_PERIOD_SEC, self._publish_complete)

        # 계산된 offset — 노드 생명주기 동안 유지, 계산될 때마다 갱신되고
        # __init__의 타이머가 goal 실행 상태와 무관하게 계속 주기 발행한다.
        self._current_offset = 0.0
        self._offset_valid = False
        self._offset_pub = self.create_publisher(Float64, YAW_OFFSET_TOPIC, 10)
        self.create_timer(YAW_OFFSET_PUBLISH_PERIOD_SEC, self._publish_offset)

        # 캘리브레이션 주행 중(FORWARD/BACKWARD)에만 True — GPS/IMU 콜백이
        # 이 플래그를 보고 샘플을 쌓을지 결정한다.
        self._collecting = False
        self._gps_points: list = []
        self._imu_yaws: list = []

        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_OUT_TOPIC, 10)
        self.create_subscription(Odometry, GPS_ODOM_TOPIC, self._on_gps, 10)
        self.create_subscription(Imu, IMU_TOPIC, self._on_imu, 50)

        # 동시에 goal 하나만 실행 — 이미 실행 중인데 새 goal이 들어오면 거부한다
        # (정상 흐름에선 mission_state_machine.py가 이전 goal을 취소한 뒤에만
        # 새 goal을 보내지만, 취소 처리와 겹치는 경쟁 상황에 대한 방어).
        self._goal_active = False

        self._server = ActionServer(
            self, Calibrate, 'calibrate', self._execute_callback,
            goal_callback=self._goal_callback,
            # rclpy ActionServer는 cancel_callback을 안 주면 기본값이
            # 무조건 REJECT라 goal_handle.is_cancel_requested가 절대
            # True가 되지 않는다(실제로 겪은 버그 — 취소해도 주행이 끝까지
            # 다 돌았음) — 반드시 ACCEPT하는 콜백을 명시해야 한다.
            cancel_callback=self._cancel_callback)

    def _goal_callback(self, goal_request) -> GoalResponse:
        if self._goal_active:
            self.get_logger().warning('[CAL] Rejecting goal: another goal is already active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_complete(self) -> None:
        msg = Bool()
        msg.data = self._complete
        self._complete_pub.publish(msg)

    def _publish_offset(self) -> None:
        """계산된 값이 하나라도 있으면 계속 재발행 — 골 실행 상태(성공/대기/
        진행 중)와 무관하게 노드가 살아있는 한 도는 타이머라, 늦게 뜬
        imu_yaw_corrector가 재시작돼도 곧 최신값을 다시 받는다."""
        if not self._offset_valid:
            return
        msg = Float64()
        msg.data = self._current_offset
        self._offset_pub.publish(msg)

    def _on_gps(self, msg: Odometry) -> None:
        if not self._collecting:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if math.isfinite(x) and math.isfinite(y):
            self._gps_points.append((x, y))

    def _on_imu(self, msg: Imu) -> None:
        if not self._collecting:
            return
        self._imu_yaws.append(self._yaw_from_quat(msg.orientation))

    @staticmethod
    def _yaw_from_quat(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _execute_callback(self, goal_handle):
        """goal 수신 콜백 — self_loop면 대기만, 아니면 실제 캘리브레이션 수행.

        `MultiThreadedExecutor`(main() 참고)로 돌기 때문에 이 함수 안에서
        `time.sleep()`으로 블로킹해도 `/imu`·`/odometry/gps` 구독 콜백과
        주기 발행 타이머는 다른 스레드에서 계속 처리된다.
        """
        self._goal_active = True
        try:
            if goal_handle.request.self_loop:
                return self._wait_idle(goal_handle)
            return self._run_calibration(goal_handle)
        finally:
            self._goal_active = False

    def _wait_idle(self, goal_handle):
        """self-loop 대기 분기 — 주행/계산 없이 취소될 때까지 블로킹."""
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Calibrate.Result(success=False, message='self-loop 대기 중 취소됨')
            time.sleep(IDLE_POLL_PERIOD_SEC)
        return Calibrate.Result(success=False, message='노드 종료')

    def _run_calibration(self, goal_handle):
        """실제 전진/후진 반복 주행 + GPS-IMU 비교 계산."""
        self._gps_points = []
        self._imu_yaws = []
        self._collecting = True

        canceled = self._drive_repetitions(goal_handle)

        self._collecting = False
        self._cmd_pub.publish(Twist())  # 정지

        if canceled:
            goal_handle.canceled()
            return Calibrate.Result(success=False, message='주행 중 취소됨')

        return self._compute_and_finish(goal_handle)

    def _drive_repetitions(self, goal_handle) -> bool:
        """FORWARD/BACKWARD를 num_repetitions회 반복. 취소되면 True 리턴."""
        for rep in range(self._num_repetitions):
            self.get_logger().info(
                f'[CAL] Rep {rep + 1}/{self._num_repetitions}: Forward')
            if self._drive_leg(goal_handle, self._forward_speed, self._forward_time):
                return True
            self.get_logger().info(
                f'[CAL] Rep {rep + 1}/{self._num_repetitions}: Backward')
            if self._drive_leg(goal_handle, self._backward_speed, self._backward_time):
                return True
        return False

    def _drive_leg(self, goal_handle, speed: float, duration_sec: float) -> bool:
        """한 구간(전진 또는 후진)을 duration_sec만큼 주행. 취소되면 True 리턴."""
        tw = Twist()
        tw.linear.x = speed
        start = time.monotonic()
        while time.monotonic() - start < duration_sec:
            if goal_handle.is_cancel_requested:
                self._cmd_pub.publish(Twist())
                return True
            self._cmd_pub.publish(tw)
            time.sleep(CONTROL_PERIOD_SEC)
        return False

    def _compute_and_finish(self, goal_handle):
        """수집된 GPS/IMU 데이터로 yaw_offset을 계산하고 goal을 마무리."""
        if len(self._gps_points) < self._min_gps_points:
            goal_handle.abort()
            return Calibrate.Result(
                success=False,
                message=f'GPS 데이터 부족: {len(self._gps_points)} < {self._min_gps_points}')
        if len(self._imu_yaws) < self._min_imu_yaws:
            goal_handle.abort()
            return Calibrate.Result(
                success=False,
                message=f'IMU 데이터 부족: {len(self._imu_yaws)} < {self._min_imu_yaws}')

        gps_points = np.array(self._gps_points)
        start_pt = gps_points[0]
        distances = np.linalg.norm(gps_points - start_pt, axis=1)
        max_dist_idx = int(np.argmax(distances))
        forward_vec = gps_points[max_dist_idx] - start_pt
        forward_distance = float(math.hypot(forward_vec[0], forward_vec[1]))
        if forward_distance < self._min_forward_distance_m:
            goal_handle.abort()
            return Calibrate.Result(
                success=False,
                message=f'전진거리 부족: {forward_distance:.2f}m < {self._min_forward_distance_m}m')

        # GPS 궤적의 주성분(PCA/SVD) — 전진+후진 왕복 전체가 대략 한 직선
        # 위에 분포하므로, 그 직선의 방향이 "실제로 움직인 축"이다.
        mean_pt = np.mean(gps_points, axis=0)
        centered = gps_points - mean_pt
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        pc1 = vt[0]
        # PCA는 부호를 모르므로, 시작->최원점(전진 방향) 벡터로 부호를 맞춘다.
        if np.dot(pc1, forward_vec) < 0.0:
            pc1 = -pc1
        gps_angle = math.atan2(pc1[1], pc1[0])

        # IMU yaw 평균 방향 — 각도를 그냥 산술평균하면 랩어라운드(179°/-179°)
        # 문제가 생기므로 단위벡터로 바꿔 평균낸 뒤 각도로 되돌린다.
        imu_yaws = np.array(self._imu_yaws)
        imu_mean_vec = np.mean(np.column_stack([np.cos(imu_yaws), np.sin(imu_yaws)]), axis=0)
        imu_angle = math.atan2(imu_mean_vec[1], imu_mean_vec[0])

        yaw_offset = self._normalize_angle(gps_angle - imu_angle)

        self._current_offset = yaw_offset
        self._offset_valid = True
        self._complete = True
        self._publish_offset()
        self._publish_complete()

        self.get_logger().info(
            f'[CAL] gps_angle={math.degrees(gps_angle):.2f}deg '
            f'imu_angle={math.degrees(imu_angle):.2f}deg '
            f'yaw_offset={math.degrees(yaw_offset):.2f}deg '
            f'forward_distance={forward_distance:.2f}m '
            f'(gps_points={len(self._gps_points)}, imu_yaws={len(self._imu_yaws)})')

        goal_handle.succeed()
        return Calibrate.Result(
            success=True, message=f'yaw_offset={math.degrees(yaw_offset):.2f}deg')


def main():
    """노드 진입점 — `MultiThreadedExecutor`로 상주한다.

    `_execute_callback`이 주행 중 `time.sleep()`으로 블로킹하는 동안에도
    `/imu`·`/odometry/gps` 구독과 주기 발행 타이머가 다른 스레드에서 계속
    돌아야 하므로(기본 `SingleThreadedExecutor`면 다 같이 멈춰버림)
    `MultiThreadedExecutor`를 명시적으로 사용한다.
    """
    rclpy.init()
    node = CalibrationActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
