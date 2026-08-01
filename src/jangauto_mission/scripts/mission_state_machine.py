#!/usr/bin/env python3
"""YASMIN 기반 미션 상태머신 노드.

## 역할
- `/app/control_state`(앱 명령) 구독 → STOP/KEY/CAL/ALIGN/RUN 중 모드 결정.
  전이 가능 여부는 `ALLOWED_TARGETS` 규칙을 따른다(내려가는 전이는 항상 자유).
- `/jangauto_mission/error`(내부 에러) 구독 → 보고되면 규칙 무시하고 즉시 STOP.
- **in_error는 한 번 켜지면 영구 래치** — 진단이 회복되거나 앱이 어떤 명령을
  보내도 소프트웨어적으로 안 풀린다(물리 E-stop 리셋과 동일한 개념). 실제
  복구는 원인을 고치고 이 노드를 재시작해야만 가능.
- 앱은 선택 장비다(없어도 로봇이 동작해야 함) — `control_state`가 안 와도
  침묵을 에러로 취급하지 않는다. 명령을 기다리는 동안은 그냥 현재 상태를
  유지(부팅 직후 기본값은 STOP)하며 무한 대기한다.
- 결정된 모드를 `/robot_status`(jangauto_msg/Status)로 발행 — 전이 시 즉시 1회 +
  `status_publish_period_sec` 주기로 재발행(변화 없어도 계속 나감). 앱에 보여줄 JSON
  조립·명령별 개별 응답(ack)은 이제 이 노드의 책임이 아니다 — `app_websocket_bridge.py`가
  `/robot_status`를 구독해서 직접 조립해 앱에 보낸다(ack 없이 이 토픽의 `current_state`
  변화만으로 앱이 수락 여부를 판단).
- CAL/ALIGN/RUN은 상태 진입 시 액션 서버(`calibrate`/`align`/`run` — CAL/RUN은
  구현됨, ALIGN은 현재 TODO placeholder)에 goal을 보내고, 결과도 앱 명령/
  에러와 같은 큐에 합류시켜 먼저 끝나는 쪽으로 outcome 결정(`STATE_ACTIONS`,
  `wait_for_outcome()`). YASMIN `Concurrence`는 "모든 자식 완료 후 조합이
  매칭돼야 하는" AND 방식이라 이 fan-in 용도에 안 맞아, 기존
  `ControlAndErrorMonitor` 큐 방식을 그대로 확장해 씀.
- CAL/RUN 액션 성공으로 인한 self-loop(자기 자신에 재전이)는 다음 진입 때
  goal의 `self_loop` 필드로 액션 서버에 알려진다 — 액션 서버는 이걸로
  "방금 성공해서 자동으로 다시 들어온 것"과 "다른 상태에 있다가 재진입한
  것"을 구분해 전자는 재측정/재주행 없이 대기만 한다(RUN 완주 후에도
  sw_bits가 안 내려가면 처음부터 재주행해버리던 버그 수정).

## 클래스 구성
- `ControlAndErrorMonitor`: 앱 명령/내부 에러/액션 결과 세 이벤트를 판단해
  outcome을 정하고, `/robot_status` 발행까지 전담하는 몸통(인스턴스 1개).
- `ControlAndErrorMonitorState`: YASMIN `State` 어댑터. 자체 로직 없이 monitor에
  위임하며, STOP/KEY/CAL/ALIGN/RUN 5개 인스턴스로 등록된다.

## main()의 동작 순서
1. rclpy 초기화, YASMIN 싱글턴 노드 획득
2. `/robot_status` 발행자 생성(latched QoS)
3. `ControlAndErrorMonitor` 생성(두 토픽 구독 시작) → `/robot_status` 주기 재발행 타이머 등록
4. 빈 `StateMachine` 생성, 5개 상태 등록(상태별 outcome/전이표는 `ALLOWED_TARGETS`
   기반, CAL/ALIGN/RUN은 `STATE_ACTIONS`의 액션 타입/이름도 같이 등록)
5. 시작 상태를 STOP으로 지정
6. yasmin_viewer 시각화 등록(디버깅용)
7. 상태머신 실행(`sm()`) — Ctrl+C 전까지 블로킹
8. 종료 시 정리
"""

import json
import queue

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from jangauto_msg.msg import Status

from yasmin import State, StateMachine
from yasmin_ros.yasmin_node import YasminNode
from yasmin_viewer import YasminViewerPub

from jangauto_msg.action import Align, Calibrate, Run

# sw_bits -> 상태 이름. jangauto_hmi/scripts/reference/app_bridge.py의
# sw_command_map과 동일한 매핑(예전 0/1 문자열 대신 정수 비트값 하나로 옴).
SW_BITS_TO_MODE = {
    16: "STOP",
    8: "KEY",
    4: "CAL",
    2: "ALIGN",
    1: "RUN",
}
# YASMIN에 등록할 상태 이름 목록. SW_BITS_TO_MODE의 값(value) 집합과 일치해야 함.
MODES = ["STOP", "KEY", "CAL", "ALIGN", "RUN"]

# 상태 전이 규칙: 현재 상태에서 앱 명령으로 갈 수 있는 목표 모드 집합.
# - STOP/KEY/CAL은 서로 자유, ALIGN은 이 셋에서만, RUN은 ALIGN에서만 진입.
# - 내려가는 전이는 항상 자유. 자기 자신 포함(재요청은 no-op accept).
ALLOWED_TARGETS = {
    "STOP":  {"STOP", "KEY", "CAL", "ALIGN"},
    "KEY":   {"STOP", "KEY", "CAL", "ALIGN"},
    "CAL":   {"STOP", "KEY", "CAL", "ALIGN"},
    "ALIGN": {"STOP", "KEY", "CAL", "ALIGN", "RUN"},
    "RUN":   {"STOP", "KEY", "CAL", "ALIGN", "RUN"},
}

# CAL/ALIGN/RUN처럼 실제 작업이 있는 상태 -> (액션 타입, 액션 이름).
# 나머지(STOP/KEY)는 감시만 한다. CAL은 calibration_action_server.py가
# GPS-IMU 비교로 실제 캘리브레이션을, RUN은 run_action_server.py가 선택된
# 커버리지 경로의 Nav2 주행을 수행한다. ALIGN은 아직 TODO placeholder(즉시
# 성공 리턴) — 실제 알고리즘은 미정.
STATE_ACTIONS = {
    "CAL": (Calibrate, "calibrate"),
    "ALIGN": (Align, "align"),
    "RUN": (Run, "run"),
}

CONTROL_STATE_TOPIC = "/app/control_state"          # 구독: app_websocket_bridge가 재발행하는 앱 명령
ERROR_TOPIC = "/jangauto_mission/error"             # 구독: 내부 에러 보고 채널
STATUS_TOPIC = "/robot_status"                      # 발행: 현재 결정된 모드(ROS 전역, typed) —
                                                     # app_websocket_bridge.py가 이걸 구독해 앱 JSON을 직접 조립함


class ControlAndErrorMonitor:
    """상태 판단 로직의 몸통 — `/app/control_state`·`/jangauto_mission/error` 구독과
    상태/핸드셰이크 발행을 전담하는, 시스템 전체에 1개뿐인 인스턴스.

    `yasmin_ros.MonitorState`(토픽 1개만 지원) 대신 직접 구현한 이유: 앱 명령과
    내부 에러라는 두 소스가 같은 목적지(STOP)로 전이해야 하기 때문.
    로직을 State 클래스가 아닌 여기 둔 이유는 `ControlAndErrorMonitorState` 참고.
    """

    def __init__(self, node, status_pub) -> None:
        """
        Args:
            node: 구독/로거 생성에 쓰는 rclpy 노드(YASMIN 싱글턴).
            status_pub: `/robot_status` 발행용 Publisher(QoS는 `main()`에서 구성).
        """
        self._node = node
        self._status_pub = status_pub
        # 구독 콜백 스레드와 wait_for_outcome() 실행 스레드를 잇는 이벤트 큐.
        # 앱 명령/에러뿐 아니라 액션 결과("action_done")도 같은 큐로 합류시켜
        # "여러 소스 중 먼저 온 것 하나를 처리"하는 동일한 fan-in 패턴을 쓴다.
        self._queue: "queue.Queue" = queue.Queue()
        # 마지막 발행 내용 — 중복 발행 방지(dedup)용 비교 기준이자,
        # publish_status_tick()이 주기 재발행할 때 쓰는 캐시. 초깃값을 실제
        # 시작 상태(STOP)로 맞춰둬서, 명령이 한 번도 안 와도 주기 타이머가
        # 부팅 직후부터 바로 "STOP"을 흘려보낸다(앱이 로봇 생존 여부를
        # `/robot_status` 수신만으로 판단하므로, 첫 이벤트 전까지 토픽이
        # 완전히 비어있으면 안 됨).
        self._last_current_state = "STOP"
        self._last_in_error = False
        self._last_error_reason = ""
        # 액션 이름 -> ActionClient. 상태 진입마다 새로 만들지 않고 재사용.
        self._action_clients: dict = {}
        # 방금 액션 성공으로 self-loop 전이(APP_TO_<자기자신>)를 만든 상태
        # 이름 — wait_for_outcome() 다음 호출 진입 시 이걸 자기 이름과
        # 비교해 "이번 진입이 self-loop인지"를 판별하는 데 쓰고 바로
        # 초기화한다(딱 다음 한 번만 반영). CAL이 self-loop과 외부 재진입을
        # 구분해 goal에 실어 보내는 데 사용(다른 상태는 아직 구분 안 함).
        self._last_action_success_state = None

        self._control_sub = self._node.create_subscription(
            String, CONTROL_STATE_TOPIC, self._on_control, 10)
        self._error_sub = self._node.create_subscription(
            String, ERROR_TOPIC, self._on_error, 10)

    def _on_control(self, msg: String) -> None:
        """control_state 구독 콜백 — 판단은 하지 않고 큐에 적재만 한다."""
        self._queue.put(("control", msg.data))

    def _on_error(self, msg: String) -> None:
        """error 구독 콜백 — 마찬가지로 큐에 적재만 한다."""
        self._queue.put(("error", msg.data))

    def _publish_status(self, current_state: str, in_error: bool, error_reason: str) -> None:
        """`/robot_status` 발행 — 상태 전이를 알리는 즉시 경로.

        - 직전과 (current_state, in_error)가 같으면 재발행하지 않는다(전이
          시에만 반응하는 ROS 내부 소비자를 위한 경로 — 값이 같으면 이벤트가
          아니므로 스킵).
        - ERROR처럼 앱 명령이 원인이 아닌 전이도 포함.
        - 이 뒤 앱에 보여줄 JSON 조립은 하지 않는다 — app_websocket_bridge.py가
          `/robot_status`를 직접 구독해서 처리한다.
        """
        if current_state == self._last_current_state and in_error == self._last_in_error:
            return
        status = Status()
        status.header.stamp = self._node.get_clock().now().to_msg()
        status.current_state = current_state
        status.in_error = in_error
        status.error_reason = error_reason if in_error else ""
        self._status_pub.publish(status)

        self._last_current_state = current_state
        self._last_in_error = in_error
        self._last_error_reason = status.error_reason

    def publish_status_tick(self) -> None:
        """주기 타이머 콜백 — 마지막으로 알려진 상태를 dedup 없이 그대로
        재발행한다. ack가 없어진 지금, 앱이 명령 수락/거부를 알 수 있는 유일한
        통로가 `/robot_status`(를 구독하는 app_websocket_bridge.py)이므로, 상태가
        안 바뀌어도 계속 흘려보내야 늦게 붙는 구독자·앱이 최신 값을 놓치지 않는다.
        `_last_current_state`가 생성자에서 이미 "STOP"으로 초기화돼있어 부팅
        직후 첫 틱부터 바로 발행된다.
        """
        status = Status()
        status.header.stamp = self._node.get_clock().now().to_msg()
        status.current_state = self._last_current_state
        status.in_error = self._last_in_error
        status.error_reason = self._last_error_reason
        self._status_pub.publish(status)

    def _get_action_client(self, action_type, action_name: str) -> ActionClient:
        client = self._action_clients.get(action_name)
        if client is None:
            client = ActionClient(self._node, action_type, action_name)
            self._action_clients[action_name] = client
        return client

    def wait_for_outcome(self, current_state_name: str,
                          action_type=None, action_name: str = None) -> str:
        """다음 이벤트가 확정될 때까지 대기하는 핵심 판단부(State.execute()가 위임하는 지점).

        Args:
            current_state_name: 호출한 상태 이름. `ALLOWED_TARGETS`로 목표 모드
                허용 여부를 판단하는 데 쓴다.
            action_type, action_name: 이 상태의 액션(`STATE_ACTIONS`) — 있으면
                진입과 동시에 goal을 보내고 결과도 같은 큐로 합류시켜 먼저
                끝나는 쪽을 outcome으로 쓴다. 둘 다 None이면 순수 감시(STOP/KEY).

        Returns:
            - "APP_TO_<모드>": 앱 명령 수락, 또는 이 상태 액션 성공(self-loop).
            - "ERROR": 에러 보고 또는 액션 실패 — 무조건 STOP.
            (거부된 요청·명령 부재는 outcome 없이 계속 대기 — 앱은 선택
            장비라 침묵을 에러로 취급하지 않는다)
        """
        # 이번 진입이 self-loop(방금 이 상태 자신의 액션 성공으로 자기
        # 자신에 재전이한 경우)인지 판별 — 딱 이번 호출에만 반영되도록
        # 바로 초기화한다.
        is_self_loop = (self._last_action_success_state == current_state_name)
        self._last_action_success_state = None

        # 액션이 있으면 진입과 동시에 goal 전송, 결과를 같은 큐로 흘려보낸다.
        # still_relevant: 이 호출 종료 후 뒤늦게 도착하는 결과가 다음 호출의
        # 큐 소비 로직을 오염시키지 않도록 막는 가드.
        goal_handle_holder = [None]
        still_relevant = [True]
        if action_type is not None:
            client = self._get_action_client(action_type, action_name)

            # 서버 디스커버리 전에 goal을 보내면 조용히 유실된다(확인됨) — 먼저 대기.
            if not client.wait_for_server(timeout_sec=5.0):
                self._node.get_logger().warning(
                    f"Action server '{action_name}' not available")
                self._queue.put(("action_done", False))
            else:
                def _on_goal_response(future):
                    if not still_relevant[0]:
                        return
                    goal_handle = future.result()
                    if not goal_handle.accepted:
                        self._queue.put(("action_done", False))
                        return
                    goal_handle_holder[0] = goal_handle
                    result_future = goal_handle.get_result_async()

                    def _on_result(rfuture):
                        if still_relevant[0]:
                            self._queue.put(("action_done", rfuture.result().result.success))

                    result_future.add_done_callback(_on_result)

                # CAL/RUN은 self-loop(대기만)와 외부 재진입(실제 수행)을
                # 구분해야 해서 goal에 그 판단을 실어 보낸다 — RUN이 이게
                # 없으면 완주 후에도 sw_bits가 RUN이면 매번 처음부터
                # 재주행해버린다(확인된 버그). ALIGN은 아직 이 구분이
                # 필요 없어 빈 Goal() 그대로.
                if action_type is Calibrate:
                    goal = Calibrate.Goal(self_loop=is_self_loop)
                elif action_type is Run:
                    goal = Run.Goal(self_loop=is_self_loop)
                else:
                    goal = action_type.Goal()
                client.send_goal_async(goal).add_done_callback(_on_goal_response)

        try:
            while True:
                # 앱은 선택 장비다 — 타임아웃 없이 다음 이벤트(명령/에러/액션결과)를
                # 그냥 무한 대기한다. 명령이 안 온다고 해서 위험 신호로 보지 않는다.
                kind, data = self._queue.get()

                if self._last_in_error:
                    # in_error는 한 번 켜지면 이 프로세스가 살아있는 동안 영구
                    # 래치다 — 진단이 회복돼 "Cleared"가 와도, 앱이 어떤
                    # control_state를 보내도 여기서 전부 무시하고 계속 STOP에
                    # 머무른다. 실제 복구는 원인을 고치고 mission_state_machine
                    # 노드를 재시작해야만 가능(물리 E-stop을 리셋하는 것과 동일한
                    # 개념 — 소프트웨어적으로 조용히 풀리면 안 되는 안전 요구사항).
                    continue

                if kind == "action_done":
                    success = data
                    if success:
                        self._publish_status(current_state_name, False, "")
                        self._last_action_success_state = current_state_name
                        return f"APP_TO_{current_state_name}"
                    self._publish_status("STOP", True, f"{current_state_name} action failed")
                    return "ERROR"

                if kind == "error":
                    if data:
                        self._publish_status("STOP", True, data)
                        return "ERROR"
                    # data가 빈 문자열(진단 회복)이어도 위의 영구 래치 방침상
                    # in_error를 되돌리지 않는다 — 이 분기는 in_error가 아직
                    # 한 번도 안 켜졌을 때만 의미 있는 no-op.
                    continue

                # kind == "control": 앱 명령 해석
                try:
                    payload = json.loads(data)
                    sw_bits = payload["sw_bits"]
                except (json.JSONDecodeError, KeyError, TypeError):
                    continue

                target = SW_BITS_TO_MODE.get(sw_bits)
                if target is None:
                    self._node.get_logger().warning(f"Unknown sw_bits value: {sw_bits!r}")
                    continue

                if target == current_state_name:
                    # 이미 이 상태인데 같은 목표로 재요청(앱이 하트비트로 현재
                    # sw_bits를 계속 재전송하는 정상 상황 포함) — 진짜 no-op.
                    # 이걸 전이로 취급해 재진입시키면 CAL처럼 액션이 걸린
                    # 상태에서는 진행 중인 액션이 매번 취소·재시작돼버린다.
                    continue

                if target not in ALLOWED_TARGETS[current_state_name]:
                    # 순서 규칙 위반 — 전이 없이 그냥 무시하고 계속 대기(개별
                    # 응답 채널 없음 — 앱은 /robot_status의 current_state가 안 바뀌는 것으로 판단).
                    continue

                self._publish_status(target, False, "")
                return f"APP_TO_{target}"
        finally:
            # 리턴 사유(명령/에러/타임아웃/액션결과)와 무관하게 이 호출의 액션은
            # 더 이상 유효하지 않다 — 진행 중이면 취소한다.
            still_relevant[0] = False
            if goal_handle_holder[0] is not None:
                goal_handle_holder[0].cancel_goal_async()


class ControlAndErrorMonitorState(State):
    """YASMIN `StateMachine`에 실제로 등록되는 State. 자체 로직 없이
    `ControlAndErrorMonitor` 하나를 공유 참조해 위임하는 얇은 어댑터다.

    로직은 모든 상태가 동일한데 껍데기만 5개 만든 이유: `yasmin_viewer`가
    상태마다 별도 State 객체를 요구한다(인스턴스 재사용 시 시각화 깨짐, 확인됨).
    """

    def __init__(self, monitor: ControlAndErrorMonitor, state_name: str,
                 outcomes: list, action: tuple = None) -> None:
        """
        Args:
            monitor: STOP/KEY/CAL/ALIGN/RUN 5개 인스턴스가 공유할 몸통 객체.
            state_name: 이 State가 등록되는 상태 이름 — `wait_for_outcome()`에
                그대로 전달해 허용된 목표 모드를 판단하는 데 쓰인다.
            outcomes: 이 상태가 낼 수 있는 outcome 목록(`main()`에서
                `ALLOWED_TARGETS[state_name]`으로부터 구성해 넘겨줌).
            action: `(action_type, action_name)` 튜플 — 이 상태에 실제
                작업이 있으면(`STATE_ACTIONS`) 전달됨, 없으면 None.
        """
        super().__init__(outcomes)
        self._monitor = monitor
        self._state_name = state_name
        self._action = action

    def execute(self, blackboard) -> str:
        """상태 진입 시 YASMIN이 호출하는 지점. 판단은 전부 monitor에 위임한다."""
        action_type, action_name = self._action if self._action else (None, None)
        return self._monitor.wait_for_outcome(self._state_name, action_type, action_name)


def _build_outcomes_and_transitions(state_name: str):
    """`ALLOWED_TARGETS[state_name]`으로부터 이 상태의 outcome 목록과
    전이표(outcome -> 다음 상태 이름)를 구성한다.

    - 앱 명령으로 낼 수 있는 outcome은 목표 모드마다 `"APP_TO_<모드>"` 형태
      (예: STOP 상태는 ALIGN까지만 허용되므로 `"APP_TO_RUN"`이 아예 없음).
    - ERROR는 모든 상태에 공통이며 항상 STOP으로 귀결된다.
    """
    transitions = {f"APP_TO_{target}": target for target in ALLOWED_TARGETS[state_name]}
    transitions["ERROR"] = "STOP"
    return list(transitions.keys()), transitions


def main(args=None):
    """노드 진입점 — 클래스 두 개를 조립해서 상태머신을 만들고 실행한다.
    각 단계의 의미는 모듈 docstring의 "main()의 동작 순서" 참고.
    """
    rclpy.init(args=args)

    node = YasminNode.get_instance()

    # /robot_status QoS:
    # - RELIABLE: 유실 시 재전송
    # - TRANSIENT_LOCAL: 늦게 붙는 구독자도 최신 값을 즉시 받음(latched)
    # - KEEP_LAST depth=1: 최신 1개만 의미 있음
    status_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    status_pub = node.create_publisher(Status, STATUS_TOPIC, status_qos)

    # 몸통은 여기서 1개만 생성 — 생성 시점에 두 토픽 구독이 시작된다.
    monitor = ControlAndErrorMonitor(node, status_pub)

    # /robot_status 주기 재발행 — 상태가 안 바뀌어도 계속 흘려보내야 app_websocket_bridge.py를
    # 거쳐 앱까지 가는 유일한 상태 채널(ack 없음)이 끊기지 않는다.
    node.declare_parameter('status_publish_period_sec', 0.5)
    status_publish_period_sec = node.get_parameter(
        'status_publish_period_sec').get_parameter_value().double_value
    node.create_timer(status_publish_period_sec, monitor.publish_status_tick)

    # 최상위 StateMachine 자체는 정상 운용 중엔 끝나지 않는다 —
    # "SHUTDOWN"은 handle_sigint(Ctrl+C)를 위한 형식상 outcome.
    sm = StateMachine(outcomes=["SHUTDOWN"], handle_sigint=True)

    # 상태별로 ALLOWED_TARGETS 기반 outcome/전이표를 따로 구성해 등록한다
    # (허용된 목표가 상태마다 달라 공유 불가). STATE_ACTIONS에 있는 상태
    # (CAL/ALIGN)는 액션 타입/이름도 같이 넘겨 진입과 동시에 감시하게 한다.
    for mode in MODES:
        outcomes, transitions = _build_outcomes_and_transitions(mode)
        state = ControlAndErrorMonitorState(
            monitor, mode, outcomes, action=STATE_ACTIONS.get(mode))
        sm.add_state(mode, state, transitions=transitions)

    # 부팅 직후 명령이 오기 전에도 안전하도록 STOP에서 시작.
    sm.set_start_state("STOP")

    # yasmin_viewer 시각화(디버깅용). 실제 시그니처는 (fsm, fsm_name, ...) —
    # 설치된 패키지의 독스트링 순서는 반대라 착오하기 쉬움(문서 버그, 확인됨).
    YasminViewerPub(sm, "JANGAUTO_MISSION")

    try:
        # 상태머신 실행 시작점. Ctrl+C 전까지 리턴하지 않는다.
        sm()
    finally:
        YasminNode.destroy_instance()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
