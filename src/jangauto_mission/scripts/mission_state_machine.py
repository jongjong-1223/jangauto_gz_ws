#!/usr/bin/env python3
"""YASMIN 기반 미션 상태머신 노드.

## 역할
- `/app/control_state`(앱 명령)를 구독해 STOP/KEY/CAL/ALIGN/RUN 중 현재
  모드를 결정한다. 단, 모든 전이가 항상 허용되는 건 아니다 — `ALLOWED_TARGETS`
  참고(STOP/KEY/CAL은 서로 자유, ALIGN은 그 셋에서만, RUN은 ALIGN에서만
  올라갈 수 있고, 내려가는 전이는 항상 자유).
- `/jangauto_mission/error`(내부 에러 보고)도 구독해서, 에러가 보고되면
  이 규칙과 무관하게 즉시 STOP으로 강제 전이한다.
- 명령이 일정 시간 안 오면 TIMEOUT으로 간주해 STOP으로 전이한다(침묵을
  안전 신호가 아닌 위험 신호로 보는 fail-safe 설계).
- 최종 결정된 모드를 `/robot_status`(jangauto_msg/Status)로 발행한다.
- 앱과 핸드셰이크한다: 앱 명령 하나를 처리할 때마다 수락/거부 결과를
  `/app/control_state_ack`로, 상태가 바뀔 때마다 앱 전용 미러를
  `/app/robot_status`로 발행한다(둘 다 `app_websocket_bridge.py`가
  웹소켓으로 앱에 중계).

## 클래스 구성
- `ControlAndErrorMonitor`: 실제 로직을 담당하는 "몸통".
  - 두 토픽을 구독하고, 이벤트를 판단해 outcome을 결정한다.
  - `/robot_status`·`/app/robot_status`·`/app/control_state_ack` 발행도 담당한다.
  - YASMIN State가 아닌 평범한 클래스이며, 인스턴스는 1개만 만들어진다.
- `ControlAndErrorMonitorState`: YASMIN이 요구하는 `State` 형식에 맞춘 얇은 어댑터.
  - 자체 로직 없이 `ControlAndErrorMonitor` 하나를 공유 참조한다.
  - 자신이 어느 상태 이름으로 등록됐는지(`state_name`)만 들고 있다가
    `wait_for_outcome()` 호출 시 넘겨준다 — 상태별로 허용된 outcome
    집합이 다르기 때문(`ALLOWED_TARGETS`).
  - STOP/KEY/CAL/ALIGN/RUN 5개 이름에 각각 하나씩, 총 5개 인스턴스가 등록된다.

## main()의 동작 순서
1. rclpy 초기화, YASMIN 싱글턴 노드 획득
2. `/robot_status` 퍼블리셔 생성(latched QoS)
3. `ControlAndErrorMonitor` 생성 → 이 시점에 두 토픽 구독이 시작됨
4. 빈 `StateMachine` 생성
5. STOP/KEY/CAL/ALIGN/RUN 5개 상태 등록 — 상태마다 `ALLOWED_TARGETS`로부터
   outcome 목록/전이표를 따로 구성(모두 같은 monitor 공유)
6. 시작 상태를 STOP으로 지정
7. yasmin_viewer 시각화 등록(선택, 디버깅용)
8. 상태머신 실행(`sm()`) — Ctrl+C 전까지 리턴하지 않는 블로킹 호출
9. 종료 시 정리

`ros2 run`/launch로 실행하면 8번 호출이 계속 이벤트(명령/에러/타임아웃)를
처리하며 도는 상시 상주 노드로 동작한다.
"""

import json
import queue

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from jangauto_msg.msg import Status

from yasmin import State, StateMachine
from yasmin_ros.yasmin_node import YasminNode
from yasmin_viewer import YasminViewerPub

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

# 상태 전이 순서 규칙: 이 현재 상태에서 앱 명령으로 직접 갈 수 있는 목표 모드 집합.
# - STOP/KEY/CAL은 서로 언제든 자유롭게 오간다.
# - ALIGN은 STOP/KEY/CAL에서만 올라갈 수 있다.
# - RUN은 ALIGN에서만 올라갈 수 있다.
# - 내려가는 전이(RUN/ALIGN -> STOP/KEY/CAL, RUN -> ALIGN)는 항상 자유다.
# (자기 자신도 포함 — 같은 모드 재요청은 항상 허용되는 no-op accept.)
ALLOWED_TARGETS = {
    "STOP":  {"STOP", "KEY", "CAL", "ALIGN"},
    "KEY":   {"STOP", "KEY", "CAL", "ALIGN"},
    "CAL":   {"STOP", "KEY", "CAL", "ALIGN"},
    "ALIGN": {"STOP", "KEY", "CAL", "ALIGN", "RUN"},
    "RUN":   {"STOP", "KEY", "CAL", "ALIGN", "RUN"},
}

CONTROL_STATE_TOPIC = "/app/control_state"          # 구독: app_websocket_bridge가 재발행하는 앱 명령
ERROR_TOPIC = "/jangauto_mission/error"             # 구독: 내부 에러 보고 채널
STATUS_TOPIC = "/robot_status"                      # 발행: 현재 결정된 모드(ROS 전역, typed)
APP_STATUS_TOPIC = "/app/robot_status"              # 발행: 앱 전용 상태 미러(JSON 문자열)
APP_CONTROL_ACK_TOPIC = "/app/control_state_ack"    # 발행: 앱 명령 1건당 1개, 수락/거부 응답

# control_state가 이 시간(초) 이상 안 오면 TIMEOUT outcome -> STOP(fail-safe).
CONTROL_TIMEOUT_SEC = 5.0
CONTROL_MAX_RETRY = 1


class ControlAndErrorMonitor:
    """상태 판단 로직의 몸통. `/app/control_state`·`/jangauto_mission/error`
    구독과 상태/핸드셰이크 발행 전부를 전담하며, 인스턴스는 시스템 전체에 1개뿐이다.

    직접 구현한 이유:
    - `yasmin_ros.MonitorState`는 토픽 1개만 지원한다.
    - "앱 명령 + 내부 에러, 두 소스가 같은 목적지(STOP)로 전이"는 토픽
      1개로는 표현할 수 없다.

    로직을 State 클래스가 아닌 여기 둔 이유는 `ControlAndErrorMonitorState`
    설명 참고.
    """

    def __init__(self, node, status_pub, app_status_pub, app_ack_pub) -> None:
        """
        Args:
            node: 구독/로거 생성에 쓰는 rclpy 노드(YASMIN 싱글턴).
            status_pub: `/robot_status` 발행용 Publisher(QoS는 `main()`에서 구성).
            app_status_pub: `/app/robot_status`(앱 전용 JSON 미러) 발행용 Publisher.
            app_ack_pub: `/app/control_state_ack`(앱 명령 수락/거부 응답) 발행용 Publisher.
        """
        self._node = node
        self._status_pub = status_pub
        self._app_status_pub = app_status_pub
        self._app_ack_pub = app_ack_pub
        # 구독 콜백 스레드와 wait_for_outcome() 실행 스레드를 잇는 이벤트 큐.
        self._queue: "queue.Queue" = queue.Queue()
        # 마지막 발행 내용 — 중복 발행 방지(dedup)용 비교 기준.
        self._last_mode = None
        self._last_in_error = False

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

    def _publish_status(self, mode: str, in_error: bool, error_reason: str) -> None:
        """`/robot_status`(ROS 전역)와 `/app/robot_status`(앱 전용 미러)를 발행.

        - 이 노드가 시스템 전역/앱에 상태를 알리는 유일한 통로다.
        - 직전과 (mode, in_error)가 같으면 재발행하지 않는다.
        - ERROR/TIMEOUT처럼 앱 명령이 원인이 아닌 전이도 여기로 들어오므로,
          앱 명령 하나당 1번 나가는 ack와 달리 이 발행은 원인과 무관하게 일어난다.
        """
        if mode == self._last_mode and in_error == self._last_in_error:
            return
        status = Status()
        status.header.stamp = self._node.get_clock().now().to_msg()
        status.mode = mode
        status.in_error = in_error
        status.error_reason = error_reason if in_error else ""
        self._status_pub.publish(status)

        app_msg = String()
        app_msg.data = json.dumps({
            "mode": mode,
            "in_error": in_error,
            "error_reason": status.error_reason,
        })
        self._app_status_pub.publish(app_msg)

        self._last_mode = mode
        self._last_in_error = in_error

    def _publish_ack(self, sw_bits, requested_mode: str, accepted: bool,
                      current_mode: str, reason: str) -> None:
        """`/app/control_state_ack` 발행 — 앱이 보낸 명령 하나에 대한 응답.

        - `sw_bits`/`requested_mode`로 앱이 보낸 요청을 그대로 echo한다
          (요청ID 없이, 보낸 값 자체가 correlation 역할을 한다).
        - `current_mode`가 로봇의 실제 현재 상태 — 앱은 수락/거부 여부와
          무관하게 이 값에 자기 화면을 맞추면 된다(로봇이 authoritative).
        """
        msg = String()
        msg.data = json.dumps({
            "sw_bits": sw_bits,
            "requested_mode": requested_mode,
            "accepted": accepted,
            "current_mode": current_mode,
            "reason": reason,
        })
        self._app_ack_pub.publish(msg)

    def wait_for_outcome(self, current_state_name: str) -> str:
        """다음 이벤트가 확정될 때까지 대기하는, 이 노드의 핵심 판단부.
        YASMIN State의 execute()가 실질적으로 위임하는 지점이 여기다.

        Args:
            current_state_name: 호출한 State가 등록된 상태 이름
                ("STOP"/"KEY"/"CAL"/"ALIGN"/"RUN"). 앱이 요청한 목표 모드가
                `ALLOWED_TARGETS[current_state_name]`에 있는지 검사하는 데 쓴다.

        Returns:
            - "APP_TO_<모드>": 앱 명령이 수락되어 결정된 다음 모드(예: "APP_TO_RUN").
            - "ERROR": 내부 에러 보고 수신 — 무조건 STOP.
            - "TIMEOUT": 명령 부재 — 안전을 위해 STOP.

            (ERROR/TIMEOUT은 `main()`의 transitions에서 둘 다 STOP으로 매핑됨.
            거부된 요청은 outcome을 내지 않고 계속 대기한다 — 아래 참고.)
        """
        retries = 0
        while True:
            try:
                kind, data = self._queue.get(timeout=CONTROL_TIMEOUT_SEC)
            except queue.Empty:
                retries += 1
                if retries > CONTROL_MAX_RETRY:
                    self._publish_status("STOP", True, "no command received (timeout)")
                    return "TIMEOUT"
                continue

            if kind == "error":
                if data:
                    self._publish_status("STOP", True, data)
                    return "ERROR"
                self._last_in_error = False
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

            if target not in ALLOWED_TARGETS[current_state_name]:
                # 순서 규칙 위반 — 전이 없이 거부만 응답하고 계속 대기.
                self._publish_ack(
                    sw_bits, target, False, current_state_name,
                    f"{current_state_name} -> {target} not allowed")
                continue

            retries = 0
            self._publish_status(target, False, "")
            self._publish_ack(sw_bits, target, True, target, "")
            return f"APP_TO_{target}"


class ControlAndErrorMonitorState(State):
    """YASMIN `StateMachine`에 실제로 등록되는 State. 자체 로직은 없고
    `ControlAndErrorMonitor` 하나를 공유 참조해 위임하는 얇은 어댑터다.

    몸통(로직)과 껍데기(YASMIN 인터페이스)를 분리한 이유:
    - 모든 상태의 판단 로직이 동일하다.
    - 그런데 `yasmin_viewer`가 상태 이름마다 서로 다른 State 객체를
      요구한다(같은 인스턴스 재사용 시 시각화가 깨짐 — 실행 확인됨).
    - 그래서 로직은 하나만 두고 껍데기만 5개 만들어야 했다.
    """

    def __init__(self, monitor: ControlAndErrorMonitor, state_name: str,
                 outcomes: list) -> None:
        """
        Args:
            monitor: STOP/KEY/CAL/ALIGN/RUN 5개 인스턴스가 공유할 몸통 객체.
            state_name: 이 State가 등록되는 상태 이름 — `wait_for_outcome()`에
                그대로 전달해 허용된 목표 모드를 판단하는 데 쓰인다.
            outcomes: 이 상태가 낼 수 있는 outcome 목록(`main()`에서
                `ALLOWED_TARGETS[state_name]`으로부터 구성해 넘겨줌).
        """
        super().__init__(outcomes)
        self._monitor = monitor
        self._state_name = state_name

    def execute(self, blackboard) -> str:
        """상태 진입 시 YASMIN이 호출하는 지점. 판단은 전부 monitor에 위임한다."""
        return self._monitor.wait_for_outcome(self._state_name)


def _build_outcomes_and_transitions(state_name: str):
    """`ALLOWED_TARGETS[state_name]`으로부터 이 상태의 outcome 목록과
    전이표(outcome -> 다음 상태 이름)를 구성한다.

    - 앱 명령으로 낼 수 있는 outcome은 목표 모드마다 `"APP_TO_<모드>"` 형태
      (예: STOP 상태는 ALIGN까지만 허용되므로 `"APP_TO_RUN"`이 아예 없음).
    - ERROR/TIMEOUT은 모든 상태에 공통이며 항상 STOP으로 귀결된다.
    """
    transitions = {f"APP_TO_{target}": target for target in ALLOWED_TARGETS[state_name]}
    transitions["ERROR"] = "STOP"
    transitions["TIMEOUT"] = "STOP"
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
    # /app/robot_status도 /robot_status와 같은 latched QoS를 쓴다 — app_websocket_bridge.py가
    # mission_state_machine.py보다 늦게 뜨거나 재시작해도, 구독하는 즉시 마지막 상태를
    # 받아서 자기 캐시(신규 웹소켓 접속자에게 즉시 보내주는 값)를 채울 수 있어야 하기 때문
    # (기본 QoS는 volatile이라 늦게 구독하면 그 사이의 마지막 메시지를 영영 못 받음).
    app_status_pub = node.create_publisher(String, APP_STATUS_TOPIC, status_qos)
    # ack는 매 명령에 대한 1회성 응답이라 latch할 대상이 없음 — 기본 QoS(depth 10) 그대로.
    app_ack_pub = node.create_publisher(String, APP_CONTROL_ACK_TOPIC, 10)

    # 몸통은 여기서 딱 1개 생성 — 생성 시점에 두 토픽 구독이 시작된다.
    monitor = ControlAndErrorMonitor(node, status_pub, app_status_pub, app_ack_pub)

    # 최상위 StateMachine 자체는 정상 운용 중엔 끝나지 않는다 —
    # "SHUTDOWN"은 handle_sigint(Ctrl+C)를 위한 형식상 outcome.
    sm = StateMachine(outcomes=["SHUTDOWN"], handle_sigint=True)

    # 상태마다 ALLOWED_TARGETS로부터 outcome/전이표를 따로 구성해서 등록한다
    # (예전엔 5개 상태가 완전연결이라 동일한 transitions 하나를 공유했지만,
    # 이제 상태별로 허용된 목표가 달라 공유할 수 없다).
    for mode in MODES:
        outcomes, transitions = _build_outcomes_and_transitions(mode)
        sm.add_state(mode, ControlAndErrorMonitorState(monitor, mode, outcomes),
                     transitions=transitions)

    # 부팅 직후 명령이 오기 전에도 안전하도록 STOP에서 시작.
    sm.set_start_state("STOP")

    # yasmin_viewer 웹 UI용 시각화(디버깅용, /robot_status와는 별개 채널).
    # 주의: 실제 __init__ 시그니처는 (fsm, fsm_name, ...) — fsm이 먼저다.
    # 설치된 yasmin_viewer_pub.py의 Args 독스트링 순서는 반대라 착오하기 쉬움(문서 버그).
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
