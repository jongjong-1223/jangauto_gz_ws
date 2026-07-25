#!/usr/bin/env python3
"""YASMIN 기반 미션 상태머신.

/app/control_state(앱 명령)와 /jangauto_mission/error(내부 에러 보고)를 함께
구독하며 STOP/KEY/CAL/ALIGN/RUN 5개 상태를 오가고, 현재 상태를
jangauto_msg/Status로 /robot_status에 발행한다.
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

# sw_bits -> 상태 이름 (jangauto_hmi/scripts/reference/app_bridge.py의 sw_command_map과 동일한 매핑)
SW_BITS_TO_MODE = {
    16: "STOP",
    8: "KEY",
    4: "CAL",
    2: "ALIGN",
    1: "RUN",
}
MODES = ["STOP", "KEY", "CAL", "ALIGN", "RUN"]

CONTROL_STATE_TOPIC = "/app/control_state"
ERROR_TOPIC = "/jangauto_mission/error"
STATUS_TOPIC = "/robot_status"

# 이 시간 동안 /app/control_state 메시지가 하나도 안 오면 TIMEOUT outcome -> STOP
CONTROL_TIMEOUT_SEC = 5.0
CONTROL_MAX_RETRY = 1


class ControlAndErrorMonitor:
    """/app/control_state와 /jangauto_mission/error 구독 + 큐잉을 담당하는
    공유 헬퍼. 5개 상태(STOP/KEY/CAL/ALIGN/RUN) 전부가 이 헬퍼 하나를
    공유한다 — 구독은 딱 1쌍만 만들고, 상태가 바뀌어도 큐에 쌓인 메시지가
    유실되지 않는다.

    MonitorState는 토픽 1개만 지원해서 "앱 명령 + 내부 에러, 두 소스 -> 같은
    목적지 전이"를 표현할 수 없다 — 이 클래스는 MonitorState 내부 패턴(구독
    유지 + 블로킹 대기)을 2토픽 버전으로 직접 구현한 것이다.
    """

    def __init__(self, node, status_pub) -> None:
        self._node = node
        self._status_pub = status_pub
        self._queue: "queue.Queue" = queue.Queue()
        self._last_mode = None
        self._last_in_error = False

        self._control_sub = self._node.create_subscription(
            String, CONTROL_STATE_TOPIC, self._on_control, 10)
        self._error_sub = self._node.create_subscription(
            String, ERROR_TOPIC, self._on_error, 10)

    def _on_control(self, msg: String) -> None:
        self._queue.put(("control", msg.data))

    def _on_error(self, msg: String) -> None:
        self._queue.put(("error", msg.data))

    def _publish_status(self, mode: str, in_error: bool, error_reason: str) -> None:
        if mode == self._last_mode and in_error == self._last_in_error:
            return
        status = Status()
        status.header.stamp = self._node.get_clock().now().to_msg()
        status.mode = mode
        status.in_error = in_error
        status.error_reason = error_reason if in_error else ""
        self._status_pub.publish(status)
        self._last_mode = mode
        self._last_in_error = in_error

    def wait_for_outcome(self) -> str:
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
                    # 에러 활성: 어느 상태에 있든 STOP으로 강제 전이
                    self._publish_status("STOP", True, data)
                    return "ERROR"
                # 빈 문자열 = 에러 해제 보고. 전이는 없고 다음 이벤트를 계속 기다림.
                self._last_in_error = False
                continue

            try:
                payload = json.loads(data)
                sw_bits = payload["sw_bits"]
            except (json.JSONDecodeError, KeyError, TypeError):
                # control_state 메시지는 sw_bits 없이 다른 키만 담길 수도 있음(app_websocket_bridge
                # 참고) — 파싱 실패/키 없음은 무시하고 현재 상태 유지.
                continue

            mode = SW_BITS_TO_MODE.get(sw_bits)
            if mode is None:
                self._node.get_logger().warning(f"Unknown sw_bits value: {sw_bits!r}")
                continue

            retries = 0
            self._publish_status(mode, False, "")
            return mode


class ControlAndErrorMonitorState(State):
    """StateMachine.add_state()에 등록하는 실제 State. 5개 상태마다 별도
    인스턴스를 만들되(yasmin_viewer가 이름당 고유한 State 객체를 요구함),
    실제 구독/큐잉 로직은 공유 ControlAndErrorMonitor에 위임한다."""

    def __init__(self, monitor: ControlAndErrorMonitor) -> None:
        super().__init__(MODES + ["ERROR", "TIMEOUT"])
        self._monitor = monitor

    def execute(self, blackboard) -> str:
        return self._monitor.wait_for_outcome()


def main(args=None):
    rclpy.init(args=args)

    node = YasminNode.get_instance()
    status_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    status_pub = node.create_publisher(Status, STATUS_TOPIC, status_qos)

    monitor = ControlAndErrorMonitor(node, status_pub)

    # 최상위 상태머신 자체는 정상 운용 중엔 끝나지 않는다(완전연결 루프) —
    # "SHUTDOWN"은 handle_sigint를 위한 형식상의 종료 outcome이며 평상시엔 반환되지 않는다.
    sm = StateMachine(outcomes=["SHUTDOWN"], handle_sigint=True)
    transitions = {mode: mode for mode in MODES}
    transitions["ERROR"] = "STOP"
    transitions["TIMEOUT"] = "STOP"
    for mode in MODES:
        sm.add_state(mode, ControlAndErrorMonitorState(monitor), transitions=transitions)
    sm.set_start_state("STOP")

    # YasminViewerPub(fsm, fsm_name, ...) — 실제 __init__ 시그니처는 fsm이 먼저다
    # (설치된 yasmin_viewer_pub.py의 Args 독스트링 순서와 실제 매개변수 순서가
    # 반대로 되어 있는 라이브러리 쪽 문서 버그이니 착오하지 말 것).
    YasminViewerPub(sm, "JANGAUTO_MISSION")

    try:
        sm()
    finally:
        YasminNode.destroy_instance()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
