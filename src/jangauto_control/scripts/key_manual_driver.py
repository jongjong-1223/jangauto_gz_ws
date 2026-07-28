#!/usr/bin/env python3
"""KEY 모드 수동조종(조이스틱) 변환 노드.

## 역할
- `/app/control_state`(앱이 500ms마다 보내는 JSON, `app_websocket_bridge.py`가
  재발행)에서 `key_bits`(조이스틱 방향)/`speed_bits`(속도 단계)만 뽑아서
  `cmd_vel_manual`(Twist)로 변환해 발행한다.
- 모드 판단은 안 한다 — `cmd_vel_arbiter.py`의 `MODE_TO_SOURCE_TOPICS`가 KEY
  모드일 때만 `cmd_vel_manual`을 실제로 통과시키므로, 이 노드는 변환만 하면 된다.
- **조이스틱을 안 건드릴 때는 `cmd_vel_manual`을 아예 발행하지 않는다** — 계속
  발행하면 이 토픽이 영원히 "최근"으로 남아 `cmd_vel_arbiter`가 MoveRequest의
  Nav2 출력(`cmd_vel_nav_out`)을 절대 통과시키지 못하게 된다. 조작 중일 때만
  주기 발행하고, 놓는 순간 정지 Twist를 1회만 보낸 뒤 조용해진다.
- `/app/control_state` 수신이 오래 끊기면(앱 연결 끊김 등) 마지막 방향과
  무관하게 정지 Twist를 1회 발행한다 — `mission_state_machine`의 5초 명령
  타임아웃(모드 레벨)보다 훨씬 빠르게 반응하는 자체 안전장치.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

CONTROL_STATE_TOPIC = '/app/control_state'
CMD_VEL_MANUAL_TOPIC = 'cmd_vel_manual'

# key_bits — 앱 JoystickView/keyToDesc와 동일한 one-hot 매핑(단일 방향만 유효).
KEY_BIT_FRONT = 0b1000
KEY_BIT_BACK = 0b0100
KEY_BIT_LEFT = 0b0010
KEY_BIT_RIGHT = 0b0001

# speed_bits — 앱은 지금 NORMAL(0b010) 고정값만 보내지만, 향후 속도 UI가
# 생겨도 바로 대응할 수 있도록 one-hot 3단계 매핑을 미리 구현해둔다.
SPEED_BIT_LOW = 0b001
SPEED_BIT_NORMAL = 0b010
SPEED_BIT_HIGH = 0b100


class KeyManualDriver(Node):
    """`/app/control_state`의 key_bits/speed_bits -> `cmd_vel_manual` 변환기."""

    def __init__(self):
        super().__init__('key_manual_driver')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('control_state_stale_timeout_sec', 1.0)
        self.declare_parameter('speed_low_linear', 0.25)
        self.declare_parameter('speed_low_angular', 0.3)
        self.declare_parameter('speed_normal_linear', 0.5)
        self.declare_parameter('speed_normal_angular', 0.6)
        self.declare_parameter('speed_high_linear', 0.75)
        self.declare_parameter('speed_high_angular', 0.9)

        self._publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._control_state_stale_timeout_sec = self.get_parameter(
            'control_state_stale_timeout_sec').get_parameter_value().double_value
        self._speed_low = (
            self.get_parameter('speed_low_linear').get_parameter_value().double_value,
            self.get_parameter('speed_low_angular').get_parameter_value().double_value,
        )
        self._speed_normal = (
            self.get_parameter('speed_normal_linear').get_parameter_value().double_value,
            self.get_parameter('speed_normal_angular').get_parameter_value().double_value,
        )
        self._speed_high = (
            self.get_parameter('speed_high_linear').get_parameter_value().double_value,
            self.get_parameter('speed_high_angular').get_parameter_value().double_value,
        )

        self._pub = self.create_publisher(Twist, CMD_VEL_MANUAL_TOPIC, 10)
        self.create_subscription(String, CONTROL_STATE_TOPIC, self._on_control_state, 10)

        # 최신 key_bits/speed_bits와 마지막 수신 시각 — 실제 발행 여부/내용은
        # _tick()이 주기적으로 판단한다.
        self._last_control_state_monotonic = None
        self._last_key_bits = 0
        self._last_speed_bits = SPEED_BIT_NORMAL
        # 직전 틱에 0이 아닌 값을 발행했는지 — 0으로 떨어지는 순간 정지를 1회만
        # 보내기 위한 엣지 감지용.
        self._prev_published_nonzero = False

        self.create_timer(1.0 / self._publish_rate_hz, self._tick)

    def _on_control_state(self, msg: String) -> None:
        """`/app/control_state` 구독 콜백 — key_bits/speed_bits만 뽑아 캐싱.
        판단·발행은 하지 않는다(그건 _tick()의 몫)."""
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self._last_control_state_monotonic = time.monotonic()
        self._last_key_bits = data.get('key_bits', 0)
        self._last_speed_bits = data.get('speed_bits', SPEED_BIT_NORMAL)

    def _speed_for(self, speed_bits) -> tuple:
        if speed_bits == SPEED_BIT_LOW:
            return self._speed_low
        if speed_bits == SPEED_BIT_HIGH:
            return self._speed_high
        return self._speed_normal  # NORMAL 또는 알 수 없는 값 -> 안전한 폴백

    def _twist_for(self, key_bits, linear: float, angular: float) -> Twist:
        """one-hot 단일 방향만 유효 — 0이거나 다중 비트/알 수 없는 값이면
        정지(Twist() 기본값)를 그대로 리턴한다."""
        twist = Twist()
        if key_bits == KEY_BIT_FRONT:
            twist.linear.x = linear
        elif key_bits == KEY_BIT_BACK:
            twist.linear.x = -linear
        elif key_bits == KEY_BIT_LEFT:
            twist.angular.z = angular
        elif key_bits == KEY_BIT_RIGHT:
            twist.angular.z = -angular
        return twist

    def _tick(self) -> None:
        """주기 타이머 — 조작 중일 때만 계속 발행, 놓거나(release) 끊기면(stale)
        정지를 1회만 발행하고 그 다음부턴 조용해진다."""
        now = time.monotonic()
        stale = (
            self._last_control_state_monotonic is None
            or (now - self._last_control_state_monotonic) > self._control_state_stale_timeout_sec
        )
        key_bits = 0 if stale else self._last_key_bits

        if key_bits == 0:
            if self._prev_published_nonzero:
                self._pub.publish(Twist())
                self._prev_published_nonzero = False
            return

        linear, angular = self._speed_for(self._last_speed_bits)
        self._pub.publish(self._twist_for(key_bits, linear, angular))
        self._prev_published_nonzero = True


def main(args=None):
    """노드 진입점 — `rclpy.spin()`으로 상주하며 주기 타이머를 계속 처리한다."""
    rclpy.init(args=args)
    node = KeyManualDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
