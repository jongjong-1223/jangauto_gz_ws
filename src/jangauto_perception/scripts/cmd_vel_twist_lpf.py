#!/usr/bin/env python3
"""`cmd_vel_out` LPF 필터 노드.

## 역할
- 실제 바퀴 엔코더/UWB 같은 속도 센서가 없어서, `ekf_local`(IMU만 융합)이
  선속도(Vx)를 아예 측정하지 못하고 IMU 가속도 적분으로만 추정한다.
- `cmd_vel_arbiter.py`가 최종 발행하는 `cmd_vel_out`("로봇에 이 속도로
  가라고 시킨 값")을 1차 저역통과 필터(LPF)로 다듬어, 액추에이터 반응
  지연을 감안한 "실제로 이 정도 속도로 움직였을 것"이라는 pseudo-measurement로
  만든다.
- `ekf_node`의 `twistN` 입력은 `geometry_msgs/TwistWithCovarianceStamped`
  타입이어야 하므로(공분산 없는 `Twist`는 직접 못 물림), 필터링한 값에
  공분산을 채워 `cmd_vel_twist_filtered`로 재발행한다 — `ekf.yaml`의
  `ekf_local.twist0`가 이 토픽을 구독.
- `linear.x`/`angular.z`는 `cmd_vel_out`을 필터링한 값을 그대로 쓴다.
  `angular.z` 공분산은 일부러 크게 잡아서(신뢰 낮게) IMU 자이로가 yaw rate
  추정을 계속 주도하게 하고, `linear.x`는 유일한 소스라 중간 수준으로 신뢰한다.
- `linear.y`는 필터링 대상이 아니라 **비홀로노믹(nonholonomic) 제약**이다 —
  트랙 구동 로봇은 몸체 좌표계에서 옆으로 미끄러지지 않는다고 가정하고 항상
  0을 발행한다(`cmd_vel_out.linear.y`는 애초에 항상 0이라 필터링할 의미가
  없음). 급회전 시 실제로는 어느 정도 스크럽(옆미끄러짐)이 있는 트랙형
  차체라, `linear_x_stddev`보다 느슨한(신뢰 낮은) 공분산으로 부드러운 제약만
  건다.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped

import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

CMD_VEL_IN_TOPIC = 'cmd_vel_out'
TWIST_OUT_TOPIC = 'cmd_vel_twist_filtered'
BASE_FRAME_ID = 'base_link'   # ekf.yaml의 base_link_frame과 일치해야 함

# cmd_vel_out이 이 시간(초) 이상 안 오면 WARN(끊김)으로 판정
STALE_TIMEOUT_SEC = 1.0


class CmdVelTwistLpf(Node):
    """`cmd_vel_out` -> (LPF + 공분산 부착) -> `cmd_vel_twist_filtered` 재발행."""

    def __init__(self):
        super().__init__('cmd_vel_twist_lpf')

        self.declare_parameter('cutoff_freq_hz', 2.0)
        self.declare_parameter('linear_x_stddev', 0.1)
        self.declare_parameter('linear_y_stddev', 0.2)
        self.declare_parameter('angular_z_stddev', 1.0)

        cutoff_freq_hz = self.get_parameter('cutoff_freq_hz').value
        # tau = 1/(2*pi*fc) — 매 콜백마다 실제 dt로 alpha를 다시 계산하므로
        # 여기서는 시상수만 한 번 구해둔다.
        self._tau = 1.0 / (2.0 * math.pi * cutoff_freq_hz)

        linear_x_stddev = self.get_parameter('linear_x_stddev').value
        linear_y_stddev = self.get_parameter('linear_y_stddev').value
        angular_z_stddev = self.get_parameter('angular_z_stddev').value
        self._linear_x_variance = linear_x_stddev * linear_x_stddev
        # linear.y는 필터링하지 않는 상수 0 제약이라 분산도 미리 한 번만 계산.
        self._linear_y_variance = linear_y_stddev * linear_y_stddev
        self._angular_z_variance = angular_z_stddev * angular_z_stddev

        # LPF 내부 상태 — 첫 메시지가 올 때까지 None(그 값으로 그대로 초기화).
        self._filtered_linear_x = None
        self._filtered_angular_z = None
        self._last_msg_monotonic = None

        self._pub = self.create_publisher(TwistWithCovarianceStamped, TWIST_OUT_TOPIC, 10)
        self.create_subscription(Twist, CMD_VEL_IN_TOPIC, self._on_cmd_vel, 10)

        self._diag_updater = diagnostic_updater.Updater(self)
        self._diag_updater.setHardwareID('cmd_vel_twist_lpf')
        self._diag_updater.add('cmd_vel_out reception', self._diagnostics_callback)

    def _on_cmd_vel(self, msg: Twist) -> None:
        """`cmd_vel_out` 구독 콜백 — 실제 경과시간으로 1차 IIR LPF를 적용한 뒤
        공분산을 채워 바로 재발행한다(타이머 없이 메시지 도착 시점마다 처리)."""
        now = time.monotonic()
        if self._filtered_linear_x is None:
            # 첫 메시지 — 필터를 현재값으로 그대로 초기화(0에서 서서히 올라오는
            # 워밍업 지연을 피함).
            self._filtered_linear_x = msg.linear.x
            self._filtered_angular_z = msg.angular.z
        else:
            dt = now - self._last_msg_monotonic
            alpha = dt / (self._tau + dt)
            self._filtered_linear_x += alpha * (msg.linear.x - self._filtered_linear_x)
            self._filtered_angular_z += alpha * (msg.angular.z - self._filtered_angular_z)
        self._last_msg_monotonic = now

        out = TwistWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = BASE_FRAME_ID
        out.twist.twist.linear.x = self._filtered_linear_x
        out.twist.twist.linear.y = 0.0  # 비홀로노믹 제약(항상 옆미끄러짐 없음으로 가정)
        out.twist.twist.angular.z = self._filtered_angular_z
        # 6x6 행-우선(row-major) 공분산 — linear.x(인덱스 0)/linear.y(인덱스 7)/
        # angular.z(인덱스 35)만 채우고, 나머지 축은 twist0_config에서 아예
        # 미사용 처리하므로 0으로 둔다.
        out.twist.covariance[0] = self._linear_x_variance
        out.twist.covariance[7] = self._linear_y_variance
        out.twist.covariance[35] = self._angular_z_variance
        self._pub.publish(out)

    def _diagnostics_callback(self, stat):
        """`gps_covariance_filler.py`와 동일한 최근성 판정 패턴."""
        if self._last_msg_monotonic is None:
            stat.summary(DiagnosticStatus.ERROR, 'No cmd_vel_out received yet')
        elif (time.monotonic() - self._last_msg_monotonic) > STALE_TIMEOUT_SEC:
            stat.summary(DiagnosticStatus.WARN, 'cmd_vel_out is stale')
        else:
            stat.summary(DiagnosticStatus.OK, 'Receiving cmd_vel_out')
        return stat


def main():
    """노드 진입점 — `rclpy.spin()`으로 상주하며 `cmd_vel_out`을 계속 필터링한다."""
    rclpy.init()
    node = CmdVelTwistLpf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
