#!/usr/bin/env python3
"""GPS covariance filler 노드.

## 역할
- `navsat`(NavSatFix)을 구독해서, 고정 `position_covariance`를 채운 뒤
  `navsat_fixed`로 그대로 재발행한다(그 외 필드는 원본 그대로).
- 시뮬레이션 navsat 센서(gz-sensors)는 노이즈를 설정해도
  `position_covariance`를 채워주지 않는다 — 값이 계속 0.0으로 오면
  하류(`navsat_transform_node` → `ekf_global`)가 "GPS가 완벽하게
  확실하다"고 착각해서 EKF 융합 가중치가 잘못 잡힌다. 이 노드가 그 빈
  값을 채워서 실제 GPS 신뢰도를 반영하게 해준다.
- `diagnostic_updater`로 GPS 수신 최근성을 `/diagnostics`에 보고한다.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

# GPS 데이터가 이 시간(초) 이상 안 오면 WARN(끊김)으로 판정
STALE_TIMEOUT_SEC = 3.0


class GpsCovarianceFiller(Node):
    """`navsat` -> (covariance 채움) -> `navsat_fixed` 재발행 + 수신 진단."""

    def __init__(self):
        super().__init__('gps_covariance_filler')
        # position_stddev(표준편차, m)를 파라미터로 받아 분산(공분산 대각 성분)으로
        # 변환 — 대각 행렬만 쓰므로 x/y/z 세 축 모두 같은 값을 재사용한다.
        self.declare_parameter('position_stddev', 0.3)
        stddev = self.get_parameter('position_stddev').value
        variance = stddev * stddev
        self._covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance,
        ]
        self._pub = self.create_publisher(NavSatFix, 'navsat_fixed', 10)
        self._sub = self.create_subscription(
            NavSatFix, 'navsat', self._callback, 10)

        # 진단 콜백이 "최근에 데이터가 왔는가"를 판단하는 데 쓰는 마지막 수신 시각.
        self._last_msg_monotonic = None

        self._diag_updater = diagnostic_updater.Updater(self)
        self._diag_updater.setHardwareID('gps_covariance_filler')
        self._diag_updater.add('GPS reception', self._diagnostics_callback)

    def _callback(self, msg: NavSatFix) -> None:
        """`navsat` 구독 콜백 — covariance 필드만 고정값으로 덮어써서 그대로 재발행."""
        self._last_msg_monotonic = time.monotonic()
        msg.position_covariance = self._covariance
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._pub.publish(msg)

    def _diagnostics_callback(self, stat):
        """`diagnostic_updater`가 주기적으로 호출 — 수신 이력 유무/최근성으로
        OK(정상 수신) / WARN(끊김, STALE_TIMEOUT_SEC 초과) / ERROR(수신 이력 없음)를 판정."""
        if self._last_msg_monotonic is None:
            stat.summary(DiagnosticStatus.ERROR, 'No GPS data received yet')
        elif (time.monotonic() - self._last_msg_monotonic) > STALE_TIMEOUT_SEC:
            stat.summary(DiagnosticStatus.WARN, 'GPS data is stale')
        else:
            stat.summary(DiagnosticStatus.OK, 'Receiving GPS data')
        return stat


def main():
    """노드 진입점 — `rclpy.spin()`으로 상주하며 `navsat` 메시지를 계속 처리한다."""
    rclpy.init()
    node = GpsCovarianceFiller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
