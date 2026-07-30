#!/usr/bin/env python3
"""`/imu` yaw 보정 노드.

## 역할
- CAL(`calibration_action_server.py`)이 계산한 `imu_yaw_offset`만큼 IMU
  orientation을 Z축(yaw)으로 회전시켜 재발행한다 — "주행체가 파워온 때
  잡은 yaw=0"과 "지도(map/GPS) 좌표계의 yaw=0" 사이의 어긋남을, 이후
  로컬라이제이션(`ekf_local`/`ekf_global`)이 쓰는 IMU 값에서 보정하기 위함.
- `orientation`만 회전시키고 `angular_velocity`/`linear_acceleration`은
  원본 그대로 통과시킨다 — 이 값들은 센서 자체 좌표계(body frame) 기준
  측정치라 "기준 좌표계가 뭐냐"의 영향을 안 받는다(회전시키면 오히려
  틀려짐).
- offset은 프로세스 생명주기 동안만 유지되는 상태값 — 재시작하면 0으로
  리셋되며, CAL도 다시 수행해야 하는 기존 설계와 일치한다.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

IMU_IN_TOPIC = 'imu'
IMU_OUT_TOPIC = 'imu_calibrated'
YAW_OFFSET_TOPIC = 'imu_yaw_offset'


def _quat_multiply(q1, q2):
    """해밀턴 곱 q1 ⊗ q2. 인자/반환 모두 (x, y, z, w) 튜플."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _yaw_quat(yaw: float):
    """yaw(rad)만큼 Z축 회전을 나타내는 쿼터니언 (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class ImuYawCorrector(Node):
    """`/imu` 원본을 받아 저장된 yaw offset만큼 회전시켜 `/imu_calibrated`로 재발행."""

    def __init__(self):
        super().__init__('imu_yaw_corrector')

        self._yaw_offset = 0.0

        self._pub = self.create_publisher(Imu, IMU_OUT_TOPIC, 10)
        self.create_subscription(Imu, IMU_IN_TOPIC, self._on_imu, 50)
        self.create_subscription(Float64, YAW_OFFSET_TOPIC, self._on_offset, 10)

    def _on_offset(self, msg: Float64) -> None:
        self._yaw_offset = float(msg.data)
        self.get_logger().info(
            f'[IMU_YAW_CORRECTOR] yaw_offset set to {math.degrees(self._yaw_offset):.2f} deg')

    def _on_imu(self, msg: Imu) -> None:
        if abs(self._yaw_offset) < 1e-12:
            self._pub.publish(msg)
            return

        q_raw = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        x, y, z, w = _quat_multiply(_yaw_quat(self._yaw_offset), q_raw)

        out = Imu()
        out.header = msg.header
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = x, y, z, w
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._pub.publish(out)


def main():
    """노드 진입점 — `rclpy.spin()`으로 상주하며 매 IMU 메시지를 보정해 재발행한다."""
    rclpy.init()
    node = ImuYawCorrector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
