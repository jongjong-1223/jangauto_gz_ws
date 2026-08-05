#!/usr/bin/env python3
"""UWB 위치의 covariance를 회전 상태에 따라 동적으로 덮어쓰는 필터(실기체 전용).

- 평소엔 UWB를 적당히 신뢰(covariance=normal_cov)하지만, 회전 중(각속도가
  임계값 이상)에는 UWB 측위가 흔들리는 경향이 있어 covariance를 크게 키워
  (rotate_cov) 사실상 무시하고 EKF가 IMU 회전만 믿게 만든다.
- 입력 각속도는 `/imu_calibrated`(jangauto_perception/imu_yaw_corrector.py가
  CAL yaw offset을 반영해 재발행하는 보정된 IMU)에서 가져온다 — 레퍼런스
  원본(`references/uwb_dynamic_cov.py`)은 레거시 파이프라인의
  `/imu_cal_fixed`를 구독했으나, 현재 아키텍처의 동일 역할 토픽으로 리매핑.
- `/abs_xy`(uwb_publisher_real.py) -> (이 노드) -> `/abs_xy_fixed`
  (jangauto_perception ekf_global의 pose0 입력).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class UwbCovFilterReal(Node):
    def __init__(self):
        super().__init__('uwb_cov_filter_real')

        self.declare_parameter('normal_cov', 2.0)
        self.declare_parameter('rotate_cov', 999.0)
        self.declare_parameter('yaw_rate_thresh', 0.05)

        self.normal_cov = self.get_parameter('normal_cov').value
        self.rotate_cov = self.get_parameter('rotate_cov').value
        self.threshold = self.get_parameter('yaw_rate_thresh').value

        self.current_yaw_rate = 0.0

        self.create_subscription(Imu, '/imu_calibrated', self.imu_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/abs_xy', self.uwb_cb, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/abs_xy_fixed', 10)

        self.get_logger().info(f'[UWB_COV] Started. Rotation threshold: {self.threshold} rad/s')

    def imu_cb(self, msg):
        self.current_yaw_rate = abs(msg.angular_velocity.z)

    def uwb_cb(self, msg):
        out = msg
        new_cov = list(msg.pose.covariance)

        target_cov = self.rotate_cov if self.current_yaw_rate > self.threshold else self.normal_cov

        new_cov[0] = target_cov  # X
        new_cov[7] = target_cov  # Y

        out.pose.covariance = new_cov
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = UwbCovFilterReal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
