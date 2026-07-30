#!/usr/bin/env python3
"""
UWB Dynamic Covariance Node (Smart Filter)
- 평소: R = 2.0 (적당히 신뢰)
- 회전 시: R = 999.0 (완전 무시 -> IMU 회전만 믿음)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import math

class UwbDynamicCov(Node):
    def __init__(self):
        super().__init__('uwb_dynamic_cov_node')
        
        # 파라미터 설정
        self.declare_parameter('normal_cov', 2.0)   # 평소 공분산
        self.declare_parameter('rotate_cov', 999.0) # 회전 시 공분산 (무시)
        self.declare_parameter('yaw_rate_thresh', 0.05) # 회전 감지 임계값 (rad/s)
        
        self.normal_cov = self.get_parameter('normal_cov').value
        self.rotate_cov = self.get_parameter('rotate_cov').value
        self.threshold = self.get_parameter('yaw_rate_thresh').value
        
        self.current_yaw_rate = 0.0
        
        # 구독
        self.create_subscription(Imu, '/imu_cal_fixed', self.imu_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/abs_xy', self.uwb_cb, 10)
        
        # 발행 (EKF로 들어갈 수정된 UWB)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/abs_xy_fixed', 10)
        
        self.get_logger().info(f'[Smart UWB] Started. Rotation Threshold: {self.threshold} rad/s')

    def imu_cb(self, msg):
        # 현재 회전 속도(절대값) 저장
        self.current_yaw_rate = abs(msg.angular_velocity.z)

    def uwb_cb(self, msg):
        out = msg
        new_cov = list(msg.pose.covariance)
        
        # 회전 중인지 판단
        if self.current_yaw_rate > self.threshold:
            target_cov = self.rotate_cov
            # self.get_logger().info(f'Rotating! (Rate: {self.current_yaw_rate:.2f}) -> Ignore UWB', throttle_duration_sec=1.0)
        else:
            target_cov = self.normal_cov
            
        # 공분산 덮어쓰기 (X, Y)
        new_cov[0] = target_cov
        new_cov[7] = target_cov
        
        out.pose.covariance = new_cov
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = UwbDynamicCov()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()