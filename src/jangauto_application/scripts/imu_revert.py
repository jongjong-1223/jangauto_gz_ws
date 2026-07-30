#!/usr/bin/env python3
"""
IMU Revert Node (Final Fix)
- 역할: 이미 회전되어버린 Accel/Gyro를 '역회전'시켜 원본 Body Frame으로 복구
- Orientation은 건드리지 않음 (Map Frame 정렬 유지)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import math

class ImuRevert(Node):
    def __init__(self):
        super().__init__('imu_revert_node')
        
        # 적용되었던 오프셋 값 (런치 파일에서 1.5987 등으로 설정됨)
        self.declare_parameter('yaw_offset_applied', 0.0)
        self.offset = self.get_parameter('yaw_offset_applied').value
        
        self.sub = self.create_subscription(Imu, '/imu_cal', self.cb, 10)
        self.pub = self.create_publisher(Imu, '/imu_cal_fixed', 10)
        
        self.get_logger().info(f'[IMU_REVERT] Reverting rotation by {-self.offset} rad')

    def cb(self, msg):
        # 1. 역회전 행렬 생성 (Rotation by -offset)
        # 이미 +offset 만큼 돌아가 있으므로, -offset 만큼 돌려야 원상복구됨
        theta = -self.offset
        c, s = math.cos(theta), math.sin(theta)
        
        # 회전 행렬 (2D Z축 회전)
        R_inv = np.array([
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0]
        ])
        
        # 2. 벡터 복구 (Un-rotate)
        # 현재 메시지의 값(이미 회전된 값)을 가져옴
        w_rotated = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        a_rotated = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
        # 역행렬을 곱해서 원본(Body Frame) 값을 계산
        w_fixed = R_inv @ w_rotated
        a_fixed = R_inv @ a_rotated
        
        # 3. 메시지 생성
        out = Imu()
        out.header = msg.header
        
        # [중요 1] Orientation은 그대로 둠 (지도 정렬 유지)
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        
        # [중요 2] 각속도/선가속도는 복구된 값(Body Frame)을 넣음
        out.angular_velocity.x = float(w_fixed[0])
        out.angular_velocity.y = float(w_fixed[1])
        out.angular_velocity.z = float(w_fixed[2])
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        
        out.linear_acceleration.x = float(a_fixed[0])
        out.linear_acceleration.y = float(a_fixed[1])
        out.linear_acceleration.z = float(a_fixed[2])
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ImuRevert()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()