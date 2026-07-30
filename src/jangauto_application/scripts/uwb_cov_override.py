#!/usr/bin/env python3
"""
UWB Filter & Covariance Override Node (Merged)
- 기능 1: 이동 평균 필터 (MA Filter)로 튀는 값 제거
- 기능 2: 공분산(Covariance) 값을 설정값으로 강제 변경
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from collections import deque

class UwbFilterAndCov(Node):
    def __init__(self):
        super().__init__('uwb_filter_and_cov_node')
        
        # [파라미터 1] MA 필터 윈도우 크기 (기본값: 4)
        self.declare_parameter('window_size', 4)
        self.window_size = self.get_parameter('window_size').value
        
        # [파라미터 2] 덮어씌울 공분산 값 (기본값: 2.0)
        self.declare_parameter('uwb_covariance', 2.0)
        self.target_cov = self.get_parameter('uwb_covariance').value
        
        # 이동 평균용 큐 (FIFO)
        self.x_queue = deque(maxlen=self.window_size)
        self.y_queue = deque(maxlen=self.window_size)
        
        # 구독: 원본 UWB (/abs_xy)
        self.sub = self.create_subscription(PoseWithCovarianceStamped, '/abs_xy', self.cb, 10)
        
        # 발행: 필터링 + 공분산 수정된 UWB (/abs_xy_fixed)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/abs_xy_fixed', 10)
        
        self.get_logger().info(f'[UWB Merged] Window: {self.window_size}, Covariance: {self.target_cov}')

    def cb(self, msg):
        # ---------------------------------------------------------
        # 1. 이동 평균 필터 (Moving Average) 적용
        # ---------------------------------------------------------
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        self.x_queue.append(curr_x)
        self.y_queue.append(curr_y)
        
        # 평균 계산
        avg_x = sum(self.x_queue) / len(self.x_queue)
        avg_y = sum(self.y_queue) / len(self.y_queue)
        
        # ---------------------------------------------------------
        # 2. 메시지 수정 (위치 & 공분산)
        # ---------------------------------------------------------
        out = msg
        
        # (1) 위치를 필터링된 값으로 교체
        out.pose.pose.position.x = avg_x
        out.pose.pose.position.y = avg_y
        
        # (2) 공분산을 설정값으로 덮어쓰기
        new_cov = list(msg.pose.covariance)
        new_cov[0] = float(self.target_cov) # Pxx
        new_cov[7] = float(self.target_cov) # Pyy
        out.pose.covariance = new_cov
        
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = UwbFilterAndCov()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()