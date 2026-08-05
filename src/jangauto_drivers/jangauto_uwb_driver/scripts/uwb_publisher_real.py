#!/usr/bin/env python3
"""실제 UWB 태그(시리얼)를 읽어 절대위치를 발행하는 노드(실기체 전용).

- 이 워크스페이스의 실측위치(localization) 소스는 GPS가 아니라 UWB — 시리얼로
  "Anchor1_x, Anchor1_y, ..., Tag_x, Tag_y" 형식의 한 줄을 받아 마지막 두
  값(태그 x,y)만 뽑아 `/abs_xy`(PoseWithCovarianceStamped)로 발행한다.
- 여기서 내는 covariance는 고정 placeholder — 실제 신뢰도 반영은
  `uwb_cov_filter_real.py`(회전 중엔 IMU만 믿도록 covariance를 동적으로
  덮어씀)가 담당하므로 이 노드는 원시값만 정직하게 낸다.
- `/abs_xy`는 `uwb_cov_filter_real.py`를 거쳐 `/abs_xy_fixed`로
  jangauto_perception의 ekf_global(pose0)에 들어간다.
"""
import queue
import threading

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


class UwbPublisherReal(Node):
    def __init__(self):
        super().__init__('uwb_publisher_real')
        self.absxy_pub = self.create_publisher(PoseWithCovarianceStamped, '/abs_xy', 1)
        self.raw_data_pub = self.create_publisher(String, '/uwb_raw_data', 1)

        self.declare_parameter('port', '/dev/usb-left-bottom')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.data_queue = queue.Queue()
        self.running = True

        try:
            self.ser = serial.Serial(port, baud, timeout=None)  # blocking read
            self.get_logger().info(f'[UWB] Opened serial port: {port} at {baud} baud')
            self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
            self.serial_thread.start()
        except Exception as e:
            self.get_logger().error(f'[UWB] Failed to open serial port {port}: {e}')
            self.ser = None

        # 큐 처리는 메인 스레드에서(10ms 주기) — 시리얼 읽기 스레드와 rclpy 콜백을 분리
        self.timer = self.create_timer(0.01, self.process_queue)

    def serial_reader(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.data_queue.put(line)
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'[UWB] Serial read error: {e}')
                break

    def process_queue(self):
        processed_count = 0
        max_process_per_cycle = 10
        while not self.data_queue.empty() and processed_count < max_process_per_cycle:
            try:
                line = self.data_queue.get_nowait()
                self.process_serial_data(line)
                processed_count += 1
            except queue.Empty:
                break

    def process_serial_data(self, line):
        raw_msg = String()
        raw_msg.data = line
        self.raw_data_pub.publish(raw_msg)

        try:
            parts = line.split(',')
            if len(parts) >= 2:
                x = float(parts[-2].strip())
                y = float(parts[-1].strip())

                p = PoseWithCovarianceStamped()
                p.header.stamp = self.get_clock().now().to_msg()
                p.header.frame_id = 'map'
                p.pose.pose.position.x = x
                p.pose.pose.position.y = y
                p.pose.pose.position.z = 0.0
                p.pose.pose.orientation.w = 1.0

                # X,Y만 신뢰, 나머지는 큰 covariance로 사실상 무시
                p.pose.covariance = [
                    0.5, 0, 0, 0, 0, 0,
                    0, 0.5, 0, 0, 0, 0,
                    0, 0, 1e6, 0, 0, 0,
                    0, 0, 0, 1e6, 0, 0,
                    0, 0, 0, 0, 1e6, 0,
                    0, 0, 0, 0, 0, 1e6,
                ]
                self.absxy_pub.publish(p)
            else:
                self.get_logger().warn(f'[UWB] Not enough data values: {len(parts)} in "{line}"')
        except ValueError as e:
            self.get_logger().warn(f'[UWB] Failed to parse position data: {line}, error: {e}')

    def destroy_node(self):
        self.running = False
        if self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UwbPublisherReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
