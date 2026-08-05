#!/usr/bin/env python3
"""실제 IMU(WT901류 시리얼 프로토콜)를 읽어 sensor_msgs/Imu를 발행하는 노드.

- 시뮬레이션에서는 Gazebo IMU 센서+ros_gz 브릿지가 `/imu`를 발행하므로 이
  노드가 필요 없다 — 실기체 전용이며 대응하는 _simul 파일이 없다
  (jangauto_gazebo 소관).
- `/imu`로 발행한다(기존 `imu_yaw_corrector.py`가 그대로 구독하는 토픽명 —
  레퍼런스 원본(`references/imu.py`)은 `/imu_data`였으나, 현재 아키텍처의
  다운스트림(EKF 등)을 건드리지 않도록 여기서 토픽명을 맞춘다).
- 패킷 파싱/체크섬/단위 변환(가속도 g, 각속도 deg/s, 각도 deg)은 WT901
  펌웨어 프로토콜 그대로 — 이 계층에서 바꿀 이유가 없다.
"""
import math
import time

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Header
from sensor_msgs.msg import Imu

# 단위 변환 상수
G_TO_MS2 = 9.80665
DEG_TO_RAD = math.pi / 180.0
WT_PKT_LEN = 11
ACC_ID, GYRO_ID, ANGLE_ID = 0x51, 0x52, 0x53

# 공분산(레퍼런스 기본값 — 실측 후 재튜닝 필요)
COV_ORIENT = [0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001]
COV_GYRO = [0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001]
COV_ACCEL = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]


def le_i16(lo, hi):
    """2바이트(리틀엔디안) -> signed int16."""
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v


def checksum_ok(pkt):
    return (sum(pkt[:10]) & 0xFF) == pkt[10]


def rpy_deg_to_quat(roll_d, pitch_d, yaw_d):
    r = roll_d * DEG_TO_RAD * 0.5
    p = pitch_d * DEG_TO_RAD * 0.5
    y = yaw_d * DEG_TO_RAD * 0.5
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yy = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, yy, z


class ImuReal(Node):
    def __init__(self):
        super().__init__('imu_real')

        self.declare_parameter('port', '/dev/usb-right-bottom')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('return_rate_hz', 50)
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        return_rate_hz = self.get_parameter('return_rate_hz').get_parameter_value().integer_value

        self.pub = self.create_publisher(Imu, '/imu', 10)

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'[IMU] Opened serial port: {port} at {baudrate} baud, frame_id={self.frame_id}')
        except Exception as e:
            self.get_logger().error(f'[IMU] Failed to open serial port {port}: {e}')
            self.ser = None

        self.buf = bytearray()
        self.acc = [0.0, 0.0, 0.0]
        self.gyr = [0.0, 0.0, 0.0]
        self.euler_deg = [0.0, 0.0, 0.0]
        self.have_acc = self.have_gyro = self.have_ang = False

        if self.ser:
            self.set_return_rate(return_rate_hz)
            self.timer = self.create_timer(0.02, self.poll)  # 50Hz 폴링

    def set_return_rate(self, hz):
        """센서 자체 출력 주파수 설정(WT901 설정 커맨드 시퀀스)."""
        rate_map = {
            1: 0x03, 2: 0x04, 5: 0x05, 10: 0x06, 20: 0x07,
            50: 0x08, 100: 0x09, 125: 0x0A, 200: 0x0B,
            0: 0x0D
        }
        if hz not in rate_map:
            self.get_logger().warn(f'[IMU] Unsupported rate {hz}Hz')
            return

        code = rate_map[hz]

        self.ser.write(bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5]))  # unlock
        time.sleep(0.05)
        self.ser.write(bytes([0xFF, 0xAA, 0x03, code, 0x00]))  # set frequency
        time.sleep(0.05)
        self.ser.write(bytes([0xFF, 0xAA, 0x00, 0x00, 0x00]))  # save
        time.sleep(0.05)

        self.get_logger().info(f'[IMU] Return rate set to {hz} Hz')

    def poll(self):
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                self.buf += data
                self._parse_buffer()
        except Exception as e:
            self.get_logger().error(f'[IMU] read err: {e}')

    def _parse_buffer(self):
        b = self.buf
        while True:
            idx = b.find(b'\x55')  # 패킷 시작 바이트
            if idx < 0:
                if len(b) > 2048:
                    del b[:-1]
                break

            if idx > 0:
                del b[:idx]

            if len(b) < WT_PKT_LEN:
                break

            pkt = bytes(b[:WT_PKT_LEN])
            if not checksum_ok(pkt):
                del b[0:1]
                continue

            del b[:WT_PKT_LEN]
            self._parse_packet(pkt)

            if self.have_acc and self.have_gyro and self.have_ang:
                self.publish()
                self.have_acc = self.have_gyro = self.have_ang = False

    def _parse_packet(self, pkt):
        typ = pkt[1]  # 0x51: 가속도, 0x52: 각속도, 0x53: 각도
        d0, d1, d2, d3, d4, d5, _, _ = pkt[2:10]

        x = le_i16(d0, d1)
        y = le_i16(d2, d3)
        z = le_i16(d4, d5)

        if typ == ACC_ID:
            # (Raw / 32768 * 16g) * 9.8(m/s^2)
            self.acc[0] = (x / 32768 * 16) * G_TO_MS2
            self.acc[1] = (y / 32768 * 16) * G_TO_MS2
            self.acc[2] = (z / 32768 * 16) * G_TO_MS2
            self.have_acc = True
        elif typ == GYRO_ID:
            # (Raw / 32768 * 2000deg/s) * (pi/180)
            self.gyr[0] = (x / 32768 * 2000) * DEG_TO_RAD
            self.gyr[1] = (y / 32768 * 2000) * DEG_TO_RAD
            self.gyr[2] = (z / 32768 * 2000) * DEG_TO_RAD
            self.have_gyro = True
        elif typ == ANGLE_ID:
            # Raw / 32768 * 180 degrees
            self.euler_deg[0] = x / 32768 * 180
            self.euler_deg[1] = y / 32768 * 180
            self.euler_deg[2] = z / 32768 * 180
            self.have_ang = True

    def publish(self):
        now = self.get_clock().now().to_msg()
        msg = Imu()
        msg.header = Header(stamp=now, frame_id=self.frame_id)

        w, x, y, z = rpy_deg_to_quat(*self.euler_deg)
        msg.orientation.w = w
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation_covariance = COV_ORIENT

        msg.angular_velocity.x = self.gyr[0]
        msg.angular_velocity.y = self.gyr[1]
        msg.angular_velocity.z = self.gyr[2]
        msg.angular_velocity_covariance = COV_GYRO

        msg.linear_acceleration.x = self.acc[0]
        msg.linear_acceleration.y = self.acc[1]
        msg.linear_acceleration.z = self.acc[2]
        msg.linear_acceleration_covariance = COV_ACCEL

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
