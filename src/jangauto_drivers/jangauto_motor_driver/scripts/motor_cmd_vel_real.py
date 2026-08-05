#!/usr/bin/env python3
"""cmd_vel_out을 실제 모터 컨트롤러로 전달하는 시리얼 트랜시버 노드.

- 시뮬레이션에서는 Gazebo diff_drive 플러그인이 cmd_vel_out을 직접 구독해
  물리 시뮬레이션을 굴리므로 이 노드가 필요 없다 — 이 노드는 실기체 전용이며
  대응하는 _simul 파일이 없다(jangauto_gazebo 소관).
- cmd_vel(선속도 v, 각속도 w) -> 좌/우 트랙 목표 RPM 변환 시 **각속도 우선
  클리핑**을 쓴다: 회전(w)을 먼저 최대 각속도로 클리핑하고, 남은 RPM 여유를
  선속도(v)에 배분한다 — 코너링/제자리회전 시 목표 조향각을 절대 깎지 않기
  위함(references/motor_cmd_vel_real_linear.py의 "선속도 우선" 로직을 그대로
  미러링하되 v/w 역할만 뒤바꿈).
- `/robot_status`(jangauto_msg/Status).current_state가 KEY/RUN/CAL/ALIGN
  중 하나가 아니면(=STOP 등) 모터 명령을 0으로 강제 — cmd_vel_arbiter가
  이미 모드별로 소스를 걸러주지만, 시리얼 링크 이중 안전장치로 유지.
- 시리얼 프로토콜(패킷 struct, 응답 송신 등)은 레퍼런스 코드 그대로 포팅 —
  모터 컨트롤러 펌웨어 쪽 규격이라 이 계층에서 바꿀 이유가 없다.
"""
import math
import struct
import threading

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from jangauto_msg.msg import Status

DRIVING_STATES = {"KEY", "RUN", "CAL", "ALIGN"}


class MotorCmdVelReal(Node):
    def __init__(self):
        super().__init__('motor_cmd_vel_trx')

        # Serial Port Parameters
        self.declare_parameter('port', '/dev/usb-left-top')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Kinematics Parameters — 실측 후 재튜닝 필요(레퍼런스 기본값 그대로)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 1.5)
        self.declare_parameter('gear_ratio', 60.0)
        self.declare_parameter('max_motor_rpm', 3000.0)
        self.declare_parameter('max_linear_velocity', 0.35)
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value
        self.max_motor_rpm = self.get_parameter('max_motor_rpm').get_parameter_value().double_value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').get_parameter_value().double_value

        self.wheel_circumference = 2.0 * math.pi * self.wheel_radius
        self.max_angular_velocity = (2.0 * self.max_linear_velocity) / self.wheel_base

        # Serial Port Open
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            self.get_logger().info(
                f'[MOTOR_ANGULAR] Opened serial port: {port} at {baudrate} baud\n'
                f'  Strategy: Angular Priority'
            )
        except Exception as e:
            self.get_logger().error(f'[MOTOR_ANGULAR] Failed to open serial port {port}: {e}')
            self.ser = None

        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_out', self.cmd_vel_callback, 10)
        self.status_sub = self.create_subscription(Status, '/robot_status', self.robot_status_callback, 10)

        # Publishers — 수신 RPM은 참고용 발행만(현재 구독자 없음, 디버깅용)
        self.motor_rpm_pub = self.create_publisher(Twist, '/motor_rpm', 10)

        # State Variables
        self.current_state = "STOP"
        self.motor_command = [0, 0]  # [rpm_left, rpm_right]
        self.received_motor_data = [0, 0]

        self.prev_motor_command = None
        self.prev_v_input = None
        self.prev_w_input = None
        self.prev_received_rpm = None
        self.prev_sent_rpm = None

        self.running = True
        if self.ser:
            self.serial_thread = threading.Thread(target=self.serial_handler, daemon=True)
            self.serial_thread.start()

        self.get_logger().info(
            f'[MOTOR_ANGULAR] Motor driver started\n'
            f'  - Wheel Radius: {self.wheel_radius} m\n'
            f'  - Wheel Base: {self.wheel_base} m\n'
            f'  - Gear Ratio: {self.gear_ratio}\n'
            f'  - Max Motor RPM: {self.max_motor_rpm}\n'
            f'  - Max Linear Velocity: {self.max_linear_velocity:.3f} m/s\n'
            f'  - Max Angular Velocity: {self.max_angular_velocity:.3f} rad/s'
        )

    def serial_handler(self):
        """시리얼 수신-송신 핸들러 — 모터 컨트롤러가 4바이트(<hh>) RPM 피드백을 보내면
        곧바로 현재 목표 RPM을 응답으로 송신하는 요청-응답 프로토콜."""
        while self.running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(4)
                if len(data) == 4:
                    motor1_rpm, motor2_rpm = struct.unpack('<hh', data)
                    self.received_motor_data = [motor1_rpm, motor2_rpm]

                    rpm_msg = Twist()
                    rpm_msg.angular.x = float(motor1_rpm)  # Left RPM
                    rpm_msg.angular.y = float(motor2_rpm)  # Right RPM
                    self.motor_rpm_pub.publish(rpm_msg)

                    if self.prev_received_rpm != self.received_motor_data:
                        self.get_logger().info(
                            f'[MOTOR_ANGULAR] Received: M1={motor1_rpm} rpm, M2={motor2_rpm} rpm'
                        )
                        self.prev_received_rpm = self.received_motor_data.copy()

                    self.send_motor_response()
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'[MOTOR_ANGULAR] Serial communication error: {e}')
                break

    def send_motor_response(self):
        try:
            response_data = struct.pack('<hh', int(self.motor_command[0]), int(self.motor_command[1]))
            self.ser.write(response_data)

            if self.prev_sent_rpm != self.motor_command:
                self.get_logger().info(
                    f'[MOTOR_ANGULAR] Sent: M1={self.motor_command[0]} rpm, M2={self.motor_command[1]} rpm'
                )
                self.prev_sent_rpm = self.motor_command.copy()
        except Exception as e:
            self.get_logger().error(f'[MOTOR_ANGULAR] Serial write error: {e}')

    def robot_status_callback(self, msg: Status):
        """/robot_status.current_state를 추적 — STOP 등 주행 상태가 아니면 즉시 정지."""
        self.current_state = msg.current_state

        if self.current_state not in DRIVING_STATES:
            self.motor_command = [0, 0]

    def cmd_vel_callback(self, msg: Twist):
        """cmd_vel -> RPM 변환(각속도 우선 클리핑).

        전략: 각속도(w)를 먼저 최대치로 클리핑하고, 남은 RPM 여유를 선속도(v)에
        배분한다 — 회전 중에는 목표 조향각을 절대 깎지 않기 위함.
        """
        if self.current_state not in DRIVING_STATES:
            self.motor_command = [0, 0]
            return

        v_cmd = msg.linear.x  # m/s
        w_cmd = msg.angular.z  # rad/s

        if self.wheel_circumference == 0:
            self.get_logger().error('[MOTOR_ANGULAR] Wheel circumference is zero!')
            return

        # Angular Velocity Clipping (우선)
        if abs(w_cmd) > self.max_angular_velocity:
            w_final = math.copysign(self.max_angular_velocity, w_cmd)
            self.get_logger().warn(
                f'[MOTOR_ANGULAR] ANGULAR CLIPPED\n'
                f'  Input w: {w_cmd:.3f} rad/s\n'
                f'  Output w: {w_final:.3f} rad/s (Max: {self.max_angular_velocity:.3f})',
                throttle_duration_sec=1.0
            )
        else:
            w_final = w_cmd

        # Linear Velocity Damping — 각속도가 쓰고 남은 RPM 여유만 선속도에 배분
        wheel_velocity_for_w = abs(w_final) * self.wheel_base * 0.5
        max_wheel_diff = self.max_linear_velocity - wheel_velocity_for_w

        if max_wheel_diff < 0:
            max_wheel_diff = 0.0

        max_v_allowed = max_wheel_diff

        if abs(v_cmd) > max_v_allowed:
            v_final = math.copysign(max_v_allowed, v_cmd)

            damping_ratio = (max_v_allowed / abs(v_cmd)) * 100 if v_cmd != 0 else 100

            self.get_logger().warn(
                f'[MOTOR_ANGULAR] LINEAR DAMPED\n'
                f'  Input: v={v_cmd:.3f}, w={w_final:.3f}\n'
                f'  Max v allowed: {max_v_allowed:.3f} m/s (after w reservation)\n'
                f'  Damping ratio: {damping_ratio:.1f}%\n'
                f'  Output: v={v_final:.3f}, w={w_final:.3f}',
                throttle_duration_sec=1.0
            )
        else:
            v_final = v_cmd

        # Convert to Wheel Velocities and RPM
        v_left = v_final - (w_final * self.wheel_base * 0.5)
        v_right = v_final + (w_final * self.wheel_base * 0.5)

        rpm_left = (v_left * 60.0 * self.gear_ratio) / self.wheel_circumference
        rpm_right = (v_right * 60.0 * self.gear_ratio) / self.wheel_circumference

        # Safety Verification
        if abs(rpm_left) > self.max_motor_rpm or abs(rpm_right) > self.max_motor_rpm:
            self.get_logger().error(
                f'[MOTOR_ANGULAR] RPM EXCEEDED (LOGIC ERROR)\n'
                f'  Target RPM: L={rpm_left:.0f}, R={rpm_right:.0f}\n'
                f'  Max RPM: {self.max_motor_rpm:.0f}\n'
                f'  Emergency Stop Activated'
            )
            rpm_left = 0.0
            rpm_right = 0.0

        self.motor_command = [int(rpm_left), int(rpm_right)]

        if (self.prev_motor_command != self.motor_command or
                self.prev_v_input != v_cmd or self.prev_w_input != w_cmd):

            v_left_target = v_cmd - (w_cmd * self.wheel_base * 0.5)
            v_right_target = v_cmd + (w_cmd * self.wheel_base * 0.5)
            rpm_left_target = (v_left_target * 60.0 * self.gear_ratio) / self.wheel_circumference
            rpm_right_target = (v_right_target * 60.0 * self.gear_ratio) / self.wheel_circumference

            self.get_logger().info(
                f'[MOTOR_ANGULAR] CmdVel (v={v_cmd:.2f}, w={w_cmd:.2f}) -> '
                f'Target RPM (L={rpm_left_target:.0f}, R={rpm_right_target:.0f}) -> '
                f'Actual RPM (L={rpm_left:.0f}, R={rpm_right:.0f})'
            )
            self.prev_motor_command = self.motor_command.copy()
            self.prev_v_input = v_cmd
            self.prev_w_input = w_cmd

    def destroy_node(self):
        self.get_logger().info('[MOTOR_ANGULAR] Shutting down...')
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                stop_data = struct.pack('<hh', 0, 0)
                self.ser.write(stop_data)
                self.get_logger().info('[MOTOR_ANGULAR] Sent final stop command')
            except Exception:
                pass
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdVelReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
