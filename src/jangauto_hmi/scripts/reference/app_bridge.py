#!/usr/bin/env python3
"""앱 브릿지: 앱이 보내는 비트 문자열을 로봇 명령으로 변환.

## 역할
- (참고용 미사용 코드) 예전 HTTP 기반 앱 프로토콜에서 쓰던 노드다. 현재는
  `app_websocket_bridge.py`(웹소켓 기반 신규 프로토콜)가 이 역할을 대신하며,
  이 파일은 빌드/실행 대상이 아니다.
- `/app/sw_bits`(모드 전환), `/app/key_bits`(KEY 모드 방향키),
  `/app/speed_bits`(KEY 모드 속도 단계), `/app/video_bit`(영상 화면 on/off),
  `/app/safe_bit`(안전 버튼) 5개 토픽을 구독해 각각 로봇이 이해하는
  명령(`/state_command`, `/cmd_vel`, `/video_enable`, `/safety`)으로 변환/발행한다.
- `/robot_state`를 구독해 현재 로봇 상태를 추적하고, 상태에 따라 명령
  처리 여부를 가른다(예: 이동 명령은 KEY 상태에서만 허용).
- 모든 비트 필드는 앱이 보낸 "0"/"1"로 이루어진 원-핫(one-hot) 문자열이다
  (mission_state_machine.py가 쓰는 정수 비트값 프로토콜과는 다른 예전 방식).

## 클래스 구성
- `AppBridge`: 구독 5개 + 발행 4개를 모두 담당하는 단일 노드. 상태 판단
  로직 없이 단순 비트-명령 변환만 수행한다(상태 판단은 별도 상태머신 담당).

## main()의 동작 순서
1. rclpy 초기화
2. `AppBridge` 노드 생성 — 이 시점에 파라미터 선언, 퍼블리셔/구독 등록 완료
3. `rclpy.spin()` — 콜백 이벤트 처리 루프(블로킹)
4. 종료 시 노드 정리
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class AppBridge(Node):
    def __init__(self):
        super().__init__('app_bridge_node')
        self.get_logger().info('App Bridge node has been started.')

        # 파라미터
        # KEY 모드에서 쓸 속도 단계 설정(느림/보통/빠름)과 회전 속도
        self.declare_parameter('low_speed', 0.1)
        self.declare_parameter('medium_speed', 0.2)
        self.declare_parameter('high_speed', 0.3)
        self.declare_parameter('turn_speed', 0.3)
        self.low_speed = self.get_parameter('low_speed').get_parameter_value().double_value
        self.medium_speed = self.get_parameter('medium_speed').get_parameter_value().double_value
        self.high_speed = self.get_parameter('high_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.current_speed = self.medium_speed  # 초기 속도는 "보통"

        # 발행 토픽
        self.state_command_pub = self.create_publisher(String, '/state_command', 10) # 모드 전환
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 속도 명령
        self.safety_pub = self.create_publisher(Int32, '/safety', 10) # 사람이 카메라 화면을 벗어난 뒤 운행 재개용 안전 버튼
        self.video_pub = self.create_publisher(Int32, '/video_enable', 10) # 영상 화면 on/off

        # 구독 토픽
        # 앱이 보내는 비트 문자열들
        self.create_subscription(String, '/app/sw_bits', self.sw_bits_callback, 10) # 모드 전환
        self.create_subscription(String, '/app/key_bits', self.key_bits_callback, 10) # KEY 모드 방향키
        self.create_subscription(String, '/app/speed_bits', self.speed_bits_callback, 10) # KEY 모드 속도 조절
        self.create_subscription(String, '/app/video_bit', self.video_bit_callback, 10) # 영상 화면 on/off
        self.create_subscription(String, '/app/safe_bit', self.safe_bit_callback, 10) # 안전 버튼

        # 상태 관리자(State Manager)가 발행하는 현재 로봇 상태
        self.create_subscription(String, '/robot_state', self.robot_state_callback, 10)

        # 동일 모드 명령 중복 발행 방지용
        self.last_mode_command = None

        # 초기 상태: STOP
        self.current_robot_state = "STOP"

        self.get_logger().info('[AppBridge] App Bridge is ready to translate app commands.')

        # sw_bits 문자열 -> 모드 이름 매핑(5비트 원-핫, 한 자리만 1)
        self.sw_command_map = {
                    "10000": "STOP",
                    "01000": "KEY",
                    "00100": "CAL",
                    "00010": "ALIGN",
                    "00001": "RUN"
                }

        # video_bit/safe_bit 상태 변화 시에만 로그를 찍기 위한 이전값 저장
        self.last_video_bit = None
        self.last_safe_bit = None

    # /robot_state 토픽을 받아 현재 로봇 상태를 갱신
    def robot_state_callback(self, msg):
        self.current_robot_state = msg.data

    # /app/sw_bits 토픽을 상태 전환 명령으로 변환
    def sw_bits_callback(self, msg):
        # 입력값 정리(앱이 따옴표를 포함해서 보내는 경우 대비)
        cleaned_bits = msg.data.replace('"', '')

        # 미리 만들어둔 매핑 테이블 사용
        if cleaned_bits in self.sw_command_map:
            command = self.sw_command_map[cleaned_bits]
            if command != self.last_mode_command:
                self.get_logger().info(f'[AppBridge] Received sw_bits "{cleaned_bits}", publishing command: "{command}"')
                command_msg = String()
                command_msg.data = command
                self.state_command_pub.publish(command_msg)
                self.last_mode_command = command
        else:
            self.get_logger().warn(f'[AppBridge] Unknown sw_bits: "{cleaned_bits}"', throttle_duration_sec=5.0)

    # /app/key_bits 토픽을 속도 명령(Twist)으로 변환
    def key_bits_callback(self, msg):
        # 'KEY' 상태에서는 모든 키 비트를 처리하고, 'STOP' 상태에서는 정지 비트만 처리

        cleaned_bits = msg.data.replace('"', '')
        twist_msg = Twist()

        # 정지 명령
        if cleaned_bits == "0000":
            # 'KEY' 또는 'STOP' 상태에서만 정지 명령 발행
            if self.current_robot_state in ['KEY', 'STOP']:
                self.cmd_vel_pub.publish(twist_msg)
            # RUN, CAL 상태에서는 수동 정지 명령 무시(해당 상태 자체 로직이 속도를 담당)
            return

        # 이동 명령
        # 'KEY' 상태에서만 이동 명령 처리
        if self.current_robot_state != 'KEY':
            self.get_logger().warn(
                f'[AppBridge] Ignoring movement key_bits in "{self.current_robot_state}" state.',
                throttle_duration_sec=5.0)
            return

        # 'KEY' 상태에서 이동 명령 매핑(4비트 원-핫)
        if cleaned_bits == "1000":  # 전진
            twist_msg.linear.x = self.current_speed
        elif cleaned_bits == "0100":  # 후진
            twist_msg.linear.x = -self.current_speed
        elif cleaned_bits == "0010":  # 좌회전
            twist_msg.angular.z = self.turn_speed
        elif cleaned_bits == "0001":  # 우회전
            twist_msg.angular.z = -self.turn_speed
        else:
            self.get_logger().warn(
                f'[AppBridge] Received unknown key_bits: "{cleaned_bits}"',
                throttle_duration_sec=5.0)
            return
        self.cmd_vel_pub.publish(twist_msg)

    # /app/speed_bits 토픽을 속도 설정값으로 변환
    def speed_bits_callback(self, msg):
        cleaned_bits = msg.data.replace('"', '')
        new_speed = self.medium_speed
        if cleaned_bits == "100":  # 느림
            new_speed = self.low_speed
        elif cleaned_bits == "010":  # 보통
            new_speed = self.medium_speed
        elif cleaned_bits == "001":  # 빠름
            new_speed = self.high_speed
        else:
            self.get_logger().warn(f'[AppBridge] Received unknown speed_bits: "{cleaned_bits}"', throttle_duration_sec=5.0)

        # 실제로 값이 바뀐 경우에만 갱신(중복 로그 방지)
        if new_speed != self.current_speed:
            self.current_speed = new_speed
            self.get_logger().info(f'[AppBridge] Speed set to {self.current_speed:.2f} m/s')

    # /app/video_bit 토픽을 영상 on/off 명령으로 변환
    def video_bit_callback(self, msg):
        cleaned_bits = msg.data.replace('"', '').strip()
        if cleaned_bits not in ("0", "1"):
            self.get_logger().warn(f'invalid video_bit: {msg.data}', throttle_duration_sec=5.0)
            return

        # 값이 바뀔 때만 로그 출력
        if cleaned_bits != self.last_video_bit:
            if cleaned_bits == "1":
                self.get_logger().info(f'[AppBridge] Video ON')
            else:
                self.get_logger().info(f'[AppBridge] Video OFF')
            self.last_video_bit = cleaned_bits
        out = Int32()
        out.data = int(cleaned_bits)
        self.video_pub.publish(out)

    # /app/safe_bit 토픽을 안전 버튼 명령으로 변환
    def safe_bit_callback(self, msg):
        cleaned_bits = msg.data.replace('"', '').strip()

        if cleaned_bits not in ("0", "1"):
            self.get_logger().warn(f'invalid safe_bit: {msg.data}', throttle_duration_sec=5.0)
            return

        # 값이 바뀔 때만 로그 출력
        if cleaned_bits != self.last_safe_bit:
            if cleaned_bits == "1":
                self.get_logger().info('[AppBridge] Safety ENABLED')
            self.last_safe_bit = cleaned_bits

        # safe_bit이 1이면(사람이 카메라 화면을 벗어남 해제) 0을 발행해 운행 재개 신호를 보냄
        if cleaned_bits == "1":
            out = Int32()
            out.data = 0
            self.safety_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    app_bridge = AppBridge()
    rclpy.spin(app_bridge)
    app_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

