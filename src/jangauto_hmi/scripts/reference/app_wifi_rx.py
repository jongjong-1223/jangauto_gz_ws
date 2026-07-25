#!/usr/bin/env python3
"""앱 WiFi 수신기: 모바일 앱이 보내는 JSON 데이터를 받아 ROS2 토픽으로 발행.

## 역할
- (참고용 미사용 코드) 예전 HTTP 기반 앱 프로토콜에서 쓰던 노드다. 현재는
  `app_websocket_bridge.py`(웹소켓 기반 신규 프로토콜)가 이 역할을 대신하며,
  이 파일은 빌드/실행 대상이 아니다.
- 자체 HTTP 서버(`http.server`)를 별도 스레드로 띄워 앱의 POST 요청을 받는다.
  ROS2 노드 자체는 rclpy 콜백/타이머 처리만 하고, HTTP 요청 처리는 완전히
  별도 스레드에서 동작한다(`AppDataHandler`).
- POST 엔드포인트 3종:
  - `/to_rasp`: 앱이 보낸 비트 필드(JSON)를 그대로 `/app/*` 토픽들로 재발행.
  - `/poweroff`: 라즈베리파이 전원을 끈다(sudoers에 NOPASSWD 설정 필요).
  - `/global_path`: 앱이 보낸 밭 치수 파라미터로 왕복 경로(보스트로페돈,
    boustrophedon)를 계산해 `nav_msgs/Path`로 발행하고, 이후 1Hz로 재발행한다
    (늦게 구독하는 PPC/EKF 노드도 최신 경로를 받을 수 있도록).

## 클래스 구성
- `AppDataHandler`: `BaseHTTPRequestHandler`를 상속한 HTTP 요청 처리기.
  요청마다 새 인스턴스가 생성되며, `ros_node` 참조를 통해 실제 처리를 위임한다.
- `AppWiFiReceiver`: ROS2 노드 본체. 퍼블리셔 등록, HTTP 서버 기동/종료,
  앱 데이터 -> 토픽 변환, 경로 계산/재발행 로직을 담당한다.

## main()의 동작 순서
1. rclpy 초기화
2. `AppWiFiReceiver` 노드 생성 — 퍼블리셔 등록, HTTP 서버를 별도 스레드로 기동,
   1Hz 경로 재발행 타이머 등록까지 이 시점에 완료
3. `rclpy.spin()` — ROS2 콜백/타이머 처리 루프(블로킹). 실제 HTTP 요청은
   별도 스레드에서 비동기로 처리되고, `ros_node.process_app_data()` 등을
   통해 이 스레드로 결과(토픽 발행)만 넘어온다.
4. Ctrl+C 시 HTTP 서버까지 함께 정리 후 종료
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json
import threading
import subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler


class AppDataHandler(BaseHTTPRequestHandler):
    """HTTP 요청 처리기. 요청 1건당 인스턴스 1개가 생성되므로 상태를 들고
    있지 않고, `ros_node`(AppWiFiReceiver)에 실제 처리를 위임하는 얇은 어댑터다."""
    def __init__(self, *args, ros_node=None, **kwargs):
        self.ros_node = ros_node
        super().__init__(*args, **kwargs)

    def _read_json_body(self):
        """Content-Length만큼 요청 바디를 읽어 JSON으로 파싱. 바디가 없으면 빈 dict."""
        content_length = int(self.headers.get('Content-Length', 0))
        if content_length <= 0:
            return {}
        post_data = self.rfile.read(content_length)
        return json.loads(post_data.decode('utf-8'))

    def _send_json(self, code, payload):
        """JSON 응답 전송. CORS 허용 헤더를 항상 포함(앱이 브라우저 기반일 수 있어서)."""
        body = json.dumps(payload).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    # 모바일 앱이 보내는 HTTP POST 요청 처리
    def do_POST(self):
        # ---- 기존: 명령 비트 ----
        if self.path == '/to_rasp':
            try:
                json_data = self._read_json_body()
                if self.ros_node:
                    self.ros_node.process_app_data(json_data)
                self._send_json(200, {"status": "ok"})
            except json.JSONDecodeError as e:
                self.send_error(400, f"Invalid JSON: {e}")
            except Exception as e:
                self.send_error(500, f"Server error: {e}")

        # ---- 신규: 라즈베리파이 전원 끄기 ----
        elif self.path == '/poweroff':
            try:
                # 응답을 먼저 보내고, 응답이 flush될 시간을 준 뒤 잠시 후 실제로 종료한다.
                self._send_json(200, {"status": "poweroff_scheduled"})
                if self.ros_node:
                    self.ros_node.get_logger().warn('[WIFI_RX] Poweroff requested from app.')
                    threading.Timer(1.0, self.ros_node.trigger_poweroff).start()
            except Exception as e:
                self.send_error(500, f"Server error: {e}")

        # ---- 신규: 전역 경로 생성 및 발행 ----
        elif self.path == '/global_path':
            try:
                params = self._read_json_body()
                if self.ros_node:
                    num = self.ros_node.set_global_path(params)
                    self._send_json(200, {"status": "ok", "num_waypoints": num})
                else:
                    self._send_json(500, {"status": "error", "msg": "ROS node not available"})
            except json.JSONDecodeError as e:
                self.send_error(400, f"Invalid JSON: {e}")
            except Exception as e:
                self.send_error(500, f"Server error: {e}")

        else:
            self.send_error(404, "Not Found")

    # 브라우저의 CORS preflight 요청(OPTIONS) 지원
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def log_message(self, format, *args):
        # BaseHTTPRequestHandler 기본 동작(stderr 직접 출력) 대신 ROS 로거로 우회
        if self.ros_node:
            self.ros_node.get_logger().debug(f"HTTP: {format % args}")


class AppWiFiReceiver(Node):
    def __init__(self):
        super().__init__('app_wifi_receiver')

        # 퍼블리셔 (기존): 앱이 보낸 비트 필드를 그대로 각 토픽으로 재발행
        self.sw_bits_pub = self.create_publisher(String, '/app/sw_bits', 10)
        self.key_bits_pub = self.create_publisher(String, '/app/key_bits', 10)
        self.speed_bits_pub = self.create_publisher(String, '/app/speed_bits', 10)
        self.raw_app_data_pub = self.create_publisher(String, '/app/raw_data', 10)
        self.video_bit_pub = self.create_publisher(String, '/app/video_bit', 10)
        self.safe_bit_pub = self.create_publisher(String, '/app/safe_bit', 10)

        # 퍼블리셔 (신규): 앱에서 생성 요청한 전역 경로
        self.global_path_pub = self.create_publisher(Path, '/global_path', 10)
        self.current_path_msg = None
        # 최신 경로를 1Hz로 계속 재발행 — 늦게 구독하는 노드(PPC/EKF)도 받을 수 있도록
        self.create_timer(1.0, self._republish_path)

        # HTTP 서버 설정
        self.declare_parameter('port', 8889)
        self.declare_parameter('host', '0.0.0.0')
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value

        # 통계용
        self.data_count = 0
        self.last_data_time = None

        self.start_http_server()
        self.get_logger().info(f'[WIFI_RX] App WiFi Receiver started on {self.host}:{self.port}')
        self.get_logger().info('[WIFI_RX] POST endpoints: /to_rasp  /poweroff  /global_path')

    # ----------------------------------------------------------------- HTTP
    def start_http_server(self):
        """HTTP 서버를 데몬 스레드로 기동. rclpy.spin()과 별개 스레드에서
        요청을 받아야 ROS 콜백 루프를 막지 않는다."""
        def handler_factory(*args, **kwargs):
            return AppDataHandler(*args, ros_node=self, **kwargs)
        try:
            self.server = HTTPServer((self.host, self.port), handler_factory)
            self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.server_thread.start()
            self.get_logger().info(f'[WIFI_RX] HTTP Server listening on {self.host}:{self.port}')
        except Exception as e:
            self.get_logger().error(f'[WIFI_RX] Failed to start HTTP server: {e}')

    # ------------------------------------------------------- command bits
    def process_app_data(self, json_data):
        """`/to_rasp`로 들어온 JSON을 필드별로 대응하는 `/app/*` 토픽에 재발행.
        존재하는 필드만 선택적으로 발행한다(앱이 매번 전체 필드를 보내지
        않을 수 있어서, 없는 필드는 건드리지 않고 넘어간다)."""
        try:
            self.data_count += 1
            current_time = self.get_clock().now()

            if 'sw_bits' in json_data:
                m = String(); m.data = str(json_data['sw_bits']); self.sw_bits_pub.publish(m)
            if 'key_bits' in json_data:
                m = String(); m.data = str(json_data['key_bits']); self.key_bits_pub.publish(m)
            if 'speed_bits' in json_data:
                m = String(); m.data = str(json_data['speed_bits']); self.speed_bits_pub.publish(m)
            if 'video_bit' in json_data:
                m = String(); m.data = str(json_data['video_bit']); self.video_bit_pub.publish(m)
            if 'safe_bit' in json_data:
                m = String(); m.data = str(json_data['safe_bit']); self.safe_bit_pub.publish(m)

            self.last_data_time = current_time
        except Exception as e:
            self.get_logger().error(f'[WIFI_RX] Error processing app data: {e}')

    # ------------------------------------------------------------ poweroff
    def trigger_poweroff(self):
        """실제 전원 종료 실행. `sudo poweroff`가 비밀번호 없이 실행되려면
        sudoers에 NOPASSWD 설정이 되어 있어야 한다(SETUP_poweroff_sudoers.md 참고)."""
        try:
            self.get_logger().warn('[WIFI_RX] Executing: sudo poweroff')
            subprocess.Popen(['sudo', 'poweroff'])
        except Exception as e:
            self.get_logger().error(f'[WIFI_RX] poweroff failed: {e}')

    # --------------------------------------------------------- global path
    def set_global_path(self, params):
        """앱 파라미터로 왕복(보스트로페돈) 웨이포인트를 계산해 nav_msgs/Path로 발행."""
        waypoints = self.compute_boustrophedon(params)
        self.current_path_msg = self.build_path_msg(waypoints)
        self._republish_path()  # 즉시 1회 발행(1Hz 타이머를 기다리지 않고 바로 반영)
        self.get_logger().info(f'[WIFI_RX] Global path set: {len(waypoints)} waypoints.')
        return len(waypoints)

    def compute_boustrophedon(self, params):
        """`global_path_generator.generate_boustrophedon_path()`를 input()/matplotlib
        의존 없이 이식한 버전. 밭 영역을 크롭 라인(crop_x) 간격으로 나눠
        위/아래를 지그재그로 오가는 왕복 경로의 코너 좌표들을 계산한다.

        Args:
            params: 앱이 보낸 dict. map_top_y/map_bottom_y(지도 상하 경계),
                field_top_y/field_bottom_y(실제 밭 상하 경계), crop_x(작물
                줄 x좌표들, 공백 구분 문자열 또는 리스트)를 담고 있으며
                누락 시 기본값을 사용한다.
        """
        def f(key, default):
            try:
                return float(params.get(key, default))
            except (TypeError, ValueError):
                return float(default)

        map_top_y    = f('map_top_y', 10.0)
        map_bottom_y = f('map_bottom_y', -10.0)
        field_top_y    = f('field_top_y', 8.0)
        field_bottom_y = f('field_bottom_y', -8.0)

        crop_raw = params.get('crop_x', "-6.6 -3.3 0 3.3 6.6")
        if isinstance(crop_raw, (list, tuple)):
            crop_x_list = [float(x) for x in crop_raw]
        else:
            crop_x_list = [float(x) for x in str(crop_raw).split()]
        crop_x_list.sort()

        # 지도 경계와 실제 밭 경계의 중간값을 왕복 경로의 상/하단 y좌표로 사용
        # (밭 끝단에 딱 붙지 않고 여유를 두기 위함)
        waypoint_top_y = (map_top_y + field_top_y) / 2.0
        waypoint_bottom_y = (map_bottom_y + field_bottom_y) / 2.0

        if len(crop_x_list) > 1:
            interval = crop_x_list[1] - crop_x_list[0]
        else:
            interval = 5.0

        # 각 작물 줄 사이 중간 지점 + 양 끝은 바깥으로 반 간격 확장한 x좌표 목록
        path_x_list = []
        path_x_list.append(crop_x_list[0] - interval / 2.0)
        for i in range(len(crop_x_list) - 1):
            path_x_list.append((crop_x_list[i] + crop_x_list[i + 1]) / 2.0)
        path_x_list.append(crop_x_list[-1] + interval / 2.0)

        # x좌표를 순서대로 훑으며 위/아래를 번갈아 왕복(보스트로페돈 패턴)
        corner_waypoints = []
        is_top = False
        for x in path_x_list:
            if not is_top:
                corner_waypoints.append((x, waypoint_bottom_y))
                corner_waypoints.append((x, waypoint_top_y))
            else:
                corner_waypoints.append((x, waypoint_top_y))
                corner_waypoints.append((x, waypoint_bottom_y))
            is_top = not is_top
        return corner_waypoints

    def build_path_msg(self, waypoints):
        """(x, y) 튜플 목록을 map 프레임 기준 nav_msgs/Path 메시지로 변환."""
        path = Path()
        path.header.frame_id = 'map'
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _republish_path(self):
        """현재 저장된 경로의 타임스탬프만 갱신해 재발행. 1Hz 타이머와
        set_global_path()의 즉시 발행 양쪽에서 호출된다."""
        if self.current_path_msg is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.current_path_msg.header.stamp = stamp
        for pose in self.current_path_msg.poses:
            pose.header.stamp = stamp
        self.global_path_pub.publish(self.current_path_msg)

    # ------------------------------------------------------------- cleanup
    def destroy_node(self):
        self.get_logger().info('[WIFI_RX] Shutting down App WiFi Receiver...')
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AppWiFiReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
